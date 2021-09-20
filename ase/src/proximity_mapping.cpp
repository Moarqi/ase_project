// ============================================================================
// Name        : mapping_with_known_poses.cpp
// Author      : Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
//               Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : The mapping exercise
// ============================================================================

// amiro
#include <amiro_msgs/UInt16MultiArrayStamped.h>
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <stack>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Helper
#include <omp.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "lib/Vector2D.hpp"

using namespace std;

// program name
const string programName = "mapping_with_known_poses";

// Ros Listener Topic and publisher
string rosListenerTopicScan;
string rosListenerTopicOdom;
string rosPublisherTopicMap;
string rosPublisherTopicCmd;

ros::Publisher pub;
ros::Publisher twistPub;

// Map variables
static int dimensionX = 4;          // (m)
static int dimensionY = 4;          // (m)
static double mapResolution = 0.05; // (m)
static int mapDimensionX;           // (cells)
static int mapDimensionY;           // (cells)
static int mapIterRange;            // (1)
#define mattype uint16_t // CV_8U == uchar, CV_16U == short, CV_32FC1 == float
#define mattypecv CV_16U
#define maxValue 65535
static cv::Mat hit, miss;
static nav_msgs::OccupancyGrid ogm;

// Robot and sensor setup
double robotOffsetX = dimensionX / 2.0f;
double robotOffsetY = dimensionY / 2.0f;

const unsigned int ALIGNED_THRESHOLD = 300;
const unsigned int TARGET_DISTANCE = 58000;
const unsigned int MIN_SENSOR_READ = 40000;
const unsigned int MAX_PATH_LENGTH = 1000;
const float PHI_OFFSET_FACTOR = 0.9; // 0.9 seems to be good
const float RIGHT_ANGLE_VALUES[5] = { 0.0, M_PI_2, M_PI, 3 * M_PI_2, 2 * M_PI };

const float STRIP_LIMIT_INCREMENT = 0.8;

int mapEvaluationIter = 0;

static struct RingSensors
{
  uint sse;
  uint ese;
  uint ene;
  uint nne;
  uint nnw;
  uint wnw;
  uint wsw;
  uint ssw;
} ringSensors;

enum TraceState
{
  BOUNDARY,
  IDENTIFY
};

enum MappingState
{
  ALIGN_BOUNDARY,
  ALIGN_GOAL,
  ALIGN,
  FIND_CORNER,
  FIND_WALL,
  INIT,
  MOVE_GOAL,
  RESET,
  SET_OUTER_CORNER,
  SET_DISTANCE_CORNER,
  SET_DISTANCE,
  STOP,
  STRIP_CORRECT,
  STRIP_CROSS,
  STRIP_MAP,
  STRIP_TRACE,
  TRACE_BOUNDARY,
  TURN_90,
};

enum WallDirection
{
  BACK,
  FRONT,
  LEFT,
  RIGHT,
  UNKNOWN,
};

struct Point
{
  Vector2D vec;
  float phi;
  WallDirection wallDir;
};

// struct BoundingBox
// {
//   Point lower;
//   Point upper;
// };

MappingState state = MappingState::INIT;
std::stack<MappingState> stateStack;

Point odom = Point{ .vec = Vector2D(), .phi = 0.0 };

WallDirection wallDir = WallDirection::UNKNOWN;
WallDirection previousWallDirection = WallDirection::UNKNOWN;

TraceState traceState = TraceState::BOUNDARY;

Vector2D corners[100];
int cornerIndex = 0;

// BoundingBox* workMapArea = nullptr;
// BoundingBox mapAreas[10];
int mapAreaIndex = 0;
int mapAreaWorkIndex = 0;

Point goal;
bool goalSet = false;
float goalETA;
ros::Time goalStart;

Vector2D recordedPath[MAX_PATH_LENGTH];
unsigned int recordedPathLength = 0;
unsigned int pathIter = 0;

Vector2D findCornerStart;

Point identifyObjectStart;
int identifyObjectConsecutiveLeftTurns = 0;

geometry_msgs::Twist lastTwist;
const auto minROSTimePoint = ros::Time(0); // TODO: ew, please no eq check
auto lastROSPublish = minROSTimePoint;

float stripLimit = 0.0;
float stripMapDistance = 0.0;
Vector2D* stripRootCorner = nullptr;

std::string
printWallDir(WallDirection _wallDir = wallDir)
{
  switch (_wallDir) {
    case WallDirection::BACK:
      return "BACK";
    case WallDirection::FRONT:
      return "FRONT";
    case WallDirection::LEFT:
      return "LEFT";
    case WallDirection::RIGHT:
      return "RIGHT";
    case WallDirection::UNKNOWN:
      return "UNKNOWN";
    default:
      return "";
  }
}

std::string
printState(MappingState _state = state)
{
  switch (_state) {
    case MappingState::ALIGN_BOUNDARY:
      return "ALIGN_BOUNDARY";
    case MappingState::ALIGN_GOAL:
      return "ALIGN_GOAL";
    case MappingState::ALIGN:
      return "ALIGN";
    case MappingState::FIND_CORNER:
      return "FIND_CORNER";
    case MappingState::FIND_WALL:
      return "FIND_WALL";
    case MappingState::INIT:
      return "INIT";
    case MappingState::MOVE_GOAL:
      return "MOVE_GOAL";
    case MappingState::RESET:
      return "RESET";
    case MappingState::SET_OUTER_CORNER:
      return "SET_OUTER_CORNER";
    case MappingState::SET_DISTANCE_CORNER:
      return "SET_DISTANCE_CORNER";
    case MappingState::SET_DISTANCE:
      return "SET_DISTANCE";
    case MappingState::STOP:
      return "STOP";
    case MappingState::STRIP_CORRECT:
      return "STRIP_CORRECT";
    case MappingState::STRIP_CROSS:
      return "STRIP_CROSS";
    case MappingState::STRIP_MAP:
      return "STRIP_MAP";
    case MappingState::STRIP_TRACE:
      return "STRIP_TRACE";
    case MappingState::TRACE_BOUNDARY:
      return "TRACE_BOUNDARY";
    case MappingState::TURN_90:
      return "TURN_90";
    default:
      return "";
  }
}

// void
// prepareMapArea()
// {
//   for (int i = 0; i <= cornerIndex; i += 3) {
//     auto upper = i + 2 > cornerIndex ? corners[0] : corners[i + 2];
//     auto bb = BoundingBox{ .lower = corners[i], .upper = upper };

//     mapAreas[mapAreaIndex++] = bb;
//   }

//   ROS_INFO("created %d mapping areas!", mapAreaIndex);
// }

void
setWallDir(WallDirection _wallDir)
{
  if (_wallDir == wallDir)
    return;

  if (wallDir != WallDirection::BACK)
    previousWallDirection = wallDir;

  wallDir = _wallDir;
  ROS_INFO("wallDir updated %s", printWallDir(wallDir).c_str());
}

void
setState(MappingState _state)
{
  state = _state;
  ROS_INFO("state updated %s", printState().c_str());
}

void
printStateStackTop()
{
  if (!stateStack.empty()) {
    ROS_INFO("top of stack: %s", printState(stateStack.top()).c_str());
  }
}

void
align()
{
  auto diff = 100.0;
  auto angle = 0.0;

  for (int i = 0; i < 5; i++) {
    auto currentDiff =
      abs(RIGHT_ANGLE_VALUES[i] - fmod(odom.phi + 2 * M_PI, 2 * M_PI));
    if (currentDiff < diff) {
      diff = currentDiff;
      angle = RIGHT_ANGLE_VALUES[i];
    }
  }

  odom.phi = fmod(angle, 2 * M_PI);
}

void
turnAround()
{
  if (wallDir == WallDirection::LEFT)
    setWallDir(WallDirection::RIGHT);
  else if (wallDir == WallDirection::RIGHT)
    setWallDir(WallDirection::LEFT);
  else {
    if (previousWallDirection == WallDirection::UNKNOWN ||
        previousWallDirection == WallDirection::BACK)
      ROS_WARN("can not turn around if wallDir unknown or back!");
    else
      setWallDir(previousWallDirection);
  }
}

bool
isNearBoundary(Vector2D p)
{
  // distance to line segment, reference
  // https://www.iquilezles.org/www/articles/distfunctions2d/distfunctions2d.htm
  for (int i = 0; i < cornerIndex; i++) {
    auto A = corners[i];
    auto B = i + 1 < cornerIndex ? corners[i + 1] : corners[0];

    auto BA = B - A;
    auto PA = p - A;

    auto hUnclamped = PA * BA / (BA * BA);
    auto h = hUnclamped < 0.0 ? 0.0 : hUnclamped > 1.0 ? 1.0 : hUnclamped;

    auto distance = (PA - BA * h).sqLength();

    ROS_INFO("isNearBoundary: distance to (%d, %d): %f",
             i,
             i + 1 < cornerIndex ? i + 1 : 0,
             distance);

    if (distance < 0.01) {
      return true;
    }
  }

  return false;
}

void
ringSubCallback(const amiro_msgs::UInt16MultiArrayStamped::ConstPtr ring)
{
  if (ring->array.data.size() == 8) {
    ringSensors.ssw = ring->array.data[0];
    ringSensors.wsw = ring->array.data[1];
    ringSensors.wnw = ring->array.data[2];
    ringSensors.nnw = ring->array.data[3];
    ringSensors.nne = ring->array.data[4];
    ringSensors.ene = ring->array.data[5];
    ringSensors.ese = ring->array.data[6];
    ringSensors.sse = ring->array.data[7];
  }
}

// Format the inner representation as OGM and send it
inline void
showMap()
{
// Copy the probabilistic map data [0 .. 1] to the occupancy grid map format [0
// .. 100] Reference:
// http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
#pragma omp parallel for
  for (int yi = 0; yi < mapDimensionY; yi++) {
    for (int xi = 0; xi < mapDimensionX; xi++) {
      const char result =
        char((hit.at<mattype>(yi, xi) /
              (float)(hit.at<mattype>(yi, xi) + miss.at<mattype>(yi, xi))) *
             100.0);
      ogm.data.at(yi * mapDimensionX + xi) = result;
    }
  }
  pub.publish(ogm);
}

void
recordPath()
{
  if (!(pathIter++ % 10)) {
    if (!(recordedPathLength < MAX_PATH_LENGTH)) {
      ROS_WARN("cannot record path: MAX_PATH_LENGTH exceeded!");
      return;
    }
    if (recordedPathLength) {
      auto prevRecord = recordedPath[recordedPathLength - 1];
      // discard records with very little change..
      if (abs(prevRecord.x - odom.vec.x) + abs(prevRecord.y - odom.vec.y) <
          0.01)
        return;
    }

    recordedPath[recordedPathLength++] = Vector2D(odom.vec);
  }
}

// TODO: add hit for walls/obstacles..
void
writePath(Vector2D goalDeviation)
{
  if (recordedPathLength < 1)
    return;

  auto deviationFraction = Vector2D(goalDeviation) / (float)recordedPathLength;

  ROS_INFO("deviation and fraction: (%f, %f) -> (%f, %f)",
           goalDeviation.x,
           goalDeviation.y,
           deviationFraction.x,
           deviationFraction.y);

  auto xDiff = recordedPath[recordedPathLength - 1].x - recordedPath[0].x;
  auto yDiff = recordedPath[recordedPathLength - 1].y - recordedPath[0].y;

  for (unsigned int i = 0; i < recordedPathLength; i++) {
    auto point = recordedPath[i];

    // TODO: determine direction of travel and wall direction respectively..
    auto correctedPoint = point + deviationFraction * i;
    int xi = round((correctedPoint.x + robotOffsetX) / mapResolution);
    int yi = round((correctedPoint.y + robotOffsetY) / mapResolution);

    miss.at<mattype>(yi, xi) = miss.at<mattype>(yi, xi) + 5;
  }

  odom.vec += goalDeviation;

  // no need to clear memory..
  recordedPathLength = 0;
}

void
publishTwist(geometry_msgs::Twist* twist)
{
  auto ROSNow = ros::Time::now();
  if (lastROSPublish == minROSTimePoint)
    lastROSPublish = ROSNow;

  auto nSeconds = (ROSNow - lastROSPublish).toNSec();
  if (nSeconds > 0) {
    double seconds = nSeconds / 1e9;

    auto deltaPhi = seconds * lastTwist.angular.z * PHI_OFFSET_FACTOR;
    auto deltaX = seconds * lastTwist.linear.x * cos(odom.phi);
    auto deltaY = seconds * lastTwist.linear.x * sin(odom.phi);

    // error terms are added in writePath()
    odom.vec.x += deltaX;
    odom.vec.y += deltaY;
    odom.phi += deltaPhi;
    odom.phi = fmod(odom.phi + 2 * M_PI, 2 * M_PI);

    if (state == MappingState::STRIP_TRACE) {
      // TODO: switch to squared distance?
      stripMapDistance += sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    }
  }

  twistPub.publish(*twist);
  lastROSPublish = ros::Time::now();
  lastTwist = *twist;

  recordPath();
}

void
setDistance(geometry_msgs::Twist* twist)
{
  // TODO: might turn back to using both..
  // auto distance = (ringSensors.nne + ringSensors.nnw) / 2;
  auto distance =
    ringSensors.nne > ringSensors.nnw ? ringSensors.nne : ringSensors.nnw;

  if (abs((int)distance - (int)TARGET_DISTANCE) < ALIGNED_THRESHOLD) {
    align();
    wallDir = previousWallDirection != WallDirection::UNKNOWN
                ? previousWallDirection
                : WallDirection::LEFT;

    if (stateStack.empty()) {
      setState(MappingState::ALIGN_BOUNDARY);
    } else {
      setState(stateStack.top());
      stateStack.pop();
    }

    return;
  }

  if (distance < TARGET_DISTANCE)
    twist->linear.x = 0.01;
  else
    twist->linear.x = -0.01;
}

void
findWall(geometry_msgs::Twist* twist)
{
  unsigned int sensorReadout = UINT16_MAX;
  switch (wallDir) {
    case WallDirection::LEFT:
      sensorReadout = (ringSensors.wnw + ringSensors.wsw) / 2.0;
      break;
    case WallDirection::RIGHT:
      sensorReadout = (ringSensors.ene + ringSensors.ese) / 2.0;
      break;
    default:
      ROS_WARN("warning findWall wallDir invalid");
      break;
  }

  if (sensorReadout > TARGET_DISTANCE - 15000) {
    if (stateStack.empty()) {
      setState(MappingState::TRACE_BOUNDARY);
    } else {
      setState(stateStack.top());
      stateStack.pop();
    }
  }

  twist->linear.x = traceState == TraceState::BOUNDARY ? 0.1 : 0.04;
}

void
turn90Deg()
{
  auto alpha = 0.0;
  switch (wallDir) {
    case WallDirection::LEFT:
      alpha = odom.phi + M_PI_2;
      break;
    case WallDirection::RIGHT:
      alpha = odom.phi - M_PI_2;
    default:
      ROS_WARN("error turn90Deg invalid wallDir!");
      break;
  }

  goal.phi = fmod(alpha + 2 * M_PI, 2 * M_PI);
  setState(MappingState::ALIGN);
}

void
setOuterCorner()
{
  if (cornerIndex % 2 == 0) {
    for (int i = 0; i <= cornerIndex; i++) {
      auto point = corners[i];
      auto distance = abs(odom.vec.x - point.x) + abs(odom.vec.y - point.y);
      ROS_INFO("distance to %dth corner: %f", i, distance);
      if (stripRootCorner == nullptr && distance < 0.2) {
        writePath(Vector2D(point.x - odom.vec.x, point.y - odom.vec.y));

        stripRootCorner = &corners[i + 2];
        // goal.vec = *stripRootCorner;
        // goalSet = true;
        
        stateStack.push(MappingState::TRACE_BOUNDARY);
        stateStack.push(MappingState::FIND_WALL);
        setState(MappingState::TURN_90);
        return;
      }
    }
  }

  if (traceState == TraceState::BOUNDARY) {
    ROS_INFO(
      "marked %dth corner at (%f, %f)", cornerIndex, odom.vec.x, odom.vec.y);
    corners[cornerIndex++] = Vector2D(odom.vec);

    auto prevCorner = cornerIndex > 1 ? corners[cornerIndex - 2] : Vector2D();
    auto currentCorner = corners[cornerIndex - 1];

    if (abs(prevCorner.x - currentCorner.x) < 0.5)
      writePath(Vector2D(prevCorner.x - currentCorner.x, 0.0));
    else
      writePath(Vector2D(0.0, prevCorner.y - currentCorner.y));
  } else {
    identifyObjectConsecutiveLeftTurns++;
    ROS_INFO("corner at (%f, %f)", odom.vec.x, odom.vec.y);
  }

  stateStack.push(MappingState::TRACE_BOUNDARY);
  stateStack.push(MappingState::FIND_WALL);
  setState(MappingState::TURN_90);
}

void
setDistanceCorner(geometry_msgs::Twist* twist)
{
  setDistance(twist);

  if (state == MappingState::ALIGN_BOUNDARY) {
    if (cornerIndex % 2 == 0) {
      for (int i = 0; i <= cornerIndex; i++) {
        auto point = &corners[i];
        auto distance = abs(odom.vec.x - point->x) + abs(odom.vec.y - point->y);
        ROS_INFO("distance to %dth corner: %f", i, distance);
        if (distance < 0.2) {
          writePath(Vector2D(point->x - odom.vec.x, point->y - odom.vec.y));

          odom.vec.x = point->x;
          odom.vec.y = point->y;
          // if at first corner again..
          if (stripRootCorner == nullptr && i == 0) {
            stripRootCorner = point;

            turnAround();
            stateStack.push(MappingState::STRIP_MAP);
          } else if (stripRootCorner != nullptr && point == stripRootCorner) {
            stateStack.push(MappingState::STRIP_MAP);
          }

          return;
        }
      }
    }

    if (traceState == TraceState::BOUNDARY) {
      ROS_INFO(
        "marked %dth corner at (%f, %f)", cornerIndex, odom.vec.x, odom.vec.y);
      corners[cornerIndex++] = Vector2D(odom.vec);

      auto prevCorner = cornerIndex > 1 ? corners[cornerIndex - 2] : Vector2D();
      auto currentCorner = corners[cornerIndex - 1];

      if (abs(prevCorner.x - currentCorner.x) < 0.5)
        writePath(Vector2D(prevCorner.x - currentCorner.x, 0.0));
      else
        writePath(Vector2D(0.0, prevCorner.y - currentCorner.y));
    } else {
      identifyObjectConsecutiveLeftTurns = 0;
      ROS_INFO("corner at (%f, %f)", odom.vec.x, odom.vec.y);
    }
  }
}

void
traceBoundary(geometry_msgs::Twist* twist, float _limit = -1)
{
  if (ringSensors.nne > TARGET_DISTANCE) {
    if (_limit > 0 && stripMapDistance < _limit) {
      setState(MappingState::STOP);
      ROS_INFO("assume mapping done!");
      return;
    }

    setState(MappingState::SET_DISTANCE_CORNER);
    return;
  }

  if (_limit > 0 && stripMapDistance > _limit) {
    ROS_INFO("trace limit (%f) reached!", _limit);
    align();
    setState(MappingState::ALIGN_BOUNDARY);
    setWallDir(WallDirection::BACK);
    stateStack.push(MappingState::STRIP_CROSS);
    return;
  }

  if (traceState == TraceState::IDENTIFY && identifyObjectConsecutiveLeftTurns > 1) {    
    auto diffVec = odom.vec - identifyObjectStart.vec;

    // TODO: add condition that checks wether the object was identified correctly..
    if (abs(diffVec.x) < 0.01 || abs(diffVec.y) < 0.01) {
      stateStack.push(MappingState::STRIP_CROSS);
      goal.phi = identifyObjectStart.phi;
      wallDir = WallDirection::BACK;
      setState(MappingState::ALIGN);
      traceState = TraceState::BOUNDARY;
    }
  }

  // TODO: add angular error for deviation from target distance.. PID?
  switch (wallDir) {
    case WallDirection::LEFT:
      if (ringSensors.wnw < TARGET_DISTANCE - 20000 &&
          ringSensors.wsw < TARGET_DISTANCE - 20000) {
        setState(MappingState::SET_OUTER_CORNER);
        return;
      }

      twist->angular.z = ringSensors.wnw > ringSensors.wsw ? -0.03 : 0.03;
      break;
    case WallDirection::RIGHT:
      if (ringSensors.ene < TARGET_DISTANCE - 20000 &&
          ringSensors.ese < TARGET_DISTANCE - 20000) {
        setState(MappingState::SET_OUTER_CORNER);
        return;
      }

      twist->angular.z = ringSensors.ene > ringSensors.ese ? 0.03 : -0.03;
      break;
    default:
      break;
  }

  twist->linear.x = traceState == TraceState::BOUNDARY ? 0.1 : 0.04;
}

bool
wallIsNear()
{
  return (ringSensors.ene > MIN_SENSOR_READ ||
          ringSensors.ese > MIN_SENSOR_READ ||
          ringSensors.nne > MIN_SENSOR_READ ||
          ringSensors.nnw > MIN_SENSOR_READ ||
          ringSensors.sse > MIN_SENSOR_READ ||
          ringSensors.ssw > MIN_SENSOR_READ ||
          ringSensors.wnw > MIN_SENSOR_READ ||
          ringSensors.wsw > MIN_SENSOR_READ);
}

void
alignWithBoundary(geometry_msgs::Twist* twist, bool _align = true)
{
  if (!wallIsNear()) {
    if (previousWallDirection == WallDirection::BACK) {
      turn90Deg();
    } else {
      ROS_ERROR("cannot align with boundary if no boundary near and previous "
                "wall direction not back..");
    }

    return;
  }

  unsigned int sensorDiff;
  bool freeSightCondition = ringSensors.nne < TARGET_DISTANCE - 5000 &&
                            ringSensors.nnw < TARGET_DISTANCE - 5000;

  switch (wallDir) {
    case WallDirection::BACK:
      sensorDiff = abs((int)ringSensors.ssw - (int)ringSensors.sse);
      break;
    case WallDirection::FRONT:
      sensorDiff = abs((int)ringSensors.nnw - (int)ringSensors.nne);
      freeSightCondition =
        ringSensors.sse < TARGET_DISTANCE && ringSensors.ssw < TARGET_DISTANCE;
      break;
    case WallDirection::LEFT:
      sensorDiff = abs((int)ringSensors.wsw - (int)ringSensors.wnw);
      break;
    case WallDirection::RIGHT:
      sensorDiff = abs((int)ringSensors.ese - (int)ringSensors.ene);
      break;
    case WallDirection::UNKNOWN:
    default:
      ROS_WARN("cannot align with unknown wall direction!");
      return;
  }

  float minPhiDiff = 100.0;
  if (traceState == TraceState::BOUNDARY) {
    for (auto _phi : RIGHT_ANGLE_VALUES) {
      if (abs(_phi - odom.phi) < minPhiDiff) {
        minPhiDiff = abs(_phi - odom.phi);
      }
    }
  } else {
    minPhiDiff = 0.0;
  }

  if (wallDir != WallDirection::UNKNOWN && freeSightCondition) {
    // ROS_INFO("align phiDiff: %d, align sensorDiff: %d",
    //          minPhiDiff < 0.4,
    //          sensorDiff < ALIGNED_THRESHOLD);
    if (minPhiDiff < 0.3 && sensorDiff < ALIGNED_THRESHOLD) {
      if (stateStack.empty()) {
        setState(MappingState::TRACE_BOUNDARY);
      } else {
        setState(stateStack.top());
        stateStack.pop();
      }

      if (_align && traceState == TraceState::BOUNDARY)
        align();

      return;
    }
    twist->angular.z = wallDir == WallDirection::LEFT ||
                           (wallDir == WallDirection::BACK &&
                            previousWallDirection == WallDirection::LEFT)
                         ? -0.1
                         : 0.1;
  } else {
    twist->angular.z = wallDir == WallDirection::LEFT ||
                           (wallDir == WallDirection::BACK &&
                            previousWallDirection == WallDirection::LEFT)
                         ? -0.2
                         : 0.2;
  }
}

void
alignToGoal(geometry_msgs::Twist* twist)
{
  auto alpha = atan2(goal.vec.y - odom.vec.y, goal.vec.x - odom.vec.x);
  ROS_INFO("alpha calculated: %f", alpha);
  goal.phi = fmod(alpha + 2 * M_PI, 2 * M_PI);
  goalETA =
    sqrt(pow(goal.vec.y - odom.vec.y, 2) + pow(goal.vec.x - odom.vec.x, 2)) /
    0.1;
  ROS_INFO("estimated time: %f", goalETA);
  setState(MappingState::ALIGN);
}

void
checkAlignment(geometry_msgs::Twist* twist)
{
  if (abs(goal.phi - odom.phi) < 0.05) {
    ROS_INFO("reached goal orientation");
    if (stateStack.empty()) {
      setState(MappingState::MOVE_GOAL);
      goalStart = ros::Time::now();
    } else {
      setState(stateStack.top());
      stateStack.pop();
    }

    return;
  }

  auto angleDiff = goal.phi - odom.phi;
  if (angleDiff < 0)
    angleDiff += 2 * M_PI;

  if (angleDiff > M_PI)
    twist->angular.z = -0.1;
  else
    twist->angular.z = 0.1;
}

void
moveToGoal(geometry_msgs::Twist* twist)
{
  if (abs(goal.vec.x - odom.vec.x) + abs(goal.vec.y - odom.vec.y) < 0.02) {
    if (stateStack.empty()) {
      setState(MappingState::STOP);
    } else {
      setState(stateStack.top());
      stateStack.pop();
    }

    ROS_INFO("x %f, odom.y %f", odom.vec.x, odom.vec.y);

    writePath(Vector2D());
    ROS_INFO("reached goal odom in %f seconds",
             (ros::Time::now() - goalStart).toSec());
    ROS_INFO("x %f, odom.y %f", odom.vec.x, odom.vec.y);

    goalSet = false;
    return;
  }

  if (ringSensors.nne > TARGET_DISTANCE || ringSensors.nnw > TARGET_DISTANCE) {
    ROS_INFO("realign..");
    // TODO: or avoid/identify obstacle..
    setState(MappingState::ALIGN_BOUNDARY);
    stateStack.push(MappingState::FIND_CORNER);

    if (ringSensors.nnw > ringSensors.nne)
      setWallDir(WallDirection::LEFT);
    else
      setWallDir(WallDirection::RIGHT);

    findCornerStart = Vector2D(odom.vec);
    return;
  }

  // TODO: obstacle avoidance
  // (if below estimated time -> obstacle)
  auto alpha = atan2(goal.vec.y - odom.vec.y, goal.vec.x - odom.vec.x);
  auto targetPhi = fmod(alpha + 2 * M_PI, 2 * M_PI);
  auto angleDiff = targetPhi - odom.phi;

  if (angleDiff < 0)
    angleDiff += 2 * M_PI;
  if (angleDiff > M_PI)
    twist->angular.z = -0.1;
  else
    twist->angular.z = 0.1;

  twist->linear.x = 0.1;
}

void
findCorner(geometry_msgs::Twist* twist)
{
  traceBoundary(twist);

  if (state == MappingState::SET_DISTANCE_CORNER) {
    // TODO: if deviation to goal is too big -> assume obstacle!
    if (goalSet) {
      writePath(Vector2D(goal.vec.x - odom.vec.x, goal.vec.y - odom.vec.y));
      // assume we are at the goal now..
      odom.vec.x = goal.vec.x;
      odom.vec.y = goal.vec.y;

      setWallDir(previousWallDirection);

      if (stateStack.empty()) {
        setState(MappingState::STOP);
      } else {
        setState(stateStack.top());
        stateStack.pop();
      }

      goalSet = false;
    } else {
      for (int i = 0; i <= cornerIndex; i++) {
        auto point = corners[i];
        auto distance = abs(odom.vec.x - point.x) + abs(odom.vec.y - point.y);
        ROS_INFO("findCorner: distance to %dth corner: %f", i, distance);

        if (distance < 0.1) {
          writePath(Vector2D(point.x - odom.vec.x, point.y - odom.vec.y));

          odom.vec.x = point.x;
          odom.vec.y = point.y;
        }
      }
    }
  }
}

void
stripCross(geometry_msgs::Twist* twist)
{
  // TODO: add obstacle avoidance/line correction if sideway sensors detect near
  // object..
  if (ringSensors.nne > TARGET_DISTANCE - 5000 ||
      ringSensors.nnw > TARGET_DISTANCE - 5000) {

    if (isNearBoundary(odom.vec)) {
      stateStack.push(MappingState::STRIP_CORRECT);
      stateStack.push(MappingState::ALIGN_BOUNDARY);
      setState(MappingState::SET_DISTANCE);
      turnAround();
    } else {
      identifyObjectConsecutiveLeftTurns = 0;
      identifyObjectStart = Point{ .vec = odom.vec, .phi = odom.phi };
      traceState = TraceState::IDENTIFY;

      // TODO: ~[1,2] blocks (TODO determine size..) in phi direction..
      auto deltaX = 0.27 * cos(odom.phi);
      auto deltaY = 0.27 * sin(odom.phi);
      // return to strip cross once identified and on other side
      // continue line of travel in forward direction..
      ROS_INFO("identify object start (%f, %f)", odom.vec.x, odom.vec.y);
      ROS_INFO("estimate end position of object to be at: (%f, %f)",
               odom.vec.x + deltaX,
               odom.vec.y + deltaY);
      ROS_WARN("assume obstacle! NOT (completely) IMPLEMENTED!");

      setWallDir(WallDirection::LEFT);
      stateStack.push(MappingState::ALIGN_BOUNDARY);
      setState(MappingState::SET_DISTANCE);
    }

    return;
  }

  twist->linear.x = 0.1;
}

void
stripMap()
{
  // if (workMapArea == nullptr || /* TODO: current area fully mapped? */ false)
  // {
  //   workMapArea = &mapAreas[mapAreaWorkIndex++];
  //   stripLimit = 0.0;
  // }
  // stripLimit = 0.0;

  stripLimit += STRIP_LIMIT_INCREMENT;
  setState(MappingState::STRIP_TRACE);
  stripMapDistance = 0.0;
}

void
mainLoop()
{
  // early return if no sensor simulation yet
  if (ringSensors.wnw < 1)
    return;

  geometry_msgs::Twist twist;
  switch (state) {
    case MappingState::ALIGN:
      checkAlignment(&twist);
      break;
    case MappingState::ALIGN_BOUNDARY:
      alignWithBoundary(&twist);
      break;
    case MappingState::ALIGN_GOAL:
      alignToGoal(&twist);
      break;
    case MappingState::FIND_CORNER:
      findCorner(&twist);
      break;
    case MappingState::FIND_WALL:
      findWall(&twist);
      break;
    case MappingState::INIT:
      setWallDir(WallDirection::FRONT);
      setState(MappingState::ALIGN_BOUNDARY);
      stateStack.push(MappingState::RESET);
      stateStack.push(MappingState::SET_DISTANCE);
      break;
    case MappingState::MOVE_GOAL:
      moveToGoal(&twist);
      break;
    case MappingState::RESET:
      recordedPathLength = 0;
      odom.vec.x = odom.vec.y = odom.phi = 0;
      align();
      setState(MappingState::ALIGN_BOUNDARY);
      stateStack.push(MappingState::TRACE_BOUNDARY);
      break;
    case MappingState::SET_DISTANCE:
      setDistance(&twist);
      break;
    case MappingState::SET_DISTANCE_CORNER:
      setDistanceCorner(&twist);
      break;
    case MappingState::SET_OUTER_CORNER:
      setOuterCorner();
      break;
    case MappingState::STOP:
      break;
    case MappingState::STRIP_CORRECT:
      findCorner(&twist);
      break;
    case MappingState::STRIP_CROSS:
      stripCross(&twist);
      break;
    case MappingState::STRIP_MAP:
      stripMap();
      break;
    case MappingState::STRIP_TRACE:
      traceBoundary(&twist, stripLimit);
      break;
    case MappingState::TRACE_BOUNDARY:
      traceBoundary(&twist);
      break;
    case MappingState::TURN_90:
      turn90Deg();
      break;
  }

  publishTwist(&twist);
}

void
process(const nav_msgs::Odometry::ConstPtr& odom)
{
  // Get the rotations of the robot
  double roll, pitch, yaw;
  tf::Quaternion q(odom->pose.pose.orientation.x,
                   odom->pose.pose.orientation.y,
                   odom->pose.pose.orientation.z,
                   odom->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  if (!(++mapEvaluationIter % mapIterRange)) {
    ROS_INFO("actual orientation: %f", yaw);
  }
}

int
main(int argc, char* argv[])
{
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  // Handle parameters
  node.param<string>(
    "ros_listener_topic_odom", rosListenerTopicOdom, "/amiro1/odom");
  node.param<string>("ros_publisher_topic_map", rosPublisherTopicMap, "/map");
  node.param<string>(
    "ros_publisher_topic_cmd", rosPublisherTopicCmd, "/amiro1/cmd_vel");

  node.param<int>("dimension_x", dimensionX, 5);
  node.param<int>("dimension_y", dimensionY, 5);
  node.param<double>("map_resolution", mapResolution, 0.05);
  node.param<int>("map_evaluation_iter", mapIterRange, 100);

  // Print some information
  ROS_INFO("ros_listener_topic_scan: %s", rosListenerTopicScan.c_str());
  ROS_INFO("ros_listener_topic_odom: %s", rosListenerTopicOdom.c_str());
  ROS_INFO("ros_publisher_topic_map: %s", rosPublisherTopicMap.c_str());
  ROS_INFO("ros_publisher_topic_cmd: %s", rosPublisherTopicCmd.c_str());

  // Robot and sensor setup (center the robot odom in the world)
  robotOffsetX = dimensionX / 2.0f;
  robotOffsetY = dimensionY / 2.0f;

  // Publisher: Send the inter map representation
  pub = node.advertise<nav_msgs::OccupancyGrid>(rosPublisherTopicMap, 1);
  twistPub = node.advertise<geometry_msgs::Twist>(rosPublisherTopicCmd, 1);
  ros::Subscriber floorSub =
    node.subscribe("/amiro1/proximity_ring/values", 1, ringSubCallback);

  // sub to odom topic (only for reference..)
  ros::Subscriber odomSub = node.subscribe(rosListenerTopicOdom, 1, process);

  // Allocate the occupancy grid and center the odom in the world
  mapDimensionX = int(dimensionX / mapResolution) + 1;
  mapDimensionY = int(dimensionY / mapResolution) + 1;
  ogm.header.frame_id = "world";
  ogm.info.resolution = mapResolution;
  ogm.info.width = mapDimensionX;
  ogm.info.height = mapDimensionY;
  ogm.info.origin.position.x = -dimensionX / 2.0f;
  ogm.info.origin.position.y = -dimensionY / 2.0f;
  ogm.data.resize(ogm.info.width * ogm.info.height, -1);
  hit = cv::Mat(mapDimensionY, mapDimensionX, mattypecv, cv::Scalar(1));
  miss = cv::Mat(mapDimensionY, mapDimensionX, mattypecv, cv::Scalar(1));

  ros::Rate r(50);
  auto iter = 0;
  while (ros::ok()) {
    ros::spinOnce();
    mainLoop();
    r.sleep();

    if (!(++iter % (3 * mapIterRange))) {
      // ROS_INFO("x: %f y: %f phi: %f", odom.x, odom.y, odom.phi);
      ogm.header.stamp = ros::Time::now();
      showMap();
    }
  }

  return 0;
}
