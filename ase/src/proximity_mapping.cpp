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

#include <cmath>
#include <stack>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Helper
#include <omp.h>

#include <boost/date_time/posix_time/posix_time.hpp>
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
const unsigned int TARGET_DISTANCE = 47000;
const unsigned int MAX_PATH_LENGTH = 1000;
const float PHI_OFFSET_FACTOR = 0.95; // 0.9 seems to be good
const float RIGHT_ANGLE_VALUES[5] = { 0.0, M_PI_2, M_PI, 3 * M_PI_2, 2 * M_PI };

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
  float x;
  float y;
  float phi;
};

MappingState state = MappingState::INIT;
// MappingState nextState = MappingState::TRACE_BOUNDARY;
std::stack<MappingState> stateStack;

Point position = Point{ .x = 0.0, .y = 0.0, .phi = 0.0 };

WallDirection wallDir = WallDirection::UNKNOWN;
WallDirection previousWallDirection = WallDirection::UNKNOWN;

Point corners[100];
int cornerIndex = 0;

Point goal;
bool goalSet = false;
float goalETA;
ros::Time goalStart;

Point recordedPath[MAX_PATH_LENGTH];
unsigned int recordedPathLength = 0;
unsigned int pathIter = 0;

Point findCornerStart;

geometry_msgs::Twist lastTwist;
const auto minROSTimePoint = ros::Time(0); // TODO: ew, please no eq check
auto lastROSPublish = minROSTimePoint;

float stripLimit = 0.0;
float stripLimitIncrement = 0.1;
float stripMapDistance = 0.0;

std::string
printState(MappingState _state = state)
{
  switch (_state) {
    case MappingState::ALIGN_BOUNDARY:
      return "ALIGN_BOUNDARY";
      break;
    case MappingState::ALIGN_GOAL:
      return "ALIGN_GOAL";
      break;
    case MappingState::ALIGN:
      return "ALIGN";
      break;
    case MappingState::FIND_CORNER:
      return "FIND_CORNER";
      break;
    case MappingState::FIND_WALL:
      return "FIND_WALL";
      break;
    case MappingState::INIT:
      return "INIT";
      break;
    case MappingState::MOVE_GOAL:
      return "MOVE_GOAL";
      break;
    case MappingState::RESET:
      return "RESET";
      break;
    case MappingState::SET_OUTER_CORNER:
      return "SET_OUTER_CORNER";
      break;
    case MappingState::SET_DISTANCE_CORNER:
      return "SET_DISTANCE_CORNER";
      break;
    case MappingState::SET_DISTANCE:
      return "SET_DISTANCE";
      break;
    case MappingState::STOP:
      return "STOP";
      break;
    case MappingState::STRIP_CORRECT:
      return "STRIP_CORRECT";
      break;
    case MappingState::STRIP_CROSS:
      return "STRIP_CROSS";
      break;
    case MappingState::STRIP_MAP:
      return "STRIP_MAP";
      break;
    case MappingState::STRIP_TRACE:
      return "STRIP_TRACE";
      break;
    case MappingState::TRACE_BOUNDARY:
      return "TRACE_BOUNDARY";
      break;
    case MappingState::TURN_90:
      return "TURN_90";
      break;
    default:
      return "";
      break;
  }
}

void
setState(MappingState _state)
{
  state = _state;
  ROS_INFO("state updated %s", printState(state).c_str());
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
      abs(RIGHT_ANGLE_VALUES[i] - fmod(position.phi + 2 * M_PI, 2 * M_PI));
    if (currentDiff < diff) {
      diff = currentDiff;
      angle = RIGHT_ANGLE_VALUES[i];
    }
  }

  position.phi = fmod(angle, 2 * M_PI);
}

void
turnAround()
{
  if (wallDir == WallDirection::LEFT)
    wallDir = WallDirection::RIGHT;
  else if (wallDir == WallDirection::RIGHT)
    wallDir = WallDirection::LEFT;
  else {
    if (previousWallDirection == WallDirection::UNKNOWN ||
        previousWallDirection == WallDirection::BACK)
      ROS_WARN("can not turn around if wallDir unknown or back!");
    else
      wallDir = previousWallDirection;
  }
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
      if (abs(prevRecord.x - position.x) + abs(prevRecord.y - position.y) <
          0.01)
        return;
    }

    recordedPath[recordedPathLength++] =
      Point{ .x = position.x, .y = position.y };
  }
}

// TODO: add hit for walls/obstacles..
void
writePath(Point goalDeviation)
{
  if (recordedPathLength < 1)
    return;

  auto deviationFraction =
    Point{ .x = goalDeviation.x / (float)recordedPathLength,
           .y = goalDeviation.y / (float)recordedPathLength };
  ROS_INFO("deviation and fraction: (%f, %f) -> (%f, %f)",
           goalDeviation.x,
           goalDeviation.y,
           deviationFraction.x,
           deviationFraction.y);
  for (unsigned int i = 0; i < recordedPathLength; i++) {
    auto point = recordedPath[i];
    // ROS_INFO("writing point (%f, %f) -> (%f, %f)", point.x, point.y,
    //          point.x + i * deviationFraction.x,
    //          point.y + i * deviationFraction.y);
    int xi =
      round((point.x + i * deviationFraction.x + robotOffsetX) / mapResolution);
    int yi =
      round((point.y + i * deviationFraction.y + robotOffsetY) / mapResolution);
    // int xi = round((point.x + robotOffsetX) / mapResolution);
    // int yi = round((point.y + robotOffsetY) / mapResolution);

    miss.at<mattype>(yi, xi) = miss.at<mattype>(yi, xi) + 5;
  }

  position.x += goalDeviation.x;
  position.y += goalDeviation.y;

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
    auto deltaX = seconds * lastTwist.linear.x * cos(position.phi);
    auto deltaY = seconds * lastTwist.linear.x * sin(position.phi);

    // error terms are added in writePath()
    position.x += deltaX;
    position.y += deltaY;
    position.phi += deltaPhi;
    position.phi = fmod(position.phi + 2 * M_PI, 2 * M_PI);

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

// void
// turnToBoundary(geometry_msgs::Twist* twist)
// {
//   if (ringSensors.nne < (uint)1 && ringSensors.nnw < (uint)1)
//     return;

//   // TODO: this is bullshit, won't work for large deviation from wall
//   if (ringSensors.nne > 20000 &&
//       abs((int)ringSensors.nne - (int)ringSensors.nnw) < ALIGNED_THRESHOLD) {
//     ROS_INFO("switch to set distance");
//     setState(MappingState::SET_DISTANCE);
//   }

//   // naive approach, twist robot such that front sensor values are equalized
//   if (ringSensors.nne < 20000) {
//     twist->angular.z = -0.2;
//   } else {
//     if (ringSensors.nne > ringSensors.nnw) {
//       twist->angular.z = -0.1;
//     } else {
//       twist->angular.z = 0.1;
//     }
//   }
// }

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
      sensorReadout = ringSensors.wnw;
      break;
    case WallDirection::RIGHT:
      sensorReadout = ringSensors.ene;
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

  twist->linear.x = 0.1;
}

void
turn90Deg()
{
  auto alpha = 0.0;
  switch (wallDir) {
    case WallDirection::LEFT:
      alpha = position.phi + M_PI_2;
      break;
    case WallDirection::RIGHT:
      alpha = position.phi - M_PI_2;
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
      auto distance = abs(position.x - point.x) + abs(position.y - point.y);
      ROS_INFO("distance to %dth corner: %f", i, distance);
      if (distance < 0.2) {
        writePath(
          Point{ .x = point.x - position.x, .y = point.y - position.y });
        // turnAround();

        goal = corners[i + 2];
        goalSet = true;

        stateStack.push(MappingState::STRIP_MAP);
        stateStack.push(MappingState::ALIGN_BOUNDARY);
        stateStack.push(MappingState::MOVE_GOAL);
        setState(MappingState::ALIGN_GOAL); // TODO: this will prob. not work..
        return;
      }
    }
  }
  ROS_INFO(
    "marked %dth corner at (%f, %f)", cornerIndex, position.x, position.y);
  corners[cornerIndex++] = Point{ .x = position.x, .y = position.y };

  auto prevCorner =
    cornerIndex > 1 ? corners[cornerIndex - 2] : Point{ .x = 0, .y = 0 };
  auto currentCorner = corners[cornerIndex - 1];

  if (abs(prevCorner.x - currentCorner.x) < 0.5)
    writePath(Point{ .x = prevCorner.x - currentCorner.x, .y = 0 });
  else
    writePath(Point{ .x = 0, .y = prevCorner.y - currentCorner.y });

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
        auto point = corners[i];
        auto distance = abs(position.x - point.x) + abs(position.y - point.y);
        ROS_INFO("distance to %dth corner: %f", i, distance);
        if (distance < 0.2) {
          writePath(
            Point{ .x = point.x - position.x, .y = point.y - position.y });

          position.x = point.x;
          position.y = point.y;
          // if at first corner again..
          if (i == 0) {
            turnAround();
            stateStack.push(MappingState::STRIP_MAP);
          }
          return;
        }
      }
    }
    ROS_INFO(
      "marked %dth corner at (%f, %f)", cornerIndex, position.x, position.y);
    corners[cornerIndex++] = Point{ .x = position.x, .y = position.y };

    auto prevCorner =
      cornerIndex > 1 ? corners[cornerIndex - 2] : Point{ .x = 0, .y = 0 };
    auto currentCorner = corners[cornerIndex - 1];

    if (abs(prevCorner.x - currentCorner.x) < 0.5)
      writePath(Point{ .x = prevCorner.x - currentCorner.x, .y = 0 });
    else
      writePath(Point{ .x = 0, .y = prevCorner.y - currentCorner.y });

    return;
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
    wallDir = WallDirection::BACK;
    stateStack.push(MappingState::STRIP_CROSS);
    // nextState = MappingState::STRIP_CROSS;
    return;
  }

  // TODO: add angular error for deviation from target distance.. PID?
  switch (wallDir) {
    case WallDirection::LEFT:
      if (ringSensors.wnw < TARGET_DISTANCE - 15000 &&
          ringSensors.wsw < TARGET_DISTANCE - 15000) {
        setState(MappingState::SET_OUTER_CORNER);
        return;
      }

      twist->angular.z = ringSensors.wnw > ringSensors.wsw ? -0.03 : 0.03;
      break;
    case WallDirection::RIGHT:
      if (ringSensors.ene < TARGET_DISTANCE - 15000 &&
          ringSensors.ese < TARGET_DISTANCE - 15000) {
        setState(MappingState::SET_OUTER_CORNER);
        return;
      }

      twist->angular.z = ringSensors.ene > ringSensors.ese ? 0.03 : -0.03;
      break;
    default:
      break;
  }

  twist->linear.x = 0.1;
}

void
alignWithBoundary(geometry_msgs::Twist* twist, bool _align = true)
{
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

  for (auto _phi : RIGHT_ANGLE_VALUES) {
    if (abs(_phi - position.phi) < minPhiDiff) {
      minPhiDiff = abs(_phi - position.phi);
    }
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

      if (_align)
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
  auto alpha = atan2(goal.y - position.y, goal.x - position.x);
  ROS_INFO("alpha calculated: %f", alpha);
  goal.phi = fmod(alpha + 2 * M_PI, 2 * M_PI);
  goalETA =
    sqrt(pow(goal.y - position.y, 2) + pow(goal.x - position.x, 2)) / 0.1;
  ROS_INFO("estimated time: %f", goalETA);
  setState(MappingState::ALIGN);
}

void
checkAlignment(geometry_msgs::Twist* twist)
{
  if (abs(goal.phi - position.phi) < 0.05) {
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

  auto angleDiff = goal.phi - position.phi;
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
  if (abs(goal.x - position.x) + abs(goal.y - position.y) < 0.05) {
    if (stateStack.empty()) {
      setState(MappingState::STOP);
    } else {
      setState(stateStack.top());
      stateStack.pop();
    }

    ROS_INFO("x %f, position.y %f", position.x, position.y);

    writePath(Point{ .x = 0, .y = 0 });
    ROS_INFO("reached goal position in %f seconds",
             (ros::Time::now() - goalStart).toSec());
    ROS_INFO("x %f, position.y %f", position.x, position.y);

    goalSet = false;
    return;
  }

  if (ringSensors.nne > TARGET_DISTANCE || ringSensors.nnw > TARGET_DISTANCE) {
    ROS_INFO("realign..");
    // TODO: or avoid/identify obstacle..
    setState(MappingState::ALIGN_BOUNDARY);
    stateStack.push(MappingState::FIND_CORNER);

    previousWallDirection = wallDir;
    if (ringSensors.nnw > ringSensors.nne)
      wallDir = WallDirection::LEFT;
    else
      wallDir = WallDirection::RIGHT;

    findCornerStart = Point{ .x = position.x, .y = position.y };
    return;
  }

  // TODO: obstacle avoidance
  // (if below estimated time -> obstacle)
  auto alpha = atan2(goal.y - position.y, goal.x - position.x);
  auto targetPhi = fmod(alpha + 2 * M_PI, 2 * M_PI);
  auto angleDiff = targetPhi - position.phi;

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
      writePath(Point{ .x = goal.x - position.x, .y = goal.y - position.y });
      // assume we are at the goal now..
      position.x = goal.x;
      position.y = goal.y;

      wallDir = previousWallDirection;

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
        auto distance = abs(position.x - point.x) + abs(position.y - point.y);
        ROS_INFO("findCorner: distance to %dth corner: %f", i, distance);

        if (distance < 0.2) {
          writePath(
            Point{ .x = point.x - position.x, .y = point.y - position.y });

          position.x = point.x;
          position.y = point.y;

          // turnAround();

          // TODO: ough, that's a mess
          previousWallDirection = wallDir;
          setState(MappingState::SET_DISTANCE);
          stateStack.push(MappingState::STRIP_MAP);
          stateStack.push(MappingState::ALIGN_BOUNDARY);
          stateStack.push(MappingState::TRACE_BOUNDARY);
          stateStack.push(MappingState::ALIGN_BOUNDARY);
        }
      }
    }
  }
}

void
stripCross(geometry_msgs::Twist* twist)
{
  if (ringSensors.nne > TARGET_DISTANCE - 5000 ||
      ringSensors.nnw > TARGET_DISTANCE - 5000) {
    // TODO: or detect obstacle..

    stateStack.push(MappingState::STRIP_CORRECT);
    stateStack.push(MappingState::ALIGN_BOUNDARY);
    setState(MappingState::SET_DISTANCE);
    turnAround();
    return;
  }

  twist->linear.x = 0.1;
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
      wallDir = WallDirection::FRONT;
      setState(MappingState::ALIGN_BOUNDARY);
      stateStack.push(MappingState::RESET);
      stateStack.push(MappingState::SET_DISTANCE);
      break;
    case MappingState::MOVE_GOAL:
      moveToGoal(&twist);
      break;
    case MappingState::RESET:
      recordedPathLength = 0;
      position.x = position.y = position.phi = 0;
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
    case MappingState::STRIP_MAP: // TODO: function..
      stripLimit += stripLimitIncrement;
      setState(MappingState::STRIP_TRACE);
      previousWallDirection = wallDir;
      stripMapDistance = 0.0;
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

  // Robot and sensor setup (center the robot position in the world)
  robotOffsetX = dimensionX / 2.0f;
  robotOffsetY = dimensionY / 2.0f;

  // Publisher: Send the inter map representation
  pub = node.advertise<nav_msgs::OccupancyGrid>(rosPublisherTopicMap, 1);
  twistPub = node.advertise<geometry_msgs::Twist>(rosPublisherTopicCmd, 1);
  ros::Subscriber floorSub =
    node.subscribe("/amiro1/proximity_ring/values", 1, ringSubCallback);

  // sub to odom topic (only for reference..)
  ros::Subscriber odomSub = node.subscribe(rosListenerTopicOdom, 1, process);

  // Allocate the occupancy grid and center the position in the world
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
      // ROS_INFO("x: %f y: %f phi: %f", position.x, position.y, position.phi);
      ogm.header.stamp = ros::Time::now();
      showMap();
    }
  }

  return 0;
}
