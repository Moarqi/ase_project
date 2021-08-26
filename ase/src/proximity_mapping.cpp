// ============================================================================
// Name        : mapping_with_known_poses.cpp
// Author      : Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
//               Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : The mapping exercise
// ============================================================================

// amiro
#include <amiro_msgs/UInt16MultiArrayStamped.h>
// ROS
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Helper
#include <boost/date_time/posix_time/posix_time.hpp>
#include <omp.h>
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
static int dimensionX = 5;         // (m)
static int dimensionY = 5;         // (m)
static double mapResolution = 0.05; // (m)
static int mapDimensionX;          // (cells)
static int mapDimensionY;          // (cells)
static int mapEvaluationIter;      // (1)
#define mattype uint16_t // CV_8U == uchar, CV_16U == short, CV_32FC1 == float
#define mattypecv CV_16U
#define maxValue 65535
static cv::Mat hit, miss;           // (1)
static nav_msgs::OccupancyGrid ogm; // (P)

// Robot and sensor setup
double robotOffsetX = dimensionX / 2.0f; // (m)
double robotOffsetY = dimensionY / 2.0f; // (m)

const unsigned int ALIGNED_THRESHOLD = 300;
const unsigned int TARGET_DISTANCE = 47000;

static struct RingSensors {
  uint sse;
  uint ese;
  uint ene;
  uint nne;
  uint nnw;
  uint wnw;
  uint wsw;
  uint ssw;
} ringSensors;

enum MappingState {
  ALIGN,
  ALIGNTOGOAL,
  INIT,
  SETDISTANCE,
  SETDISTANCECORNER,
  STOP,
  TRACEBOUNDARY,
  TURNLEFT,
  TURNRIGHT,
  TURNTOBOUNDARY,
};

struct Point {
  float x;
  float y;
  float phi;
};

MappingState state = MappingState::INIT;
float x = 0.0;
float y = 0.0;
float phi = 0.0;

Point corners[100];
int cornerIndex = 0;

Point goal;

float rightAngleValues[5] = {0.0, M_PI_2, M_PI, 3 * M_PI_2, 2 * M_PI};

void align() {  
  auto diff = 100.0;
  auto angle = 0.0;

  for (int i = 0; i < 5; i++) {
    auto currentDiff = abs(rightAngleValues[i] - fmod(phi + 2 * M_PI, 2*M_PI));
    if (currentDiff < diff) {
      diff = currentDiff;
      angle = rightAngleValues[i];
    }
  }
  
  phi = fmod(angle, 2 * M_PI);
}

geometry_msgs::Twist lastTwist;
const auto minROSTimePoint = ros::Time(0); // TODO: ew, please no eq check
auto lastROSPublish = minROSTimePoint;

void ringSubCallback(const amiro_msgs::UInt16MultiArrayStamped::ConstPtr ring) {
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
inline void showMap() {
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

void publishTwist(geometry_msgs::Twist *twist) {
  auto ROSNow = ros::Time::now();
  if (lastROSPublish == minROSTimePoint)
    lastROSPublish = ROSNow;

  auto nSeconds = (ROSNow - lastROSPublish).toNSec();  
  if (nSeconds > 0) {
    double seconds = nSeconds / 1e9;

    auto deltaPhi = seconds * lastTwist.angular.z;
    auto deltaX = seconds * lastTwist.linear.x * cos(phi);
    auto deltaY = seconds * lastTwist.linear.x * sin(phi);

    // TODO: add error terms
    x += deltaX;
    y += deltaY;
    phi += deltaPhi;
    phi = fmod(phi + 2 * M_PI, 2 * M_PI);
  }

  twistPub.publish(*twist);
  lastROSPublish = ros::Time::now();
  lastTwist = *twist;

  // it might be better to only save the path of the robot (sample the
  // coordinates) and later creating a map of it, while scaling/correcting
  // errors

  // cross area when finished tracing boundary (diagonal?)
  // simply consider each point not taken to be occupied?!

  // hit.at<mattype>(yi, xi) = hit.at<mattype>(yi, xi) + 1;
  int xi = round((x + robotOffsetX) / mapResolution);
  int yi = round((y + robotOffsetY) / mapResolution);

  miss.at<mattype>(yi, xi) = miss.at<mattype>(yi, xi) + 1;
}

void turnToBoundary(geometry_msgs::Twist *twist) {
  if (ringSensors.nne < (uint)1 && ringSensors.nnw < (uint)1)
    return;

  // TODO: this is bullshit, won't work for large deviation from wall
  if (ringSensors.nne > 20000 &&
      abs((int)ringSensors.nne - (int)ringSensors.nnw) < ALIGNED_THRESHOLD) {
    ROS_INFO("switch to set distance");
    state = MappingState::SETDISTANCE;
  }

  // naive approach, twist robot such that front sensor values are equalized
  if (ringSensors.nne < 20000) {
    twist->angular.z = -0.2;
  } else {
    if (ringSensors.nne > ringSensors.nnw) {
      twist->angular.z = -0.1;
    } else {
      twist->angular.z = 0.1;
    }
  }
}

void setDistance(geometry_msgs::Twist *twist) {
  auto distance = (ringSensors.nne + ringSensors.nnw) / 2;

  if (abs((int)distance - (int)TARGET_DISTANCE) < ALIGNED_THRESHOLD) {
    ROS_INFO("turn right");
    state = MappingState::TURNRIGHT;
    return;
  }

  if (distance < TARGET_DISTANCE)
    twist->linear.x = 0.01;
  else
    twist->linear.x = -0.01;
}

void setDistanceCorner(geometry_msgs::Twist *twist) {
  setDistance(twist);

  if (state == MappingState::TURNRIGHT) {
    if (cornerIndex % 2 == 0) {
      for (int i = 0; i <= cornerIndex; i++) {
        auto point = corners[i]; 
        auto distance = abs(x - point.x) + abs(y - point.y);
        ROS_INFO("distance to %dth corner: %f", i, distance);
        if (distance < 0.1) {
          state = MappingState::ALIGNTOGOAL;
          goal = corners[2];
          return;
        }
      }
    }
    ROS_INFO("marked %dth corner at (%f, %f)", cornerIndex, x, y);
    corners[cornerIndex++] = Point { x = x, y = y };    
  }
}

// void alignWithBoundary(geometry_msgs::Twist *twist) {
//   if (ringSensors.wsw < (uint)1 && ringSensors.wnw < (uint)1)
//     return;

//   // TODO: refine implementation here
//   if (ringSensors.wsw > 25000 &&
//       abs((int)ringSensors.wsw - (int)ringSensors.wnw) < ALIGNED_THRESHOLD) {
//     ROS_INFO("switch to trace");
//     state = MappingState::TRACEBOUNDARY;
//     align();
//     return;
//   }

//   // naive approach, twist robot such that leftmost sensor values are equalized
//   if (ringSensors.wnw > ringSensors.wsw) {
//     twist->angular.z = -0.1;
//   } else {
//     twist->angular.z = 0.1;
//   }
// }

void traceBoundary(geometry_msgs::Twist *twist) {
  if (ringSensors.nne > TARGET_DISTANCE) {    
    state = MappingState::SETDISTANCECORNER;
    return;
  }

  if (ringSensors.wsw < 25000 && ringSensors.wnw < 25000) {
    state = MappingState::TURNLEFT;
    return;
  }

  if (ringSensors.wnw > ringSensors.wsw) {
    twist->angular.z = -0.02;
  } else {
    twist->angular.z = 0.02;
  }

  twist->linear.x = 0.08;  
}

void turnRight(geometry_msgs::Twist *twist) {
  if (ringSensors.nne < 30000 && ringSensors.nnw < 30000 &&
      abs((int)ringSensors.wsw - (int)ringSensors.wnw) <
          ALIGNED_THRESHOLD) {
    state = MappingState::TRACEBOUNDARY;
    align();
    return;
  }

  twist->angular.z = -0.1;
}

void turnLeft(geometry_msgs::Twist *twist) {
  // TODO: this will not work..
  if (ringSensors.nne < 30000 && ringSensors.nnw < 30000 &&
      abs((int)ringSensors.wsw - (int)ringSensors.wnw) <
          2 * ALIGNED_THRESHOLD) {
    state = MappingState::TRACEBOUNDARY;
    align();
    return;
  }

  twist->angular.z = 0.1;
}

void alignToGoal(geometry_msgs::Twist *twist) {
  auto alpha = atan2(goal.y - y, goal.x - x);
  ROS_INFO("alpha calculated: %f", alpha);  
  goal.phi = fmod(alpha + 2*M_PI, 2*M_PI);
  state = MappingState::ALIGN;
}

void checkAlignment(geometry_msgs::Twist *twist) {
  if (abs(goal.phi - phi) < 0.05) {
    ROS_INFO("reached goal orientation");
    state = MappingState::STOP;
    return;
  }

  auto angleDiff = goal.phi - phi;
  ROS_INFO("angleDiff pre %f", angleDiff);
  if (angleDiff < 0)
    angleDiff += 2*M_PI;
  ROS_INFO("angleDiff post %f", angleDiff);

  if (angleDiff > M_PI)
    twist->angular.z = -0.1;
  else
    twist->angular.z = 0.1;
}

void moveToGoal(geometry_msgs::Twist *twist) {

}

void mainLoop() {
  geometry_msgs::Twist twist;
  switch (state) {
  case MappingState::INIT:
    state = MappingState::TURNTOBOUNDARY;
    break;
  case MappingState::TURNTOBOUNDARY:
    turnToBoundary(&twist);
    break;
  case MappingState::SETDISTANCE:
    setDistance(&twist);
    break;
  case MappingState::SETDISTANCECORNER:
    setDistanceCorner(&twist);
    break;
  case MappingState::TURNLEFT:
    turnLeft(&twist);
    break;
  case MappingState::TRACEBOUNDARY:
    traceBoundary(&twist);
    break;
  case MappingState::TURNRIGHT:
    turnRight(&twist);
    break;
  case MappingState::ALIGNTOGOAL:
    alignToGoal(&twist);
    break;
  case MappingState::ALIGN:
    checkAlignment(&twist);
    break;
  case MappingState::STOP:
    break;
  }

  if (state != MappingState::TURNTOBOUNDARY && state != MappingState::SETDISTANCE)
    publishTwist(&twist);
  else
    twistPub.publish(twist);
}

void process(const sensor_msgs::LaserScan::ConstPtr &scan,
             const nav_msgs::Odometry::ConstPtr &odom) {
  // Get the rotations of the robot
  // double roll, pitch, yaw;
  // tf::Quaternion q(odom->pose.pose.orientation.x,
  // odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
  // odom->pose.pose.orientation.w); tf::Matrix3x3 m(q); m.getRPY(roll, pitch,
  // yaw);

  // // Store the position (xx, xy) and orientation (xt) of the robot in the map
  // const double xx = (robotOffsetX + odom->pose.pose.position.x) /
  // mapResolution; const double xy = (robotOffsetY +
  // odom->pose.pose.position.y) / mapResolution;

  // const double xt = yaw;

  // Update every cell regarding the scan
#pragma omp parallel for
  for (int yi = 0; yi < mapDimensionY; yi++) {
    for (int xi = 0; xi < mapDimensionX; xi++) {
      // if () {
      //   hit.at<mattype>(yi, xi) = hit.at<mattype>(yi, xi) + 1;
      // } else if () {
      //   miss.at<mattype>(yi, xi) = miss.at<mattype>(yi, xi) + 1;
      // }
    }
  }

  // if (!(++iterator % mapEvaluationIter)) {
  //   ROS_INFO("-- Publish map --");
  //   ogm.header.stamp = scan->header.stamp;
  //   showMap();
  // }
}

int main(int argc, char *argv[]) {

  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  // Handle parameters
  node.param<string>("ros_listener_topic_odom", rosListenerTopicOdom,
                     "/amiro1/odom");
  node.param<string>("ros_publisher_topic_map", rosPublisherTopicMap, "/map");
  node.param<string>("ros_publisher_topic_cmd", rosPublisherTopicCmd,
                     "/amiro1/cmd_vel");

  node.param<int>("dimension_x", dimensionX, 5);
  node.param<int>("dimension_y", dimensionY, 5);
  node.param<double>("map_resolution", mapResolution, 0.05);
  node.param<int>("map_evaluation_iter", mapEvaluationIter, 100);

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
  // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(node,
  // rosListenerTopicOdom, 1); odom_sub.registerCallback(boost::bind(&process,
  // _1, _2));

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

  ros::Rate r(100);
  auto iter = 0;
  while (ros::ok()) {
    ros::spinOnce();
    mainLoop();
    r.sleep();

    if (++iter % mapEvaluationIter == 0) {      
      ROS_INFO("x: %f y: %f phi: %f", x, y, phi);
      ogm.header.stamp = ros::Time::now();
      showMap();
    }
  }

  return 0;
}
