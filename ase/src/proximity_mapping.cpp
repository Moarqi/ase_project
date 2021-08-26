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
static int dimensionX = 9;         // (m)
static int dimensionY = 9;         // (m)
static double mapResolution = 0.1; // (m)
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

const unsigned int ALIGNED_THRESHOLD = 200;
const unsigned int TARGET_DISTANCE = 40000;

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
  INIT,
  TURNTOBOUNDARY,
  SETDISTANCE,
  PREPARETRACE,
  TRACEBOUNDARY,
  TURNRIGHT,
};

MappingState state = MappingState::INIT;
float x = 0.0;
float y = 0.0;
float phi = 0.0;

geometry_msgs::Twist lastTwist;
const auto minTimePoint = chrono::high_resolution_clock::time_point::min();
auto lastPublish = minTimePoint;

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
  auto now = chrono::high_resolution_clock::now();
  if (lastPublish == minTimePoint) {
    lastPublish = now;
  }

  auto uSecondsSinceLastPublish =
      chrono::duration_cast<chrono::microseconds>(now - lastPublish);
  if (uSecondsSinceLastPublish.count() > 0) {
    double seconds = uSecondsSinceLastPublish.count() / 1e6;

    auto deltaPhi = seconds * lastTwist.angular.z;
    auto deltaX = seconds * lastTwist.linear.x * cos(phi + deltaPhi);
    auto deltaY = seconds * lastTwist.linear.x * sin(phi + deltaPhi);

    // TODO: add error terms
    x += deltaX;
    y += deltaY;
    phi += deltaPhi;
  }

  twistPub.publish(*twist);
  lastPublish = chrono::high_resolution_clock::now();
  lastTwist = *twist;

  // it might be better to only save the path of the robot (sample the coordinates) and later creating a map of it, 
  // while scaling/correcting errors

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
  ROS_INFO("nne val: %d", ringSensors.nne);  
  ROS_INFO("nnw val: %d", ringSensors.nnw);  

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
    ROS_INFO("switch to prepare");
    state = MappingState::PREPARETRACE;
    return;
  }

  if (distance < TARGET_DISTANCE)
    twist->linear.x = 0.01;
  else
    twist->linear.x = -0.01;
}

void alignWithBoundary(geometry_msgs::Twist *twist) {
  ROS_INFO("wnw, wsw: %d, %d", ringSensors.wnw, ringSensors.wsw);
  if (ringSensors.wsw < (uint)1 && ringSensors.wnw < (uint)1)
    return;

  // TODO: refine implementation here
  if (ringSensors.wsw > 25000 && abs((int)ringSensors.wsw - (int)ringSensors.wnw) < ALIGNED_THRESHOLD) {
    ROS_INFO("switch to trace");
    state = MappingState::TRACEBOUNDARY;
  }

  // naive approach, twist robot such that leftmost sensor values are equalized
  if (ringSensors.wnw > ringSensors.wsw) {
    twist->angular.z = -0.1;
  } else {
    twist->angular.z = 0.1;
  }
}

void traceBoundary(geometry_msgs::Twist *twist) {
  if (ringSensors.nne > TARGET_DISTANCE) {
    state = MappingState::TURNRIGHT;
    return;
  }

  if (ringSensors.wnw > ringSensors.wsw) {
    twist->angular.z = -0.01;
  } else {
    twist->angular.z = 0.01;
  }

  twist->linear.x = 0.1;

  publishTwist(twist);
}

void turnRight(geometry_msgs::Twist *twist) {
  if (ringSensors.nne < 20000 && abs((int)ringSensors.wsw - (int)ringSensors.wnw) < ALIGNED_THRESHOLD) {
    state = MappingState::TRACEBOUNDARY;
    return;
  }

  twist->angular.z = -0.2;
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
  case MappingState::PREPARETRACE:
    alignWithBoundary(&twist);
    break;
  case MappingState::TRACEBOUNDARY:
    traceBoundary(&twist);
    break;
  case MappingState::TURNRIGHT:
    turnRight(&twist);
    break;
  }

  if (state == MappingState::TRACEBOUNDARY || state == MappingState::TURNRIGHT)
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

  node.param<int>("dimension_x", dimensionX, 9);
  node.param<int>("dimension_y", dimensionY, 9);
  node.param<double>("map_resolution", mapResolution, 0.1);
  node.param<int>("map_evaluation_iter", mapEvaluationIter, 20);

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

  ros::Rate r(10);
  auto iter = 0;
  while (ros::ok()) {
    ros::spinOnce();
    mainLoop();
    r.sleep();

    if (++iter % mapEvaluationIter == 0) {
      ROS_INFO("-- Publish map --");
      ROS_INFO("x: %f y: %f phi: %f", x, y, phi);
      ogm.header.stamp = ros::Time::now();
      showMap();
    }
  }

  return 0;
}
