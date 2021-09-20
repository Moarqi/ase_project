cmake_minimum_required(VERSION 2.8.3)
project(ase)

find_package(OpenCV 4.2 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost COMPONENTS graph system REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
  sensor_mockup
  amiro_msgs
  )

if (OPENMP_FOUND)
  message("OpenMP FOUND!")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else (OPENMP_FOUND)
  message("NO OpenMP FOUND!")
endif (OPENMP_FOUND)

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated-declarations -Ofast -Wno-write-strings -std=c++11")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  amiro_msgs
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  includes
  src/lib
)

add_executable(mapping
  src/proximity_mapping.cpp
  )
target_link_libraries(mapping
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )
