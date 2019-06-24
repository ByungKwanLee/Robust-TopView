cmake_minimum_required(VERSION 2.8.3)
project(robust_topview)


add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  image_transport
  std_msgs
)

catkin_package(
)


find_package( OpenCV 3.2 REQUIRED )

include_directories(
 #include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(robust_topview src/robust_topview.cpp)
target_link_libraries(robust_topview ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

