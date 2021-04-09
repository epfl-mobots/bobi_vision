cmake_minimum_required(VERSION 3.0.2)
project(bobi_vision)

add_compile_options(-std=c++11)


file(GLOB TRACKING_SOURCES
    src/*.cpp
)
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "individual_tracking_node.cpp")

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    image_transport
    cv_bridge
    dynamic_reconfigure
)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  config/BlobDetector.cfg
)

catkin_package(
  CATKIN_DEPENDS bobi_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

message(STATUS ${TRACKING_SOURCES})

add_executable(individual_tracking_node src/individual_tracking_node.cpp ${TRACKING_SOURCES})
target_link_libraries(individual_tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(individual_tracking_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)
