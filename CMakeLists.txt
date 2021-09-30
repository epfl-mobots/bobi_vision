cmake_minimum_required(VERSION 3.0.2)
project(bobi_vision)

add_compile_options(-std=c++11)


file(GLOB TRACKING_SOURCES
    src/*.cpp
)
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "individual_tracking_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "visualization_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "trajectory_identification_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "robot_tracking_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "aruco_tracking_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "coordinate_converter_node.cpp")

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
  config/ColourDetector.cfg
  config/TrajectoryIdentificationConfig.cfg
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

add_executable(visualization_node src/visualization_node.cpp ${TRACKING_SOURCES})
target_link_libraries(visualization_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(visualization_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(trajectory_identification_node src/trajectory_identification_node.cpp ${TRACKING_SOURCES})
target_link_libraries(trajectory_identification_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(trajectory_identification_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(robot_tracking_node src/robot_tracking_node.cpp ${TRACKING_SOURCES})
target_link_libraries(robot_tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(robot_tracking_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(coordinate_converter_node src/coordinate_converter_node.cpp ${TRACKING_SOURCES})
target_link_libraries(coordinate_converter_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(coordinate_converter_node ${catkin_EXPORTED_TARGETS} bobi_msgs_generate_messages_cpp)

add_executable(aruco_tracking_node src/aruco_tracking_node.cpp ${TRACKING_SOURCES})
target_link_libraries(aruco_tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(aruco_tracking_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)