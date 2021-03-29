cmake_minimum_required(VERSION 3.0.2)
project(bobi_vision)

add_compile_options(-std=c++11)

include_directories(include) # TODO: remove for ROS

# file(GLOB TRACKING_SOURCES
#     src/*.cpp
# )
# list(FILTER TRACKING_SOURCES EXCLUDE REGEX "thymio_tracking_node.cpp")
# list(FILTER TRACKING_SOURCES EXCLUDE REGEX "thymio_simu_tracking_node.cpp")

# find_package(
#     catkin REQUIRED COMPONENTS
#     roscpp
#     rospy
#     geometry_msgs
#     ros_basics_msgs
#     sensor_msgs
#     image_transport
#     cv_bridge
# )
find_package(OpenCV REQUIRED)

# catkin_package(
#   CATKIN_DEPENDS ros_basics_msgs
# )

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
# )

if (BUILD_EXAMPLES)
    set(BLOB_DETECTOR_INCLUDES include/bobi_vision/blob_detector.h)
    set(BLOB_DETECTOR_SOURCES src/blob_detector.cpp)
    add_executable(fish_blob_detector_example examples/fish_blob_detector.cpp ${BLOB_DETECTOR_SOURCES})
    target_link_libraries(fish_blob_detector_example ${OpenCV_LIBS})
endif()

# message(STATUS ${TRACKING_SOURCES})

# add_executable(thymio_tracking_node src/thymio_tracking_node.cpp ${TRACKING_SOURCES})
# target_link_libraries(thymio_tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
# add_dependencies(thymio_tracking_node ${catkin_EXPORTED_TARGETS} ros_basics_msgs_generate_messages_cpp)

# add_executable(thymio_simu_tracking_node src/thymio_simu_tracking_node.cpp ${TRACKING_SOURCES})
# target_link_libraries(thymio_simu_tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
# add_dependencies(thymio_simu_tracking_node ${catkin_EXPORTED_TARGETS} ros_basics_msgs_generate_messages_cpp)

