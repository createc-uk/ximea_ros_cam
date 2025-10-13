cmake_minimum_required(VERSION 3.5.1)
project(ximea_camera)

set(CMAKE_CXX_STANDARD 17)

## Set ximea environment variables
set(XIMEA_INCLUDE_PATH /usr/include)
set(XIMEA_LIBRARY m3api)

## Find ament and ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
find_package(yaml-cpp REQUIRED)
find_package(ximea_camera_interfaces REQUIRED)

# # Generate ROS2 messages
# rosidl_generate_interfaces(${PROJECT_NAME}_msgs
#   "msg/XiImageInfo.msg"
#   DEPENDENCIES std_msgs
# )

# Set dependencies for ament
set(dependencies
  rclcpp
  rclcpp_components
  image_transport
  camera_info_manager
  diagnostic_updater
  std_msgs
  sensor_msgs
  ximea_camera_interfaces
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
    ${XIMEA_INCLUDE_PATH}
    ${Boost_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC
    src/ximea_camera.cpp
)

# Link the ament libraries
target_link_libraries(${PROJECT_NAME}
    ${XIMEA_LIBRARY}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

## Package depends on its generated messages
#rosidl_target_interfaces(${PROJECT_NAME} ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/ximea_camera_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node PUBLIC
    # ${XIMEA_LIBRARY}
    # ${OpenCV_LIBRARIES}
    # yaml-cpp
    ${PROJECT_NAME}
)
#ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}/
)

# Add ament package macro
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
