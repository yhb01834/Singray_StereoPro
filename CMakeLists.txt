cmake_minimum_required(VERSION 3.10)
project(xv_sdk)

## Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-unused-function)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    roslint
    camera_info_manager
    image_transport
    ddynamic_reconfigure
    tf2_ros
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

# Declare the message files to be built
add_message_files(FILES
  FisheyeImages.msg
  OrientationStamped.msg
  Plane.msg
  Planes.msg
  Lost.msg
  PoseStampedConfidence.msg
)

# Declare the service files to be built
add_service_files(FILES
  GetPose.srv
  GetPoseAt.srv
  GetOrientation.srv
  GetOrientationAt.srv
  GetDevices.srv
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    camera_info_manager
    image_transport
    ddynamic_reconfigure
    tf2_ros
  DEPENDS
  ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
  ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
  # Eigen3

  ## Boost is not part of the DEPENDS since it is only used in source files,
  ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

#find_package(xv-sdk REQUIRED)
if (XVSDK_INCLUDE_DIRS AND XVSDK_LIBRARIES)
  set(XVSDK_FOUND TRUE)
else()
  set(XVSDK_FOUND FALSE)
endif()
if (XVSDK_FOUND)
  message(STATUS " - xvsdk includes: ${XVSDK_INCLUDE_DIRS}")
  message(STATUS " - xvsdk libraries: ${XVSDK_LIBRARIES}")
else()
  message(FATAL_ERROR "xvsdk not provided")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
  ${EIGEN3_INCLUDE_DIR}
  # Set because Boost is an internal dependency, not transitive.
  ${Boost_INCLUDE_DIRS}
  
  ${XVSDK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare cpp executables
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_wrapper.cpp src/${PROJECT_NAME}_node.cpp)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME} ${XVSDK_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(${CATKIN_ENABLE_TESTING})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_xv_sdk.cpp)
target_link_libraries(${PROJECT_NAME}-test ${XVSDK_LIBRARIES})
endif()

##########################
## Static code analysis ##
##########################

roslint_cpp()


