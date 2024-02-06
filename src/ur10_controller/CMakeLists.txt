cmake_minimum_required(VERSION 3.0.2)
project(ur10_controller)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tum_ics_ur_robot_msgs
  tum_ics_ur_robot_lli
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)

##############################################################################
# Eigen
##############################################################################
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(TMath REQUIRED)
add_definitions(${TMATH_DEFINITIONS})
include_directories(${TMATH_INCLUDE_DIRS})

find_package(RtThreads REQUIRED)
add_definitions(${RTTHREADS_DEFINITIONS})
include_directories(${RTTHREADS_INCLUDE_DIR})

find_package(Qt5Network)
add_definitions(${Qt5Network_DEFINITIONS})
include_directories(${Qt5Network_INCLUDE_DIR})
set(CMAKE_CXX_FLAGS "${Qt5Network_EXECUTABLE_COMPILE_FLAGS}")

find_package(Qt5Widgets)
add_definitions(${Qt5Widgets_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDE_DIR})
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES my_ur10_controller
 CATKIN_DEPENDS tum_ics_ur_robot_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions visualization_msgs tum_ics_ur_robot_lli
 DEPENDS ${DEPENDS_VAR}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp -O3" )
set(CMAKE_BUILD_TYPE Release)

file(GLOB_RECURSE MY_CTRL_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
	include/*.h)


## Declare a C++ library
add_library(my_ur10_controller
   src/ur10_effort_controller.cpp
   src/UR10Model.cpp
   src/UR10Regressor.cpp
   ${MY_CTRL_INC}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(my_ur10_controller
    ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS}
    ${sensor_msgs_EXPORTED_TARGETS}
    ${std_msgs_EXPORTED_TARGETS}
    ${visualization_msgs_EXPORTED_TARGETS}
)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ur10_controller_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set(APP_TARGETS
  ur10_effort_controller_node
)

foreach(target ${APP_TARGETS})
    add_executable(${target} src/applications/${target}.cpp)
    target_link_libraries(${target}
        my_ur10_controller
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${tum_ics_ur_robot_lli_LIBRARIES}
        ${Qt5Network_LIBRARIES}
        ${Qt5Widgets_LIBRARIES}
        ${RTTHREADS_LIBRARIES}
        ${TMATH_LIBRARIES})
    add_dependencies(${target}
        ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS}
        ${sensor_msgs_EXPORTED_TARGETS}
        ${std_msgs_EXPORTED_TARGETS}
        ${visualization_msgs_EXPORTED_TARGETS}
    )
endforeach()
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(my_ur10_controller
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tum_ics_ur_robot_lli_LIBRARIES}
    ${Qt5Network_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${RTTHREADS_LIBRARIES}
    ${TMATH_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set(INS_TARGETS
my_ur10_controller
    ${APP_TARGETS}
)

install(TARGETS ${INS_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur10_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
