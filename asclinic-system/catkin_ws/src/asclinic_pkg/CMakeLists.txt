cmake_minimum_required(VERSION 3.0.2)
project(asclinic_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  genmsg
  roslib
  
  geometry_msgs
)

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
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)s

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_message_files(
  DIRECTORY msg/amr_msgs

  FILES
  AngularVel.msg
  AngularVel1D.msg
  LinearVel.msg
  LinearVel2D.msg
  MotorsAngVelocity.msg
  MotorsRotAngle.msg
  MotorsVoltage.msg 
  Pose2D.msg 
  Pose2DWithCovariance.msg 
  RotationAngle.msg 
  RotationAngle1D.msg 
  Voltage.msg 
)

add_message_files(
  DIRECTORY msg

  FILES
  TransformationMatrix.msg
  AmrPose.msg
  MotorAngularVelocity.msg
  MotorAngularVelocityPWM.msg
  MotorRotationAngle.msg
  ServoPulseWidth.msg
  pathCommand.msg
)

add_message_files(
  DIRECTORY msg/geometry_msgs

  FILES 
  EulerAngle.msg
  Point.msg
  Pose.msg
  Quaternion.msg
  Point32.msg
  Polygon.msg
)

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
#   std_msgs
# )
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asclinic
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
# )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs roslib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )
include_directories(
  include
  src/drivers/src/
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/asclinic.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/asclinic_node.cpp)


# NEW CPP NODES:
# Added by Quang Trung Le
add_executable(timer_monitor                  src/nodes/main/timer_monitor.cpp)
add_executable(motor_controller               src/nodes/main/motor_controller.cpp)
add_executable(motor_dynamics                 src/nodes/main/motor_dynamics.cpp
                                              src/drivers/src/i2c_driver/i2c_driver.cpp
                                              src/drivers/src/pololu_smc_g2/pololu_smc_g2.cpp
                                              src/drivers/src/pca9685/pca9685.cpp)
add_executable(encoder_monitor_r              src/nodes/main/encoder_monitor/encoder_monitor_r.cpp
                                              src/drivers/src/pololu_smc_g2_encoder/pololu_smc_g2_encoder.cpp)
add_executable(encoder_monitor_l              src/nodes/main/encoder_monitor/encoder_monitor_l.cpp
                                              src/drivers/src/pololu_smc_g2_encoder/pololu_smc_g2_encoder.cpp)
add_executable(enc_monitor_4_motor_ctl        src/nodes/main/encoder_monitor/enc_monitor_4_motor_ctl.cpp
                                              src/drivers/src/pololu_smc_g2_encoder/pololu_smc_g2_encoder.cpp)
add_executable(enc_monitor_4_odom_ctl         src/nodes/main/encoder_monitor/enc_monitor_4_odom_ctl.cpp
                                              src/drivers/src/pololu_smc_g2_encoder/pololu_smc_g2_encoder.cpp)
# add_executable(servo_dynamics                 src/nodes/main/servo/servo_dynamics.cpp
#                                               src/drivers/src/i2c_driver/i2c_driver.cpp
#                                               src/drivers/src/pololu_smc_g2/pololu_smc_g2.cpp
#                                               src/drivers/src/pca9685/pca9685.cpp)
# add_executable(servo_controller               src/nodes/main/servo/servo_controller.cpp)
# add_executable(pose_estimator_fusion          src/nodes/main/pose_estimator_fusion.cpp)
# add_executable(encoder_subscriber             src/nodes/miscellaneous/subtasks/encoder_subscriber.cpp
#                                               src/drivers/src/pololu_smc_g2_encoder/pololu_smc_g2_encoder.cpp)


# Added by Hai Dang Nguyen
# add_executable(motion_planning                src/nodes/motion_planning.cpp)
# add_executable(trajectory_track_controller    src/nodes/trajectory_track_controller.cpp)
# add_executable(servo_motor                    src/nodes/servo_motor.cpp
#                                               src/drivers/src/i2c_driver/i2c_driver.cpp
#                                               src/drivers/src/pca9685/pca9685.cpp)
add_executable(ProximitySensor                src/nodes/ProximitySensor.cpp)

# NODE 4 Functional Testing
add_executable(ref_pwm_publish                src/nodes/functional_test/ref_pwm_publish.cpp)
add_executable(ref_ang_vel_publish            src/nodes/functional_test/ref_ang_vel_publish.cpp)
add_executable(ref_pwm_servo_publish          src/nodes/functional_test/ref_pwm_servo_publish.cpp)

# V2
add_executable(motion_planning_v2               src/nodes/motion_planning_v2.cpp)
add_executable(servo_controller_v2              src/nodes/main/servo/servo_controller_v2.cpp)
add_executable(trajectory_track_controller_v2   src/nodes/trajectory_track_controller_v2.cpp)
add_executable(pose_estimator_fusion_v2         src/nodes/main/pose_estimator_fusion_v2.cpp)
add_executable(pose_estimator_fusion_v22         src/nodes/main/pose_estimator_fusion_v22.cpp)

add_executable(motor_controller_v2              src/nodes/main/motor_controller_v2.cpp)
add_executable(motion_planning_v3               src/nodes/motion_planning_v3.cpp)
add_executable(trajectory_track_controller_v3   src/nodes/trajectory_track_controller_v3.cpp)
add_executable(trajectory_track_controller_v22   src/nodes/trajectory_track_controller_v22.cpp)

# copy
add_executable(motion_planning_v2_copy               src/nodes/motion_planning_v2_copy.cpp)
add_executable(trajectory_track_controller_v2_copy  src/nodes/trajectory_track_controller_v2_copy.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# NEW CPP NODES:
# Added by Quang Trung Le
add_dependencies(timer_monitor                  asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(motor_controller               asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(motor_dynamics                 asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(encoder_monitor_r              asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(encoder_monitor_l              asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(enc_monitor_4_motor_ctl        asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(enc_monitor_4_odom_ctl         asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
# add_dependencies(servo_dynamics                 asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
# add_dependencies(servo_controller               asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
# add_dependencies(pose_estimator_fusion          asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
# add_dependencies(encoder_subscriber             asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# Added by Hai Dang Nguyen
# add_dependencies(motion_planning                    asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
# add_dependencies(trajectory_track_controller        asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
# add_dependencies(servo_motor                        asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(ProximitySensor                        asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# NODES 4 Funcitonal Testing
add_dependencies(ref_pwm_publish                asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(ref_ang_vel_publish            asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(ref_pwm_servo_publish          asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# V2
add_dependencies(motion_planning_v2                 asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(servo_controller_v2                asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_track_controller_v2     asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_estimator_fusion_v2           asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_estimator_fusion_v22           asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_dependencies(motor_controller_v2                asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(motion_planning_v3                 asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_track_controller_v3     asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_track_controller_v22     asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# copy
add_dependencies(motion_planning_v2_copy                 asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_track_controller_v2_copy     asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# NEW CPP NODES:
# Added by Quang Trung Le
target_link_libraries(timer_monitor                  ${catkin_LIBRARIES})
target_link_libraries(motor_controller               ${catkin_LIBRARIES})
target_link_libraries(motor_dynamics                 ${catkin_LIBRARIES} -lgpiod)
target_link_libraries(encoder_monitor_r              ${catkin_LIBRARIES} -lgpiod)
target_link_libraries(encoder_monitor_l              ${catkin_LIBRARIES} -lgpiod)
target_link_libraries(enc_monitor_4_motor_ctl        ${catkin_LIBRARIES} -lgpiod)
target_link_libraries(enc_monitor_4_odom_ctl         ${catkin_LIBRARIES} -lgpiod)

# target_link_libraries(servo_dynamics                 ${catkin_LIBRARIES})
# target_link_libraries(servo_controller               ${catkin_LIBRARIES})
# target_link_libraries(pose_estimator_fusion          ${catkin_LIBRARIES})
# target_link_libraries(encoder_subscriber             ${catkin_LIBRARIES} -lgpiod)

# Added by Hai Dang Nguyen
# target_link_libraries(motion_planning                ${catkin_LIBRARIES})
# target_link_libraries(trajectory_track_controller    ${catkin_LIBRARIES})
# target_link_libraries(servo_motor                     ${catkin_LIBRARIES})
target_link_libraries(ProximitySensor         ${catkin_LIBRARIES} -lgpiod)

# V2
target_link_libraries(motion_planning_v2             ${catkin_LIBRARIES})
target_link_libraries(servo_controller_v2            ${catkin_LIBRARIES})
target_link_libraries(trajectory_track_controller_v2 ${catkin_LIBRARIES})
target_link_libraries(pose_estimator_fusion_v2       ${catkin_LIBRARIES})
target_link_libraries(pose_estimator_fusion_v22       ${catkin_LIBRARIES})

target_link_libraries(motor_controller_v2            ${catkin_LIBRARIES})
target_link_libraries(motion_planning_v3             ${catkin_LIBRARIES})
target_link_libraries(trajectory_track_controller_v3 ${catkin_LIBRARIES})
target_link_libraries(trajectory_track_controller_v22 ${catkin_LIBRARIES})

# copy
target_link_libraries(motion_planning_v2_copy             ${catkin_LIBRARIES})
target_link_libraries(trajectory_track_controller_v2_copy ${catkin_LIBRARIES})

# NODES 4 Functional Testing
target_link_libraries(ref_pwm_publish                 ${catkin_LIBRARIES})
target_link_libraries(ref_ang_vel_publish             ${catkin_LIBRARIES})
target_link_libraries(ref_pwm_servo_publish           ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_asclinic.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
