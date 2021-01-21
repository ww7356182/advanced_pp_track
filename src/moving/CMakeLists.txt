cmake_minimum_required(VERSION 2.8.3)
project(moving)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  learning_tf
  turtlesim
  message_generation
  visualization_msgs
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
#   std_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES moving
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/moving.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/moving_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_moving.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(spline src/spline.cpp)
target_link_libraries(spline ${catkin_LIBRARIES})

add_executable(path_demo src/path_demo.cpp)
target_link_libraries(path_demo ${catkin_LIBRARIES})

include_directories(/usr/include)
add_executable(path_track src/path_track.cpp)
target_link_libraries(path_track ${catkin_LIBRARIES})
target_link_libraries(path_track -lusbcan -lcanlib)

add_executable(tra_receive src/tra_receive.cpp)
target_link_libraries(tra_receive ${catkin_LIBRARIES})

include_directories(/usr/include)
add_executable(track_con src/track_con.cpp)
target_link_libraries(track_con ${catkin_LIBRARIES})
target_link_libraries(track_con -lusbcan -lcanlib)

include_directories(/usr/include)
add_executable(track_agv src/track_agv.cpp)
target_link_libraries(track_agv ${catkin_LIBRARIES})
target_link_libraries(track_agv -lusbcan -lcanlib)

include_directories(/usr/include)
add_executable(track_carto src/track_carto.cpp)
target_link_libraries(track_carto ${catkin_LIBRARIES})
target_link_libraries(track_carto -lusbcan -lcanlib)


include_directories(/usr/include)
add_executable(track_10 src/track_10.cpp)
target_link_libraries(track_10 ${catkin_LIBRARIES})
target_link_libraries(track_10 -lusbcan -lcanlib)


include_directories(/usr/include)
add_executable(track_carto_10 src/track_carto_10.cpp)
target_link_libraries(track_carto_10 ${catkin_LIBRARIES})
target_link_libraries(track_carto_10 -lusbcan -lcanlib)

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_ym src/track_ym.cpp)
target_link_libraries(track_ym ${catkin_LIBRARIES})
target_link_libraries(track_ym -lusbcan -lcanlib)

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_carto_ym src/track_carto_ym.cpp)
target_link_libraries(track_carto_ym ${catkin_LIBRARIES})
target_link_libraries(track_carto_ym -lusbcan -lcanlib)

include_directories(/usr/include)
add_executable(track_fast_ym src/track_fast_ym.cpp)
target_link_libraries(track_fast_ym ${catkin_LIBRARIES})
target_link_libraries(track_fast_ym -lusbcan -lcanlib)

include_directories(/usr/include)
add_executable(track_carto_fast src/track_carto_fast.cpp)
target_link_libraries(track_carto_fast ${catkin_LIBRARIES})
target_link_libraries(track_carto_fast -lusbcan -lcanlib)

include_directories(/usr/include)
add_executable(ym_point src/ym_point.cpp)
target_link_libraries(ym_point ${catkin_LIBRARIES})


include_directories(/usr/include)

include_directories(/usr/include)
add_executable(tar_theta src/tar_theta.cpp)
target_link_libraries(tar_theta ${catkin_LIBRARIES})

include_directories(/usr/include)
include_directories(/usr/include/eigen)
add_executable(track_pp src/track_pp.cpp)
target_link_libraries(track_pp ${catkin_LIBRARIES})
target_link_libraries(track_pp -lusbcan -lcanlib)

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_carto_pp src/track_carto_pp.cpp)
target_link_libraries(track_carto_pp ${catkin_LIBRARIES})
target_link_libraries(track_carto_pp -lusbcan -lcanlib)

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_carto_curv src/track_carto_curv.cpp)
target_link_libraries(track_carto_curv ${catkin_LIBRARIES})
target_link_libraries(track_carto_curv -lusbcan -lcanlib)


include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_carto_line src/track_carto_line.cpp)
target_link_libraries(track_carto_line ${catkin_LIBRARIES})
target_link_libraries(track_carto_line -lusbcan -lcanlib)


include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_gd_line src/track_gd_line.cpp)
target_link_libraries(track_gd_line ${catkin_LIBRARIES})
target_link_libraries(track_gd_line -lusbcan -lcanlib)

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_gd_curv src/track_gd_curv.cpp)
target_link_libraries(track_gd_curv ${catkin_LIBRARIES})
target_link_libraries(track_gd_curv -lusbcan -lcanlib)

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_line src/track_line.cpp)
target_link_libraries(track_line ${catkin_LIBRARIES})
target_link_libraries(track_line -lusbcan -lcanlib)


include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_demo src/track_demo.cpp)
target_link_libraries(track_demo ${catkin_LIBRARIES})
target_link_libraries(track_demo -lusbcan -lcanlib)

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_plan src/track_plan.cpp)
target_link_libraries(track_plan ${catkin_LIBRARIES})
target_link_libraries(track_plan -lusbcan -lcanlib)

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(adv_start src/adv_start.cpp)
target_link_libraries(adv_start ${catkin_LIBRARIES})
target_link_libraries(adv_start -lusbcan -lcanlib)

include_directories(/usr/include)
add_executable(start src/start.cpp)
target_link_libraries(start ${catkin_LIBRARIES})

include_directories(/usr/include)
add_executable(pose src/pose.cpp)
target_link_libraries(pose ${catkin_LIBRARIES})

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(track_fangyi src/track_fangyi.cpp)
target_link_libraries(track_fangyi ${catkin_LIBRARIES})
target_link_libraries(track_fangyi -lusbcan -lcanlib)

include_directories(/usr/include)
include_directories("/usr/include/eigen3")  ##坐标系转换
add_executable(can_send src/can_send.cpp)
target_link_libraries(can_send ${catkin_LIBRARIES})
target_link_libraries(can_send -lusbcan -lcanlib) ##CAN库文件

include_directories(/usr/include)
include_directories("/usr/include/eigen3")
add_executable(listener_tf src/listener_tf.cpp)
target_link_libraries(listener_tf ${catkin_LIBRARIES})
target_link_libraries(listener_tf -lusbcan -lcanlib)

