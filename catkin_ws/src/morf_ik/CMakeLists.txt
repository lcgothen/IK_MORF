cmake_minimum_required(VERSION 3.16.3)
project(morf_ik)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR /usr/share/OpenCV/)
set(fann_DIR ~/leonor/fann/)
set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
list(APPEND CMAKE_MODULE_PATH /home/leonor/CoppeliaSim/programming/libPlugin/cmake)
set(CoppeliaSim_DIR ${COPPELIASIM_ROOT_DIR})
find_package(CoppeliaSim 4.1.0.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  fann
)
find_package(Boost REQUIRED COMPONENTS system)

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
#  LIBRARIES morf_IK
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/morf_IK.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/morf_IK_node.cpp)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_morf_IK.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# add_executable(main_eqs src/main_eqs.cpp)
# add_executable(main_nn src/main_nn.cpp)
# coppeliasim_add_executable(main_eqs_data 
#     SOURCES
#     src/main_eqs_data.cpp
#     ${COPPELIASIM_COMMON_DIR}/shared_memory.c
#     ${COPPELIASIM_ROOT_DIR}/programming/remoteApi/extApi.c
#     ${COPPELIASIM_ROOT_DIR}/programming/remoteApi/extApiPlatform.c)


coppeliasim_add_executable(main 
    SOURCES
    src/main.cpp
    ${COPPELIASIM_COMMON_DIR}/shared_memory.c
    ${COPPELIASIM_ROOT_DIR}/programming/remoteApi/extApi.c
    ${COPPELIASIM_ROOT_DIR}/programming/remoteApi/extApiPlatform.c)
# add_executable(main_nn_data src/main_nn_data.cpp)
# add_executable(gen_data src/gen_data_ik.cpp)
coppeliasim_add_executable(gen_data 
    SOURCES
    src/gen_data_sim.cpp
    ${COPPELIASIM_COMMON_DIR}/shared_memory.c
    ${COPPELIASIM_ROOT_DIR}/programming/remoteApi/extApi.c
    ${COPPELIASIM_ROOT_DIR}/programming/remoteApi/extApiPlatform.c)
add_executable(gen_data_ik src/gen_data_ik.cpp)
add_executable(sort_data src/sort_data.cpp)
add_executable(sort_data2 src/sort_data2.cpp)
add_executable(nn src/nn.cpp)
add_executable(calib src/calibration.cpp)
add_executable(morf_controller_real src/test_real.cpp include/coordinates_real.cpp include/controller_real.cpp)
add_executable(gen_data_real src/gen_data_real.cpp include/coordinates_real.cpp include/controller_real.cpp)


target_compile_definitions(main PRIVATE NON_MATLAB_PARSING)
target_compile_definitions(main PRIVATE MAX_EXT_API_CONNECTIONS=255)
target_include_directories(main PRIVATE ${COPPELIASIM_INCLUDE_DIR})
target_include_directories(main PRIVATE ${COPPELIASIM_ROOT_DIR}/programming/remoteApi)
target_compile_definitions(main PRIVATE NON_MATLAB_PARSING)
target_compile_definitions(main PRIVATE MAX_EXT_API_CONNECTIONS=255)
target_include_directories(main PRIVATE ${COPPELIASIM_INCLUDE_DIR})
target_include_directories(main PRIVATE ../remoteApi)
target_link_libraries(main Boost::system)
target_link_libraries(main Threads::Threads rt)

target_compile_definitions(gen_data PRIVATE NON_MATLAB_PARSING)
target_compile_definitions(gen_data PRIVATE MAX_EXT_API_CONNECTIONS=255)
target_include_directories(gen_data PRIVATE ${COPPELIASIM_INCLUDE_DIR})
target_include_directories(gen_data PRIVATE ${COPPELIASIM_ROOT_DIR}/programming/remoteApi)
target_compile_definitions(gen_data PRIVATE NON_MATLAB_PARSING)
target_compile_definitions(gen_data PRIVATE MAX_EXT_API_CONNECTIONS=255)
target_include_directories(gen_data PRIVATE ${COPPELIASIM_INCLUDE_DIR})
target_include_directories(gen_data PRIVATE ../remoteApi)
target_link_libraries(gen_data Boost::system)
target_link_libraries(gen_data Threads::Threads rt)

add_library(coords include/coordinates.cpp)
add_library(control include/controller.cpp)


target_link_libraries(control 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      fann)
target_link_libraries(main 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      coords
                      control
                      fann)
target_link_libraries(morf_controller_real 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      fann)
target_link_libraries(gen_data_real 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      fann)
target_link_libraries(gen_data_ik 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      fann)
target_link_libraries(gen_data 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      coords
                      control)
target_link_libraries(sort_data 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      coords
                      control)
target_link_libraries(sort_data2 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      coords
                      control)
target_link_libraries(nn 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      coords
                      control
                      fann)
target_link_libraries(calib 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      coords
                      control)