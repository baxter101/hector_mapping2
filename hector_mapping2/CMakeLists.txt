cmake_minimum_required(VERSION 2.8.3)
project(hector_mapping2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS sensor_msgs geometry_msgs nav_msgs laser_geometry tf tf_conversions rosconsole cmake_modules roscpp nodelet hector_timing)

## System dependencies are found with CMake's conventions
## Find boost thread
#find_package(Boost REQUIRED COMPONENTS thread)
find_package(Boost REQUIRED)

## Find eigen
find_package(Eigen REQUIRED)

## Find ceres-solver
find_package(Ceres REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_mapping2
  CATKIN_DEPENDS sensor_msgs geometry_msgs nav_msgs visualization_msgs laser_geometry roscpp nodelet
  DEPENDS EIGEN Boost
)

###########
## Build ##
###########

## Set some compiler flags
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE:        ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "CMAKE_CXX_FLAGS_MINSIZEREL:     ${CMAKE_CXX_FLAGS_MINSIZEREL}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG:          ${CMAKE_CXX_FLAGS_DEBUG}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
if(CERES_LIBRARIES)
  add_definitions(-DHAVE_CERES)
  include_directories(${CERES_INCLUDES})
endif()

## Declare a cpp library
add_library(hector_mapping2
  src/map.cpp
  src/map/conversion.cpp
  src/map/occupancy.cpp
  src/map/types.cpp
  src/scan.cpp
  src/matcher.cpp
  src/matcher/ceres.cpp
  src/matcher/interpolated_map.hpp
  src/matcher/residuals.hpp
  src/matcher/solver.hpp
  src/util/marker_conversion.cpp
  src/hector_mapping.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(hector_mapping2
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CERES_LIBRARIES}
)

## Declare a cpp executable
add_executable(hector_mapping2_node src/node.cpp)
target_link_libraries(hector_mapping2_node hector_mapping2 ${catkin_LIBRARIES})
set_target_properties(hector_mapping2_node PROPERTIES OUTPUT_NAME hector_mapping)

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
install(TARGETS hector_mapping2 hector_mapping2_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hector_mapping2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
