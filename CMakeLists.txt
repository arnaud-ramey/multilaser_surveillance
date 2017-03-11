### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(multilaser_surveillance)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
SET(DEPS  bayes_people_tracker  cv_bridge  roscpp  tf  vision_utils)
find_package(catkin REQUIRED COMPONENTS ${DEPS})
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
### 5 - Invoke message/service/action generation (generate_messages())
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${DEPS}
  DEPENDS
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
# lmfit
include_directories(src/lmfit-6.1/lib)
add_library(lmfit
  src/lmfit-6.1/lib/lmmin.c
  src/lmfit-6.1/lib/lmcurve.c
  src/lmfit-6.1/lib/lmcurve_tyd.c
)
#add_executable(nonlin1  src/lmfit-6.1/demo/nonlin1.c)
#target_link_libraries(nonlin1  lmfit ${catkin_LIBRARIES})
#add_executable(curve1  src/lmfit-6.1/demo/curve1.c)
#target_link_libraries(curve1  lmfit ${catkin_LIBRARIES})

SET(PKG_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/multilaser_surveillance)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(map_builder_watcher.exe
  src/map_builder_watcher.cpp
  ${PKG_INC_DIR}/lite_obstacle_map.h
  ${PKG_INC_DIR}/map_builder_watcher.h)
target_link_libraries(map_builder_watcher.exe ${catkin_LIBRARIES})

add_executable(2dclusterer.exe
  src/2dclusterer.cpp
  ${PKG_INC_DIR}/2dclusterer.h)
target_link_libraries(2dclusterer.exe  lmfit  ${catkin_LIBRARIES})

add_executable(pose_matcher.exe src/pose_matcher.cpp)
target_link_libraries(pose_matcher.exe  ${catkin_LIBRARIES})


### 8 - Tests to build (catkin_add_gtest())
catkin_add_gtest(gtest_clusterer  test/gtest_clusterer.cpp)
target_link_libraries(gtest_clusterer  lmfit  ${catkin_LIBRARIES})
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
