### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(multilaser_surveillance)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
SET(DEPS  bayes_people_tracker  cv_bridge  roscpp  tf)
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
SET(PKG_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/multilaser_surveillance)
include_directories( include ${catkin_INCLUDE_DIRS})
add_executable(multilaser_surveillance.exe
  src/multilaser_surveillance.cpp
  ${PKG_INC_DIR}/wanderer.h)
target_link_libraries(multilaser_surveillance.exe ${catkin_LIBRARIES})

add_executable(2dclusterer.exe
  src/2dclusterer.cpp
  ${PKG_INC_DIR}/2dclusterer.h)
target_link_libraries(2dclusterer.exe ${catkin_LIBRARIES})
### 8 - Tests to build (catkin_add_gtest())
catkin_add_gtest(gtest_clusterer  test/gtest_clusterer.cpp)
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
