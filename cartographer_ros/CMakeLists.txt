# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(cartographer_ros)

find_package(ament_cmake REQUIRED)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

#add_compile_options(-g)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)

google_initialize_cartographer_project()

find_package(cartographer_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
#find_package(backward_ros REQUIRED)

find_package(absl REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)

if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()


include_directories(
  include
  "."
)

file(GLOB_RECURSE ALL_SRCS "cartographer_ros/*.cc" "cartographer_ros/*.h")
file(GLOB_RECURSE ALL_TESTS "cartographer_ros/*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "cartographer_ros/*_main.cc")
file(GLOB_RECURSE ALL_GRPC_FILES "cartographer_ros/cartographer_grpc/*")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()

add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  #backward_ros
  cartographer
  cartographer_ros_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  visualization_msgs
  rosbag2_compression
  rosbag2_cpp
  rosbag2_storage
  urdf
  urdfdom_headers
)
add_subdirectory("cartographer_ros")
target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

## YAML in ros2 1.0
#target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${YAMLCPP_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} PUBLIC ${YAMLCPP_LIBRARIES})

# Boost not in ros2
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

## URDFDOM in ros2 1.0
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${urdfdom_headers_INCLUDE_DIRS})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

#if (CATKIN_ENABLE_TESTING)
#  foreach(TEST_SOURCE_FILENAME ${ALL_TESTS})
#    get_filename_component(TEST_NAME ${TEST_SOURCE_FILENAME} NAME_WE)
#    catkin_add_gtest(${TEST_NAME} ${TEST_SOURCE_FILENAME})
#    # catkin_add_gtest uses a plain (i.e. no PUBLIC/PRIVATE/INTERFACE) call to
#    # target_link_libraries. That forces us to do the same.
#    target_link_libraries(${TEST_NAME} ${GMOCK_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
#    target_include_directories(${TEST_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
#    target_link_libraries(${TEST_NAME} ${LUA_LIBRARIES})
#    target_include_directories(${TEST_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
#    target_link_libraries(${TEST_NAME} ${catkin_LIBRARIES})
#    add_dependencies(${TEST_NAME} ${catkin_EXPORTED_TARGETS})
#    target_link_libraries(${TEST_NAME} cartographer)
#    target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
#    set_target_properties(${TEST_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
#    # Needed for dynamically linked GTest on Windows.
#    if (WIN32)
#      target_compile_definitions(${TEST_NAME} PUBLIC -DGTEST_LINKED_AS_SHARED_LIBRARY)
#    endif()
#  endforeach()
#endif()

install(DIRECTORY configuration_files
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# ros1 install
#install(DIRECTORY launch urdf configuration_files
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(PROGRAMS scripts/tf_remove_frames.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

## Install source headers.
#file(GLOB_RECURSE HDRS "cartographer_ros/*.h")
#foreach(HDR ${HDRS})
#  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
#  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
#  install(
#    FILES
#      ${HDR}
#    DESTINATION
#      include/${INSTALL_DIR}
#  )
#endforeach()

ament_package()
