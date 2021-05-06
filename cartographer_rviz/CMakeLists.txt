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

cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

#project(cartographer_rviz)

#set(PACKAGE_DEPENDENCIES
#  cartographer_ros
#  cartographer_ros_msgs
#  message_runtime
#  roscpp
#  roslib
#  rviz
#)

#if(WIN32)
#  set(Boost_USE_STATIC_LIBS FALSE)
#endif()
#find_package(Boost REQUIRED COMPONENTS system iostreams)
#find_package(cartographer REQUIRED)
#include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
#google_initialize_cartographer_project()

#find_package(absl REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

#catkin_package(
#  CATKIN_DEPENDS
#    message_runtime
#    ${PACKAGE_DEPENDENCIES}
#  INCLUDE_DIRS "."
#)

#file(GLOB_RECURSE ALL_SRCS "cartographer_rviz/*.cc" "cartographer_rviz/*.h")
#set(CMAKE_AUTOMOC ON)

#if(rviz_QT_VERSION VERSION_LESS "5")
#  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
#  include(${QT_USE_FILE})
#else()
#  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
#  set(QT_LIBRARIES Qt5::Core Qt5::Widgets)
#  include_directories(${Qt5Widgets_INCLUDE_DIRS})
#endif()
#add_definitions(-DQT_NO_KEYWORDS)
#add_library(${PROJECT_NAME} ${ALL_SRCS})
#target_link_libraries(${PROJECT_NAME} PUBLIC ${QT_LIBRARIES})

## Add the binary directory first, so that port.h is included after it has
## been generated.
#target_include_directories(${PROJECT_NAME} PUBLIC
#    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
#    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
#    $<INSTALL_INTERFACE:include>
#)

#target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
#  "${EIGEN3_INCLUDE_DIR}")
#target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

#target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
#  "${Boost_INCLUDE_DIRS}")
#target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

#target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
#set_target_properties(${PROJECT_NAME} PROPERTIES
#  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

#target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

## On windows, rviz won't find the DLL in CATKIN_PACKAGE_BIN_DESTINATION,
## but it will in CATKIN_PACKAGE_LIB_DESTINATION?
#if(WIN32)
#  set(RUNTIME_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#else()
#  set(RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#endif()

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
#)

#install(FILES rviz_plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY ogre_media
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
