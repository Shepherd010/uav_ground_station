cmake_minimum_required(VERSION 3.0.2)
project(rviz_waypoint_panel)

## 编译选项
add_compile_options(-std=c++14)

set(CMAKE_AUTOMOC ON)

## 查找catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  geometry_msgs
  std_msgs
  visualization_msgs
  actionlib_msgs
  nav_msgs
  mavros_msgs
  uav_navigator
  uav_waypoint_manager
  tf
)

## 查找 yaml-cpp
find_package(yaml-cpp REQUIRED)

## 查找Qt
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

## catkin配置
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rviz
    geometry_msgs
    std_msgs
    visualization_msgs
    actionlib_msgs
    nav_msgs
    mavros_msgs
    tf
)

## 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 源文件
set(SOURCE_FILES
  src/waypoint_panel.cpp
)

## 插件库
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

## 安装规则
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)