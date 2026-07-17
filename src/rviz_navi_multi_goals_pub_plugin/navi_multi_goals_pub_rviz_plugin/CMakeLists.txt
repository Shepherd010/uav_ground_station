cmake_minimum_required(VERSION 3.0.2)
project(navi_multi_goals_pub_rviz_plugin)

# 查找依赖
find_package(catkin REQUIRED COMPONENTS
  rviz
  geometry_msgs
  std_msgs
  actionlib_msgs
  uav_waypoint_manager
  tf
)

catkin_package(
  CATKIN_DEPENDS
    rviz
    geometry_msgs
    std_msgs
    actionlib_msgs
    uav_waypoint_manager
    tf
)

include_directories(${catkin_INCLUDE_DIRS})

# Qt 自动 MOC 生成
set(CMAKE_AUTOMOC ON)

# 根据 rviz 的 Qt 版本选择
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

# 避免 boost signals 与 Qt signals/slots 冲突
add_definitions(-DQT_NO_KEYWORDS)

# 源文件
set(SOURCE_FILES
  src/multi_navi_goal_panel.cpp
  ${MOC_FILES}
)

# 构建共享库
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

# 安装
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml")
