cmake_minimum_required(VERSION 3.0.2)
project(uav_waypoint_manager)

## 编译选项
add_compile_options(-std=c++14)

## 查找catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

## 声明服务
add_service_files(
  FILES
  LoadWaypoints.srv
  SaveWaypoints.srv
  ClearWaypoints.srv
)

## 生成消息
 generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## catkin配置
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
)

## 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 可执行文件
add_executable(waypoint_manager src/waypoint_manager.cpp)
add_dependencies(waypoint_manager ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(waypoint_manager ${catkin_LIBRARIES})

## 安装规则
install(TARGETS waypoint_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)