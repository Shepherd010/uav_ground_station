cmake_minimum_required(VERSION 3.0.2)
project(uav_navigator)

## 编译选项
add_compile_options(-std=c++14)

## 查找catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  message_generation
  tf
  rostest
  rosbag
)

## 声明消息
add_message_files(
  FILES
  NavigatorStatus.msg
  ExperimentMetrics.msg
)

## 声明服务
add_service_files(
  FILES
  NavigatorCommand.srv
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
    nav_msgs
    mavros_msgs
    message_runtime
    tf
    rosbag
)

## 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 可执行文件: navigator
add_executable(navigator src/navigator.cpp)
add_dependencies(navigator ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(navigator ${catkin_LIBRARIES})

## 可执行文件: safety_monitor
add_executable(safety_monitor src/safety_monitor.cpp)
add_dependencies(safety_monitor ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(safety_monitor ${catkin_LIBRARIES})

## 可执行文件: logger
add_executable(logger src/logger.cpp)
add_dependencies(logger ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(logger ${catkin_LIBRARIES})

## 可执行文件: experiment_recorder
add_executable(experiment_recorder src/experiment_recorder.cpp)
add_dependencies(experiment_recorder ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(experiment_recorder ${catkin_LIBRARIES})

## 安装规则
install(TARGETS navigator safety_monitor logger experiment_recorder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 测试（已移除 test/ 目录，测试框架保留以便将来添加）
# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(test_uav_system test/test_system.launch test/test_system.cpp)
#   add_dependencies(test_uav_system ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#   target_link_libraries(test_uav_system ${catkin_LIBRARIES})
# endif()