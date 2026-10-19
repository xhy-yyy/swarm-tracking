cmake_minimum_required(VERSION 3.0.2)
project(fly_px4)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  sensor_msgs
  std_msgs
  track_msgs
  traj_utils
)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS
  geometry_msgs mavros_msgs nav_msgs quadrotor_msgs roscpp sensor_msgs std_msgs track_msgs traj_utils)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fly_px4_2 src/fly_px4_2.cpp src/control_px4.cpp src/function_px4.cpp)
add_dependencies(fly_px4_2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(fly_px4_2 ${catkin_LIBRARIES} pthread)

# 巡逻主控共用控制层，状态机和恢复逻辑位于 fly_px4_1.cpp。
add_executable(fly_px4_1 src/fly_px4_1.cpp src/control_px4.cpp)
add_dependencies(fly_px4_1 ${catkin_EXPORTED_TARGETS})
target_link_libraries(fly_px4_1 ${catkin_LIBRARIES} pthread)

add_executable(ego_to_mavros src/ego_to_mavros.cpp)
add_dependencies(ego_to_mavros ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_to_mavros ${catkin_LIBRARIES} pthread)

add_executable(ego_trigger src/ego_trigger.cpp)
add_dependencies(ego_trigger ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_trigger ${catkin_LIBRARIES} pthread)

catkin_install_python(PROGRAMS
  scripts/cloud_transformer.py
  scripts/odom_world_relay.py
  scripts/laser_scan_to_world_cloud.py
  scripts/body_band_cloud.py
  scripts/expl_monitor.py
  scripts/swarm_vision.py
  scripts/swarm_fusion.py
  scripts/preflight_check.py
  scripts/show_swarm_cams.py
  scripts/check_patrol_routes.py
  scripts/check_patrol_maps.py
  scripts/patrol_world_geometry.py
  scripts/start_positions.py
  scripts/swarm_gimbal_recovery.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# shell 包装使用普通 install；Python 节点使用 catkin_install_python。
install(PROGRAMS
  scripts/run_swarm_vision.sh
  scripts/with_catkin_libs.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# Importable helpers beside the vision/fusion scripts in installed layouts.
install(FILES scripts/camera_pose_provider.py scripts/camera_gimbal_kinematics.py
  scripts/world_frame_transform.py scripts/publication_prediction.py
  scripts/gimbal_recovery_logic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_test(NAME gimbal_recovery COMMAND /usr/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gimbal_recovery.py)
  add_test(NAME body_band_cloud COMMAND /usr/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_body_band_cloud.py)
  add_executable(test_altitude_hold test/test_altitude_hold.cpp)
  add_test(NAME altitude_hold COMMAND test_altitude_hold)
  # Capture actual PositionTarget publications on an isolated loopback master.
  add_executable(test_control_altitude test/test_control_altitude.cpp src/control_px4.cpp)
  add_dependencies(test_control_altitude ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_control_altitude ${catkin_LIBRARIES})
  add_test(NAME control_altitude COMMAND /usr/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/test/smoke_control_altitude.py
    --executable $<TARGET_FILE:test_control_altitude>)
  # Exercise the same reference generator used by fly_px4_1, without ROS/PX4.
  add_executable(test_track_yaw_controller test/test_track_yaw_controller.cpp)
  add_test(NAME track_yaw_controller COMMAND test_track_yaw_controller)
  add_executable(test_tracking_navigation_policy test/test_tracking_navigation_policy.cpp)
  add_test(NAME tracking_navigation_policy COMMAND test_tracking_navigation_policy)
  # 跟踪避障: 净空场/站位/速度过滤单测 + 原方法对比闭环 (可带真实地图轮廓做压力测试)
  add_executable(test_track_obstacle_avoidance test/test_track_obstacle_avoidance.cpp)
  add_test(NAME track_obstacle_avoidance COMMAND test_track_obstacle_avoidance)
endif()

install(TARGETS fly_px4_1 fly_px4_2 ego_to_mavros ego_trigger
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
