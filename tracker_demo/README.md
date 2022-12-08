# tracker_demo
## 运行需知

1.启动单机仿真时
- 若是使用hunter进行仿真,则要修改`/hunter2/hunter2_control/config/ctrl_ackermann_steering_controller.yaml` 中有关hunter odom配置的2处信息,注意与robot_num对应上
- 修改`/hunter2/hunter2_control/launch/hunter2_v_contol.launch` 中 `robot_namespace`参数的设置(/robot1 || /robot2)
- 修改`tracker_demo/src/getTargetinfor_point.cpp` 中关于define robot_type的值
- 修改`mpc/scripts/local_planner.py` 中import mpc的文件(Ackerman || Differential)以及 变量`robot_type` 的值

2.启动多机仿真时

hunter_track_scout.launch
- 修改`/hunter2/hunter2_control/config/ctrl_ackermann_steering_controller.yaml` 中有关hunter odom配置的2处信息,注意与robot_num对应上
- 修改`/hunter2/hunter2_control/launch/hunter2_v_contol.launch` 中 `robot_namespace`参数的设置(/robot1 || /robot2)
- 修改`tracker_demo/src/getTargetinfor_robot.cpp` 中主程序有关 odom_sub1, odom_sub2的注释选择放开
- 修改`mpc/scripts/local_planner.py` 中import mpc的文件(Ackerman || Differential)以及 变量`robot_type` 的值

scout_track_hunter
- 修改`/hunter2/hunter2_control/config/ctrl_ackermann_steering_controller.yaml` 中有关hunter odom配置的2处信息,注意与robot_num对应上
- 修改`/hunter2/hunter2_control/launch/hunter2_v_contol.launch` 中 `robot_namespace`参数的设置(/robot1 || /robot2)
- 修改`tracker_demo/src/getTargetinfor_robot.cpp` 中主程序有关 `odom_sub1`, `odom_sub2`的注释选择放开
- 修改`mpc/scripts/local_planner.py` 中import mpc的文件(Ackerman || Differential)以及 变量`robot_type` 的值