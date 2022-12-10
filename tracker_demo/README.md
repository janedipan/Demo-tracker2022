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

## 展示

gazebo仿真[here](images/mpc-gazebo.mp4)

rviz显示[here](images/mpc-rviz.mp4)

## 修改记录
2022-12-10
- hunterSE 自己发布的odom不准确,需要手动订阅`/gazebe_model`话题中hunter模型的odom,处理一下再发布出去供状态积节点使用；

## 问题记录
1.尝试做路径跟踪的时候,最初的想法是增加一个deque容器存放订阅得到的target_odom,在push_back之前添加一个判断,当前点与传入的点距离在一定范围之外才允许压入,但实际跑的时候发现存在一直压入点的意外情况,分析可能原因:1)mpc求解无法到达准确的点 2)因为deque容器的维护不是在同一个频率下进行了,push_back以50Hz进行,pop_front以20Hz进行,可能会有一定的问题
