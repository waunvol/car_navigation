# 简介
本项目在ROS（机器人操作系统）的基础上，在gazebo上进行的一个运动规划的仿真实验。
## 运行
1. 复制至catkin工作空间的src后编译
2. 运行`roslaunch car_gazebo car_gazebo.launch`
3. 待Gazebo和RVIZ启动完成后，运行`roslaunch car_planning planningNode.laucnch`
## 使用
1. 使用RVIZ上“2D new goal”给小车指定一个目标，小车便会想目标开始移动
2. 绿色路径dijkstra算法寻路结果，蓝色路径为A* 算法寻路结果
3. 可在小车运动过程指定新的目标点
## 详细介绍
大致的运行过程就是读取静态地图，根据给定的参数生成一个costmap；在接受到目标后，运用dijkstra或是Astar算法铺出一个权重网络，根据权重先得到一个梯度路径，然后用贝塞尔曲线圆滑梯度路径并发布，就是RVIZ上显示路径结果，最后运用PID算法控制小车往目标移动
* 模型
