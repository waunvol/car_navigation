# 简介
本项目在ROS（机器人操作系统）的基础上，在gazebo上进行的一个运动规划的仿真实验。其中Astar和dijkstra算法是依据官方github源码的思想来写（因为写的很好。。），其他均为自行编写。

展示视频：<https://www.bilibili.com/video/BV1v54y1V7qV>
## 运行
1. 复制至catkin工作空间的src后编译
2. 运行`roslaunch car_gazebo car_gazebo.launch`
3. 待Gazebo和RVIZ启动完成后，运行`roslaunch car_planning planningNode.launch`
## 使用
1. 使用RVIZ上“2D new goal”给小车指定一个目标，小车便会想目标开始移动
2. 绿色路径dijkstra算法寻路结果，蓝色路径为A* 算法寻路结果
3. 可在小车运动过程指定新的目标点
## 详细介绍
大致的运行过程就是读取静态地图，根据给定的参数生成一个costmap；在接受到目标后，运用dijkstra或是Astar算法铺出一个权重网络，根据权重先得到一个梯度路径，然后用贝塞尔曲线圆滑梯度路径并发布，就是RVIZ上显示路径结果，最后运用PID算法控制小车往目标移动
* 模型: 有solidwork导出的urdf更改成xacro文件，在 `navigation/car/robots`；
* 环境：利用gazebo搭建成的墙壁，launch文件和环境配置在 `navigation/car_gazebo`；
* 地图：在搭建的gazebo仿真环境中利用gmapping搭建（都是原始参数，没有深入调整），在 `navigation/car/map`；
* 路径构建：包含costmap，两种寻路算法，梯度路径和贝塞尔曲线，在 `navigation/car_planning`；
* 定位：仅利用tf与里程计进行定位，借助state publisher和tf转换生成完整的tf树， `navigation/car_state`；
* 运动：PID控制， `navigation/car_motion_ctrl`；
