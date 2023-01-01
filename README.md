## UPDATE
# 2023.1.1
更新了一个简陋的DWA控制器，详情跳转 car_motion_ctrl 下的文档。

# 2022.6.16
本分支施工中，有各种BUG，不要使用。

# 2021.6.24

新增了混合A* 和 RRT 两种算法的路径搜索，详见 `navigation/car_planning_2`


# 简介
这是一个用于个人练手路径规划的小项目，包含了dijkstra, astar, hybirdastar, RRT几种全局路径规划，贝塞尔曲线平滑，梯度下降平滑，一个简单的PID控制器以及一个简陋的仿真。

项目依赖：move_base那一套东西，反正如果当时安装ros是 sudo apt install ros-noetic-desktop ， 那依赖应该不是问题。

其中：car为车子模型相关的内容，car_planning对应是dijkstra, astar, 贝塞尔曲线几个内容， car_planning_2对应是hyburdastar, RRT, 梯度下降平滑几个内容，car_motion_ctrl是控制器，simulation是仿真。

展示视频：<https://www.bilibili.com/video/BV1v54y1V7qV>  【这是2022.5.14更新前的内容，新版本会跟旧版本会略微有所不同】



## 运行
1. 复制至catkin工作空间的src后编译
2. 运行`roslaunch car display.launch`

## 使用
1. 先用RVIZ界面上“2D Nav Goal”旁边绿色箭头的按钮（2D Pose Estimate），把小车定位到地图有效区域（若不在地图区域就进行路径搜索程序就会崩溃）
2. 使用RVIZ界面上“2D Nav Goal”给小车指定一个目标，小车便会向目标开始移动
3. 绿色路径dijkstra算法寻路结果，蓝色路径为A* 算法寻路结果
4. 可在小车运动过程指定新的目标点（目前只有A,D两种算法支持寻路行走，因为控制器只支持往前走）
5. 可在planning_node.launch那里修改“PathTopic”的参数，将寻路行走设定为A算法或者D算法
6. 在car/launch/display.launch中最后一行取消planner_2.launch的注释，可以看hybirdastar和RRT的路径规划效果（不支持寻路行走）
## 详细介绍
大致的运行过程就是读取静态地图，根据给定的参数生成一个costmap；在接受到目标后，运用dijkstra或是Astar算法铺出一个权重网络，根据权重先得到一个梯度路径，然后用贝塞尔曲线圆滑梯度路径并发布，就是RVIZ上显示路径结果，最后运用PID算法控制小车往目标移动
* 模型: 有solidwork导出的urdf更改成xacro文件，在 `navigation/car/robots`；
* 环境：利用gazebo搭建成的墙壁，launch文件和环境配置在 `navigation/car_gazebo`；
* 路径构建：包含costmap，两种寻路算法，梯度路径和贝塞尔曲线，在 `navigation/car_planning`；
* 运动：PID控制， `navigation/car_motion_ctrl`；
