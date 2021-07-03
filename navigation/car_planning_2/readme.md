# 基于混合A* 和 RRT 算法 的粗路径搜索

本功能包依赖于上一级的目录构建的环境（最后将整个目录拉到工作空间下编译）

## 运行
在 `roslaunch car_gazebo car_gazebo.launch` 后，等待rviz和gazebo完成加载， 在新终端中键入 `roslaunch car_planning_2 planner_2.launch`

## 说明
1. 使用RVIZ界面上“2D Nav Goal”给小车指定一个目标, rviz 上就会显示出算法搜索到的路径；
2. 橙色线为 混合A* 算法搜索到的路径， 深红色则为 RRT 算法搜索到的；
3. 目前尚未小车依路径运动的功能；
4. RRT算法使用了梯度下降算法对其进行了轨迹圆滑；
5. pkg包中reedsshepp曲线是由OMPL上移植的。
