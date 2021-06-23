#ifndef RRT__H
#define RRT__H
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <random>

class RRT_planner
{
public:
    struct waypoint
    {
        std::pair<float, float> point;
        waypoint* parent;
    };
    
    const std::vector<int8_t> *COST;    //costmap
    int wid, hig;
    std::vector<waypoint*> route_tree;  //路径树
    std::random_device rd;
    // std::mt19937 rdnum(rd());
    std::pair<float, float> goal;
    float step = 0.1;

    RRT_planner(/* args */);
    ~RRT_planner();

    //设置costmap以及相关参数
    void setCOSTMAP(int hight, int width, const std::vector<int8_t> *costmap)   
    {
        wid = width;
        hig = hight;
        COST = costmap;
    }
    //搜索路径
    bool searchRoute(const std::vector<float> *start, const std::vector<float> *goal);  
    int GetIndex(float x, float y);
    //获得搜索到的路径
    std::vector<std::vector<float>> getRoute();      

private:
    //获得最近点
    waypoint* nearestPoint(std::pair<float, float> target);
    //判断是否碰撞， 若有碰撞则返回true
    bool CheckObstacle(float x, float y);
    //获得随机点       
    std::pair<float, float> getRandomPoint();   
    void addPoint(waypoint* nearest, const std::pair<float, float> &target);
    //释放空间
    void free();                                
    

};














#endif