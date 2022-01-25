#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include "rs_path.h"
#include <memory>

using namespace std;

class hybridAstar
{
public:
    struct waypoint
    {
        vector<float> point;
        waypoint* parent;  //第一个记为父母，其余均为子树指针
    };

    struct greater1 {
            bool operator()(const waypoint* a, const waypoint* b) const {
                return a->point[4] > b->point[4];
            }
    };
    double a=0.02, rho=0.30, v=0.15;  //最大加速度与转弯半径,速度
    waypoint *cur_ptr;
    // shared_ptr<waypoint> cur_ptr1;
    const vector<int8_t> *COST;
    vector<waypoint*> openSET;  //用openSET排序选出最优点
    vector<waypoint*> Pending;  //用pending记录点是否已被搜索过，且仅记录最优点（一个pending仅对应一个点）
    vector<waypoint*> route_tree;   //存放生成的路径树
    int wid, hig;

    hybridAstar(/* args */);
    ~hybridAstar();
    void setCOST(int hight, int width, const vector<int8_t> *costmap)
    {
        wid = width;
        hig = hight;
        COST = costmap;
        
    }
    bool calculateRoute(const vector<float> *start, const vector<float> *goal); //传入带位姿的起点与终点和 costmap
    void free();         //释放占用内存
    vector<vector<float>> getRoute();

private:
    // waypoint *root=nullptr; //记录根

    //若检测到障碍，返回true
    bool CheckObstacle(float x, float y);   
    int GetIndex(float x, float y);
    bool GetReedsShepp(const vector<float> *goal);
    void add_set(double acceleration, double radius, const vector<float> &goal);
    bool CheckSet(int index);    //判断是否在close集合内
    // bool CheckOpenSet(const waypoint *ptr);

};


























#endif