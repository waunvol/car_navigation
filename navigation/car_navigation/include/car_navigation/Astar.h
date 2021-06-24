#ifndef ASTAR_H
#define ASTAR_H

#include <queue>
#include <vector>
#include <set>
#include <ros/ros.h>
#include <map_server/image_loader.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include "Dijkstra.h"

#define x_t 4
#define y_t 7       //target x and target y

using namespace std;

//可改写成继承自dijkstra类的子类，把曼哈顿和search重写一下即可

/*
struct waypoint
{
    int x, y;
    waypoint *front = nullptr;
};
struct graph
{
    int weight=1;     //权重
    int distance = 65535;
    int visit = 0;                //访问标记
    waypoint *way = nullptr;      //路径点

    struct neighbor //定义相邻点结构体
    {
        graph **data;
        neighbor *next=nullptr;
    };
    neighbor *itsNeighbor =nullptr;        //相邻点
};*/

class astar
{
private:
    vector<graph **> ShortPathTable;     //存储目前的最短路径点
    nav_msgs::GetMap map_msg;
    // nav_msgs::Path shortestPath;

    double resolution;                  //保存地图的单位
    int height, width;                  //保存解析的图片的长宽

    void decodeMap();   //地图解析
    int MHTdistance(int x, int y)   //计算曼哈顿距离
    {
        x = x_t - x;
        if(x<0)
            x = -x;
        y = y_t - y;
        if(y<0)
            y = -y;
        return x + y;
    };


public:
    graph*** map;
    int start_x, start_y;
    int target_x, target_y;

    astar(nav_msgs::GetMap rec_map);
    ~astar();

    void setStartPoint(double x, double y)
    {
        start_x = x/resolution;     //切换为类中的计量单位
        start_y = y/resolution;
    };
    void setTargetPoint(double x, double y)
    {
        target_x = x/resolution;
        target_y = y/resolution;
    };
    bool search();
    void updateMap(){};

};

























#endif