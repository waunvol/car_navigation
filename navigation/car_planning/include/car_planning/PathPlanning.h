#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#include <ros/ros.h>
#include <map_server/image_loader.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <memory.h>
#include <cmath>
// #include <iostream>

using namespace std;


class PathPlanning
{
public:
    int wid=0, hig=0;                         //地图长宽
    vector<int8_t> *COST;                  //记录地图的cost

    PathPlanning()
    {

    }

    virtual ~PathPlanning()
    {

    }

    void setCostMap(int hight, int width, vector<int8_t> *costMap)
    {
        hig = hight;
        wid = width;
        COST = costMap;
    }

    bool CheckCostMap()
    {
        if(wid==0|hig==0)
        {
            ROS_ERROR("Please check if costmap is correctly inited");
            return false;
        }
        else return true;
    }

    int GetIndex(double x, double y)        
    {
        int tmp;
        tmp = (x+0.025)/0.05;
        x = tmp;
        tmp = (y+0.025)/0.05;
        y = tmp;
        return x+y*wid;
        //若不用tmp作中间变量，返回的点可能不正确
    }

    int GetMapSize()
    {
        return wid*hig;
    }

    virtual bool calculatePotential(float start_x, float start_y, vector<pair<float, float>> destination, int* potential)=0;
};


#endif