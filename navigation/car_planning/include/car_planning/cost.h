#ifndef COST_H_
#define COST_H_

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
private:
    void rebound(int index,int operation, int threshold);            //遇到障碍物后反弹铺COST格


public:
    double original_x, original_y, resolution;
    int origin;                           //默认的地图起点
    double InterferenceArea, Multiple;    //干涉区域及区域放大系数
    void init();

    int wid, hig;                         //地图长宽
    string frame_id, topicName;                      //地图frame ID
    vector<int8_t> COST;                  //记录地图的cost

    PathPlanning():InterferenceArea(0.25), Multiple(1.0), topicName("static_map")
    {
        
    }

    virtual ~PathPlanning()
    {

    }

    void calculateCOST();

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

    void setTopicName(string name)
    {
        topicName = name;
    }

    void setInterferenceArea(double interference, double multiple)
    {
        InterferenceArea = interference;
        Multiple = multiple;
    }
    
    virtual void calculatePotential(float start_x, float start_y, vector<pair<float, float>> destination, int* potential)=0;

    void testFun();
};


#endif