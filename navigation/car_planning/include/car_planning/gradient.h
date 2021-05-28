#ifndef GRID_H_
#define GRID_H_
#include "cost.h"
#include <nav_msgs/Path.h>

#define undefined 65536

class gradient
{
private:
    int wid, hig;
    float *gradx, *grady;
    nav_msgs::Path path;
    ros::NodeHandle n;
    //path的变量存储值

    bool pathSearch(int *potential, float start_x, float start_y, float end_x, float end_y, vector<pair<float, float>> &path);
    void addGrad(int now, int* potential);
    int getIndex(float x, float y)
    {
        int tmp;
        tmp = (x+0.025)/0.05;
        x = tmp;
        tmp = (y+0.025)/0.05;
        y = tmp;
        return x+y*wid;
    };
    void gradient_init()
    {
        memset(gradx, 0, wid*hig * sizeof(float));
        memset(grady, 0, wid*hig * sizeof(float));  //初始化所有值都初始化为0
    }

public:
    gradient(int width=1, int hight=1):wid(width), hig(hight)   //存储斜率
    {
        gradx = new float[wid*hig];     
        grady = new float[wid*hig];
    };
    ~gradient()
    {
        delete gradx;
        delete grady;
    };
    void setSize(int width, int hight)
    {
        wid = width;
        hig = hight;
    }

    void calculatePath(int *potential, float start_x, float start_y, vector<pair<float, float>> goal, vector<vector<pair<float, float>>> &path);

};








#endif