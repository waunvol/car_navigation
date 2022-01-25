#ifndef SMOOTH_H
#define SMOOTH_H
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <queue>

using namespace std;

class smooth
{
private:

    void BezierCalculate(int level, float t);
    pair<float,float> BezierSingalPointCalculate(int level, float t, queue<pair<float, float>> ctrl_pt);


public:
    smooth();
    ~smooth();

    vector<pair<float, float>> PathSmooth(int order ,const vector<pair<float, float>> &path);
};

smooth::smooth(/* args */)
{
}

smooth::~smooth()
{
}



#endif
