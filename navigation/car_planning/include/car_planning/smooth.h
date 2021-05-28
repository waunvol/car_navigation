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

    queue<pair<float, float>> _queue;

    void BezierCalculate_(int level, float t);//
    void BezierCalculate(int level, float t);

public:
    smooth();
    ~smooth();

    vector<pair<float, float>> PathSmooth(const vector<pair<float, float>> &path);
};

smooth::smooth(/* args */)
{
}

smooth::~smooth()
{
}



#endif
