#include "astar.h"

bool astar::calculatePotential(float start_x, float start_y, vector<pair<float, float>> destination, int* potential)
{
    if(!CheckCostMap())
        return false;

    //potential初始化
    std::fill(potential, potential + wid*hig, 65536);
    
    //A*从起始点开始搜索（寻梯度都是从终点开始）
    queue_.clear();
    int start_i = GetIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    if(destination.size()>1)
        ROS_INFO("astar only support searching the frist goal!"); //"astar only support searching the frist goal!"

    double end_x = destination.at(0).first;
    double end_y = destination.at(0).second;

    
    potential[start_i] = 0;

    int goal_i = GetIndex(end_x, end_y);
    int count = 99999;

    while (queue_.size()>0) //对优先单元格进行搜索
    {
        Index top = queue_.at(0);
        pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();
        if(top.i == goal_i)
        {
            ROS_INFO("planning done!");
            break;}

        addQueue(top.i, top.i + 1, end_x, end_y, potential);
        addQueue(top.i, top.i - 1, end_x, end_y, potential);
        addQueue(top.i, top.i + wid, end_x, end_y, potential);
        addQueue(top.i, top.i - wid, end_x, end_y, potential);

        count--;
        if(!count)//搜索失败
        {
            ROS_ERROR("Astar cant find solution! Please try anthor point");
            return false;
        }
    }
    ROS_INFO("Astar done!");
    return true;
}


void astar::addQueue(int before ,int now, double endx, double endy, int* potential)
{
    //对当前传入的点进行计算优先度等操作
    if((before/wid!=now/wid && (abs(before-now)!=wid)) || now<0 || now>wid*hig)//边界条件
        return;
    if((*COST)[now] >= 70 || (*COST)[now]<=0 )
        return;
    if (potential[now]<65536)
        return;

    int distance = (*COST)[now] + potential[before];

    potential[now] = distance;
    float cost_ = potential[now]+MHTdistance(now, endx, endy);
    queue_.push_back(Index(now, cost_));
    push_heap(queue_.begin(), queue_.end(), greater1());
    
}



