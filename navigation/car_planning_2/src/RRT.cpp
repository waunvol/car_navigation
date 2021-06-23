#include "RRT.h"

RRT_planner::RRT_planner(/* args */)
{
}

RRT_planner::~RRT_planner()
{
}

inline bool RRT_planner::CheckObstacle(float x, float y)
{
    int index = GetIndex(x, y);
    if((*COST)[index]>=60|(*COST)[index]<0)
        return true;
    else
        return false;
}

inline int RRT_planner::GetIndex(float x, float y)        
{
    int tmp;
    tmp = (x+0.025)/0.05;
    x = tmp;
    tmp = (y+0.025)/0.05;
    y = tmp;
    return x+y*wid;
    //若不用tmp作中间变量，返回的点可能不正确
} 

bool RRT_planner::searchRoute(const std::vector<float> *start, const std::vector<float> *target)
{
    goal.first = (*target)[0];
    goal.second = (*target)[1];
    waypoint* cur_ptr;
    cur_ptr = new waypoint;
    cur_ptr->point.first = (*start)[0];
    cur_ptr->point.second = (*start)[1];
    cur_ptr->parent = nullptr;
    route_tree.push_back(cur_ptr);

    int count=0;

    while(1)
    {
        std::pair<float, float> randomPoint = getRandomPoint();
        cur_ptr = nearestPoint(randomPoint);     //从生成的随机点中获得路径树中离其最近一点
        addPoint(cur_ptr, randomPoint);                            //沿随机点进行生长

        if(fabs(cur_ptr->point.first - goal.first)<0.05 && fabs(cur_ptr->point.second - goal.second)<0.05)
        {
            ROS_INFO("Path found! Search times: %d", count);
            break;
        }

        count++;
    }
    ROS_ERROR("Search fial, please try angain");
}

RRT_planner::waypoint* RRT_planner::nearestPoint(std::pair<float, float> target)
{
    float min_distance = 65536;
    waypoint* nearest_point = new waypoint;
    for(auto it:route_tree)
    {
        if(min_distance > fabs(it->point.first - target.first)+fabs(it->point.second - target.second))
        {
            min_distance = fabs(it->point.first - target.first)+fabs(it->point.second - target.second);
            nearest_point = it;
        }
    }
    return nearest_point;
}

std::pair<float, float> RRT_planner::getRandomPoint()
{
    std::pair<float, float> rdpoint;
    // int random = ;
    if(rd()%9>4)
    {
        rdpoint.first = goal.first + (rd()%41 - 20)*0.1;
        rdpoint.second = goal.second + (rd()%41 - 20)*0.1;
    }
    else
    {
        rdpoint.first = rd()%hig;
        rdpoint.second = rd()%wid;
    }
    return rdpoint;
}

void RRT_planner::addPoint(waypoint* nearest, const std::pair<float, float> &target)
{
    //先求出两点间斜率
    double gradiant = hypot(nearest->point.first-target.first,nearest->point.second-target.second);
    float grad_x,grad_y; 
    if(gradiant>0)
    {
        grad_x = 1/gradiant * (nearest->point.first-target.first);
        grad_y = 1/gradiant * (nearest->point.second-target.second);
    }
    else return;

    //生成新点
    waypoint* newPoint = new waypoint;
    newPoint->parent = nearest;
    newPoint->point.first = nearest->point.first + grad_x * step;
    newPoint->point.second = nearest->point.second + grad_x * step;

    //检测障碍, 合规则push，否则del
    if(CheckObstacle(newPoint->point.first, newPoint->point.second))
    {
        delete newPoint;
    }
    else
    {
        route_tree.push_back(newPoint);
    }
}
