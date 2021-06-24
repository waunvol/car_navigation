#include "RRT.h"

RRT_planner::RRT_planner(/* args */)
{
}

RRT_planner::~RRT_planner()
{
    free();
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
        addPoint(&cur_ptr, randomPoint);                            //沿随机点进行生长

        if(fabs(cur_ptr->point.first - goal.first)<0.1 && fabs(cur_ptr->point.second - goal.second)<0.1)
        {
            ROS_INFO("Path found! Search times: %d", count);
            return true;
        }

        count++;
    }
    ROS_ERROR("Search fial, please try angain");
    return false;
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
    int tmp1;
    float tmp2 = rd()%10*0.1;
    if(tmp2>0.55)
    {
        tmp1 =rd()%21 - 10;
        tmp2 = tmp1 * 0.1;
        rdpoint.first = goal.first + tmp2;
        tmp1 = rd()%21 - 10;
        tmp2 = tmp1 * 0.1;
        rdpoint.second = goal.second + tmp2;
    }
    else
    {
        tmp2 = (rd()%(hig-1))*0.05;
        rdpoint.first = tmp2;
        tmp2 = (rd()%(wid-1))*0.05;
        rdpoint.second = tmp2;
    }
    return rdpoint;
}

std::vector<std::vector<float>> RRT_planner::getRoute()
{

    std::vector<std::vector<float>> route;
    waypoint *ptr;
    ptr = route_tree[route_tree.size()-1];

    std::vector<float> temp = {0.0, 0.0};
    while(ptr->parent!=nullptr)
    {
        temp[0] = ptr->point.first;
        temp[1] = ptr->point.second;
        route.insert(route.begin(), temp);
        ptr = ptr->parent;
    }
    free();

    return route;
}

void RRT_planner::addPoint(waypoint** nearest, const std::pair<float, float> &target)
{
    //先求出两点间斜率
    double gradiant = hypot(target.first-(*nearest)->point.first,target.second-(*nearest)->point.second);
    float grad_x,grad_y; 
    if(gradiant>0)
    {
        grad_x = 1/gradiant * (target.first-(*nearest)->point.first);
        grad_y = 1/gradiant * (target.second-(*nearest)->point.second);
    }
    else return;

    //生成新点
    waypoint* newPoint = new waypoint;
    newPoint->parent = *nearest;
    newPoint->point.first = (*nearest)->point.first + grad_x * step;
    newPoint->point.second = (*nearest)->point.second + grad_y * step;

    //检测障碍, 合规则push，否则del
    if(CheckObstacle(newPoint->point.first, newPoint->point.second))
    {
        delete newPoint;
    }
    else
    {
        *nearest = newPoint;
        route_tree.push_back(newPoint);
    }
}

void RRT_planner::free()
{
    for(auto it:route_tree)
    {
        delete it;
    }
    route_tree.clear();
}