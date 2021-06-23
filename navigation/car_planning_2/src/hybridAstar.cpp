#include "hybridAstar.h"

hybridAstar::hybridAstar(/* args */)
{
}

hybridAstar::~hybridAstar()
{
    if(route_tree.size()>0)
        free();
}

inline int hybridAstar::GetIndex(float x, float y)        
{
    int tmp;
    tmp = (x+0.025)/0.05;
    x = tmp;
    tmp = (y+0.025)/0.05;
    y = tmp;
    return x+y*wid;
    //若不用tmp作中间变量，返回的点可能不正确
} 

inline bool hybridAstar::CheckObstacle(float x, float y)
{
    int index = GetIndex(x, y);
    if((*COST)[index]>=60|(*COST)[index]<0)
        return true;
    else
        return false;
}

inline bool hybridAstar::CheckSet(int index)
{

    return false;
}

bool hybridAstar::calculateRoute(const vector<float> *start, const vector<float> *goal)
{
    Pending.resize(wid * hig, nullptr);

    cur_ptr = new waypoint;
    cur_ptr->parent = nullptr;
    cur_ptr->point.resize(5);
    cur_ptr->point = {(*start)[0], (*start)[1], (*start)[2], .0, .0}; //x,y,theta,journey,cost

    int index_cur = GetIndex((*start)[0], (*start)[1]);
    openSET.push_back(cur_ptr);
    Pending[index_cur] = cur_ptr;

    while(!openSET.empty())
    {
        cur_ptr = openSET[0];
        pop_heap(openSET.begin(), openSET.end(), greater1());
        openSET.pop_back();

        if(GetReedsShepp(goal))//若不能生成无障碍的RS曲线，则继续循环
        {
            ROS_INFO("Path found!");
            return true;
            }
        // if(fabs(cur_ptr->point[0]-(*goal)[0])<0.1 && fabs(cur_ptr->point[1]-(*goal)[1])<0.1)
        // {
        //     ROS_INFO("Path found!");
        //     return true;
        //     }

        //将六个方向加入队列
        //同一个父节点产生的子节点会全部加入openSET，并在pending中加入cost最小的节点（如果有重复归属的话）
        add_set(1, 0, *goal);
        add_set(1, 1/rho, *goal);
        add_set(1, -1/rho, *goal);
        add_set(-1, 0, *goal);
        add_set(-1, 1/rho, *goal);
        add_set(-1, -1/rho, *goal);
    }
    ROS_INFO("Search failed");
    free();
    openSET.clear();
    Pending.clear();
    return false;
}


void hybridAstar::add_set(double acceleration, double radius, const vector<float> &goal)
{
    waypoint *ptr = new waypoint;

    ptr->parent = cur_ptr;          //指向父母
    ptr->point.resize(5);

    ptr->point[3] = cur_ptr->point[3] + fabs(0.072*acceleration);
    ptr->point[2] = cur_ptr->point[2] + 0.072*radius*acceleration;
    ptr->point[0] = cur_ptr->point[0] + 0.072 * cos(ptr->point[2])*acceleration;
    ptr->point[1] = cur_ptr->point[1] + 0.072 * sin(ptr->point[2])*acceleration;

    //0.05/0.07=0.071取0.072，原0.1

    int index_cur = GetIndex(ptr->point[0], ptr->point[1]);

    if(CheckObstacle(ptr->point[0], ptr->point[1]))
    {
        delete ptr;
        return;
        }

    //代价 = 已行进路程 + 运动方向的增益/减益 + 距离目标的距离
    ptr->point[4] = ptr->point[3] + acceleration*-2.0 + 1.00*sqrt(pow(ptr->point[0]-goal[0], 2) + pow(ptr->point[1]-goal[1], 2));

    if(Pending[index_cur] && Pending[index_cur]->point[4] < ptr->point[4])  //若已经存有指针且其中cost值要更小，则跳出
    {  
        if(ptr->parent!=Pending[index_cur]->parent) //当已存在指针且其不为同一父节点时，才舍弃当前
        {
            delete ptr;  
            return;    
        }
    }
    else
        Pending[index_cur] = cur_ptr;       //无指针或者更小的cost值，更新pending集合

    route_tree.push_back(ptr);
    openSET.push_back(ptr);
    push_heap(openSET.begin(), openSET.end(), greater1());
}

bool hybridAstar::GetReedsShepp(const vector<float> *goal)
{
    ReedsSheppStateSpace curve;//以当前点到终点生成一条RS曲线
    double now[3] = {cur_ptr->point[0], cur_ptr->point[1], cur_ptr->point[2]};
    double goal_[3] = {(*goal)[0], (*goal)[1], (*goal)[2]};
    vector<vector<float>> rs_route = curve.GetCurve(now, goal_, 0.1);

    for(auto it:rs_route)
    {
        if(CheckObstacle(it[0], it[1]))
            return false;
    }

    for(auto it:rs_route)
    {
        waypoint *ptr = new waypoint;
        ptr->point.resize(3);
        ptr->point = {it[0], it[1], it[2]};
        ptr->parent = cur_ptr;
        cur_ptr = ptr;
    }

    return true;
}

// void hybridAstar::free(waypoint *deletePTR)
// {
//     if(deletePTR == nullptr)
//     {    
//         root = nullptr;
//         return;
//     }
//     if(deletePTR->members.size()==1)
//     {
//         delete deletePTR;
//         return;
//     }
//     for(int i=1; i<deletePTR->members.size(); i++)
//     {
//         free(deletePTR->members[i]);
//     }
// }
void hybridAstar::free()
{
    for(auto it:route_tree)
    {
        delete it;
    }
    route_tree.clear();
}


vector<vector<float>> hybridAstar::getRoute()
{
    vector<vector<float>> route;
    waypoint *ptr;
    ptr = cur_ptr;
    while(ptr->parent!=nullptr)
    {
        vector<float> temp = ptr->point;
        route.insert(route.begin(), temp);
        ptr = ptr->parent;
    }
    free();
    openSET.clear();
    Pending.clear();

    return route;
}