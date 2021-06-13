#include "cost.h"



void costmap::init()
{
    ros::NodeHandle n;
    nav_msgs::GetMap mapRev;
    ros::ServiceClient Mapsrv = n.serviceClient<nav_msgs::GetMap>(topicName);

    if(Mapsrv.call(mapRev))
    {
        ROS_INFO("Mapserver has been connectted!");
    }
    else
    {
        ROS_INFO("May something worried");
    }

    resolution = mapRev.response.map.info.resolution;
    //获得起始点
    original_x = mapRev.response.map.info.origin.position.x*(-1);
    original_y = mapRev.response.map.info.origin.position.y*(-1);   
    // ROS_INFO("%f, %f", mapRev.response.map.info.origin.position.x, mapRev.response.map.info.origin.position.y); 
    //获得初始的COST值 
    // *COST = mapRev.response.map.data;   
    //默认模式下，free的cost值为0，未知为-1，不可到达为100    
    for (auto it:mapRev.response.map.data)
    {
        if(it != 0)
            COST.push_back(it);
        else
            COST.push_back(1);
    }                                                              

    //获得地图其他参数
    wid = mapRev.response.map.info.width;
    hig = mapRev.response.map.info.height;
}

#define left_obstacle   1
#define right_obstacle  2
#define up_obstacle     3
#define down_obstacle   4

//向除了有障碍物方向扩散

void costmap::rebound(int index,int operation, int threshold)
{
    if (threshold <= 50)
        return;

    //车体外径(interferenceArea)影响步长，上界固定为50
    float step = 50/(InterferenceArea/(0.05*Multiple));
    if (operation != left_obstacle && COST[index - 1] <= threshold)
    {
        COST[index - 1] = threshold;
        rebound(index - 1, operation, threshold-step);
    }
    if (operation != right_obstacle && COST[index + 1] <= threshold)
    {
        COST[index + 1] = threshold;
        rebound(index + 1, operation, threshold-step);
    }
    if (operation != up_obstacle && COST[index - wid] <= threshold)
    {
        COST[index - wid] = threshold;
        rebound(index-wid, operation, threshold-step);
    }
    if (operation != down_obstacle && COST[index + wid] <= threshold)
    {
        COST[index + wid] = threshold;
        rebound(index+wid, operation, threshold-step);
    }
    return;
}


void costmap::calculateCOST()
{
    origin = GetIndex(original_x, original_y);
    int *curBUFF = new int[wid*hig*4];
    int *nextBUFF = new int[wid*hig*4];
    int *swapPTR;
    bool pending[wid * hig];
    memset(pending, 0, sizeof(bool)*wid*hig);

    int curPTR=0, nextPTR=0;

    //curbuff从1开始存放数据，0为空数
    pending[origin] = true;
    curBUFF[++curPTR] = origin+1;
    curBUFF[++curPTR] = origin-1;
    curBUFF[++curPTR] = origin+wid;
    curBUFF[++curPTR] = origin-wid;
    // ROS_INFO("cost %d", COST.size());
    while (curPTR!=0)
    {
        //先遍历curbuff，在过程中将新加入的加入nextbuff
        //判断curbuff中的格子周围4个，符合条件则加入nextbuff（不为-1，不超界）
        int i = curPTR;

        while (i && !pending[curBUFF[i]])
        {
            // ROS_INFO("%d", i);
            // 未检索 && 不超界 
            if(!pending[curBUFF[i]-1] && (curBUFF[i]-1)%wid<curBUFF[i]%wid)
            {
                // ROS_INFO("a %d", (curBUFF[i]-1));
                if (COST.at(curBUFF[i]-1)==100)
                {
                    rebound(curBUFF[i], left_obstacle, 99); //左侧为障碍，反弹至右侧
                }
                else nextBUFF[++nextPTR]=curBUFF[i]-1;
            }
            if(!pending[curBUFF[i]+1] && (curBUFF[i]+1)%wid>curBUFF[i]%wid)
            {
                // ROS_INFO("b %d", i);
                if (COST.at(curBUFF[i]+1)==100)
                {
                    rebound(curBUFF[i], right_obstacle, 99); //右侧为障碍，反弹至左侧
                }   
                else nextBUFF[++nextPTR]=curBUFF[i]+1;
            }
            if(!pending[curBUFF[i]-wid] && curBUFF[i]-wid>=0)
            {
                // ROS_INFO("c %d", i);
                if (COST.at(curBUFF[i]-wid)==100)
                {
                    rebound(curBUFF[i], up_obstacle, 99); //上侧为障碍，反弹至下侧
                }
                else nextBUFF[++nextPTR]=curBUFF[i]-wid;
            }
            if(!pending[curBUFF[i]+wid] && curBUFF[i]+wid<=wid*hig)
            {
                // ROS_INFO("d %d", i);
                if (COST.at(curBUFF[i]+wid)==100)
                {
                    rebound(curBUFF[i], down_obstacle, 99); //下侧为障碍，反弹至上侧
                }
                else nextBUFF[++nextPTR]=curBUFF[i]+wid;
            }
            pending[curBUFF[i]] = true;
            i--;
        }     

        //完成遍历后，将curbuff和nextbuff交换(指针也交换)
        swapPTR = curBUFF;
        curBUFF = nextBUFF;
        nextBUFF = swapPTR;
        curPTR = nextPTR;
        nextPTR = 0;
    }
    ROS_INFO("COST map done!");
}

void costmap::testFun()    //测试读数
{
    cout<<"width:"<<wid<<endl;
    cout<<"hight:"<<hig<<endl;
}


