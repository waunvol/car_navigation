#include "Dijkstra.h"


dijkstra::dijkstra(nav_msgs::GetMap rec_map)
{
    map_msg = rec_map;
    ROS_INFO("Dijkstra now start initing....");
    resolution = map_msg.response.map.info.resolution;

    //获取默认起始位置（一般rviz机器人的起始位置都为0，0，0， 所以减去地图的原始偏移就是其在地图的初始位置）
    start_x = map_msg.response.map.info.origin.position.x * (-1) / resolution;
    start_y = map_msg.response.map.info.origin.position.y * (-1) / resolution;

    //创建空地图
    height = map_msg.response.map.info.height;
    width = map_msg.response.map.info.width;
    map = new graph**[height];
    for(int i=0; i<height; i++)
        map[i] = new graph*[width];

    //地图解析
    decodeMap();
    // cout <<"原点值为:" << map[276][20]->visit <<endl;

    ROS_INFO("Initing complete!");

}

dijkstra::~dijkstra()
{
}

void dijkstra::decodeMap()
{
    int i, j;
    int map_index = 0;
    ROS_INFO("Decoding map....");
    for (i = 0; i < height;i++)
    {
        //y=>height, x=>width
        for (j = 0; j < width; j++)
        {

            map[i][j] = new graph;
            map[i][j]->way = new waypoint;
            map[i][j]->way->x = j;
            map[i][j]->way->y = i;

            map[i][j]->visit = map_msg.response.map.data[map_index];    
            /*
            -If p > occupied_thresh, output the value 100 to indicate the cell is occupied.
            -If p < free_thresh, output the value 0 to indicate the cell is free.
            -Otherwise, output -1 a.k.a. 255 (as an unsigned char), to indicate that the cell is unknown.

            所以将data赋值到visit， 仅当visit为0时，该单元格为未搜索的有效单元格
            */
            // cout << map[i][j]->visit <<" "; 
            graph::neighbor **nebPTR;
            nebPTR = &(map[i][j]->itsNeighbor);
            //指针指向相邻
            if(i+1<height)
            {
                *nebPTR = new graph::neighbor;
                (*nebPTR)->data = &map[i + 1][j];
                nebPTR = &(*nebPTR)->next;
            }
            if(j+1<width)
            {
                *nebPTR = new graph::neighbor;
                (*nebPTR)->data = &map[i][j+1];
                nebPTR = &(*nebPTR)->next;
            }
            if(i-1>=0)
            {
                *nebPTR = new graph::neighbor;
                (*nebPTR)->data = &map[i - 1][j];
                nebPTR = &(*nebPTR)->next;
            }
            if(j-1>=0)
            {
                *nebPTR = new graph::neighbor;
                (*nebPTR)->data = &map[i][j-1];
                nebPTR = &(*nebPTR)->next;
            }

            map_index++;
        }
        // cout <<endl;
    }  

}

bool dijkstra::search()
{
    graph *now = map[start_y][start_x];     //y=>height, x=>width
    now->distance = 0;

    graph *tmp; //用于简化代码
    
    vector<graph **>::iterator i;
    vector<graph **>::iterator shortest;

    do  //搜索邻居最短点
    {
        int min = 65535;

        tmp = *(now->itsNeighbor->data);
        graph::neighbor *nebPTR;
        nebPTR = now->itsNeighbor;
        while(1)
        {
            if(tmp->visit==0)   //若点未搜索过
            {
                min = tmp->weight;      //记录当前最小
                if(tmp->distance > now->distance + tmp->weight)    
                {
                    tmp->distance = now->distance + tmp->weight;    //更新点中的最短距离
                    tmp->way->front = now->way;                     //更新路径
                    ShortPathTable.push_back(nebPTR->data);         //加入下一轮迭代
                }

            }
            nebPTR = nebPTR->next;
            if(nebPTR == nullptr)       //跳出条件
                break;
            tmp = *(nebPTR->data);
        }
        min = 65535;

        for (i = ShortPathTable.begin(); i != ShortPathTable.end();i++)     //从搜索队列中找出当前最近点
        {
            // cout << (*(*i))->distance << endl;
            if((*(*i))->distance < min)
            {
                min = (*(*i))->distance;
                shortest = i;
            }
        }
        (*(*shortest))->visit = 1;      //标记搜索标志
        now = (*(*shortest));
        ShortPathTable.erase(shortest);     //从搜索队列中删除

        if(ShortPathTable.empty())
            return false;
        // cout << ShortPathTable.size() << endl;

    } while (!( now->way->x==target_x && now->way->y==target_y));
    return true;
}
