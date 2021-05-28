#ifndef DIJKSTRA_H_
#define DIJKSTRA_H

#include "cost.h"

#define pushCur(n)             \
    {                          \
        curBUFF[++curPTR] = n; \
        pending[n] = true;     \
    }
#define pushNext(n)             \
    {                          \
        nextBUFF[++nextPTR] = n; \
        pending[n] = true;     \
    }

class dijkstra:public PathPlanning
{
private:
    bool complete(vector<int> goal, int* potential);//判断是否已完成所有点搜索

public:
    void calculatePotential(float start_x, float start_y, vector<pair<float, float>> destination, int* potential);



};


#endif