#ifndef ASTAR_H_
#define ASTAR_H_

#include "cost.h"
#include <queue>
#include <algorithm>
#include "PathPlanning.h"

class Index {
    public:
        Index(int a, float b) {
            i = a;
            cost = b;
        }
        int i;
        float cost;
};

struct greater1 {
        bool operator()(const Index& a, const Index& b) const {
            return a.cost > b.cost;
        }
};


class astar:public PathPlanning
{
private:
    std::vector<Index> queue_;

    //曼哈顿距离
    float MHTdistance(int now, double end_x, double end_y) 
    {
        int now_x = now % wid;
        int now_y = now / wid;
        return fabs(end_x - now_x*0.05) + fabs(end_y - now_y*0.05);
    };

    void addQueue(int before ,int now, double endx, double endy, int* potential);

public:
    bool calculatePotential(float start_x, float start_y, vector<pair<float, float>> destination, int* potential);
};










#endif
