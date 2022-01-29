#include "smooth.h"



using namespace std;

pair<float,float> smooth::BezierSingalPointCalculate(int order, float t, queue<pair<float, float>> ctrl_pt)
{
    while(ctrl_pt.size()!=1)
    {
        int cnt =0;
        pair<float, float> tmp;
        while (order - cnt)
        {
            // get a control point from every two points
            pair<float,float> f_p = ctrl_pt.front();
            ctrl_pt.pop();
            pair<float,float> r_p = ctrl_pt.front();
            tmp = {f_p.first*(1-t) + r_p.first*t, 
                    f_p.second*(1-t) + r_p.second*t};
            ctrl_pt.push(tmp);  // push into queue for next level

            cnt++;
        }
        ctrl_pt.pop();  // clear last rear control point
        
        order--;
    }
    return ctrl_pt.front();
}


vector<pair<float, float>> smooth::PathSmooth(int order ,const vector<pair<float, float>> &path)
{
    vector<pair<float, float>> waypoint;

    float step = 1.0/order;
    for(size_t i=0;; )
    {
        queue<pair<float,float>> tmp;
        
        if( i>=path.size()-order) // for the rest path
        {
            order = path.size()-i;
            step = 1.0/order;
            for(size_t j=i; j<path.size();++j)
            {
                tmp.push(path[j]);
            }
            for(float t=0; t<1; t+=step)    // end point do not abort
            {
                waypoint.push_back(BezierSingalPointCalculate(order, t, tmp));
            }
            break;
        }

        
        for(size_t j=0; j<=order; ++j) 
        {
            tmp.push(path[i+j]);
        }
        i+=order;
        for(float t=0; t<1-step/2; t+=step) // abort the last point
        {
            waypoint.push_back(BezierSingalPointCalculate(order, t, tmp));
        }

    }

    ROS_INFO("Path-smoothing done!");
    return waypoint;
}

