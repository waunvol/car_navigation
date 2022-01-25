#include "smooth.h"



using namespace std;

pair<float,float> smooth::BezierSingalPointCalculate(int order, float t, queue<pair<float, float>> ctrl_pt)
{
    while(order>0)
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

    for(size_t i=0;; )
    {
        queue<pair<float,float>> tmp;
        for(size_t j=0; j<=order; j++) 
        {
            tmp.push(path[i+j]);
        }
        
        float step = 1.0/order;
        for(float t=0; t<1-step/2; t+=step) // abort the last point
        {
            waypoint.push_back(BezierSingalPointCalculate(order, t, tmp));
        }
        

        i+=order;
        if( i>=path.size()) // for the rest path
        {
            step = 1.0/(path.size()-i+order);
            for(float t=0; t<1; t+=step)    // end point do not abort
            {
                waypoint.push_back(BezierSingalPointCalculate(order, t, tmp));
            }
            break;
        }

    }

    ROS_INFO("Path-smoothing done!");
    return waypoint;
}

