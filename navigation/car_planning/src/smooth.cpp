#include "smooth.h"



using namespace std;


//逐层递归，一层一阶
void smooth::BezierCalculate_(int level, float t)//level:阶数，t：贝塞尔曲线公式中的原版t
{
    if(_queue.size()==1)    //结束条件
        return;

    pair<float, float> tmp;
    int count=0;
    while (level-count)
    {
        pair<float, float> f_p = _queue.front();  //提取控制点
        _queue.pop();       //移出队列
        pair<float, float> r_p = _queue.front();

        tmp.first = (f_p.first*(1-t) + r_p.first*t);    //计算次阶控制点
        tmp.second = (f_p.second*(1-t) + r_p.second*t);
        _queue.push(tmp);   //加入队列

        count++;
    }
    _queue.pop();   //处理玩当前层时，需要将所有此层元素清空
    BezierCalculate_(level-1, t);

    return;
}

//解递归
void smooth::BezierCalculate(int level, float t)
{
    while(_queue.size()!=1)
    {
        pair<float, float> tmp;
        int count=0;
        while (level-count)
        {
            pair<float, float> f_p = _queue.front();  //提取控制点
            _queue.pop();       //移出队列
            pair<float, float> r_p = _queue.front();

            tmp.first = (f_p.first*(1-t) + r_p.first*t);    //计算次阶控制点
            tmp.second = (f_p.second*(1-t) + r_p.second*t);
            _queue.push(tmp);   //加入队列

            count++;
        }
        _queue.pop();   //处理玩当前层时，需要将所有此层元素清空
        level--;
    }
}

vector<pair<float, float>> smooth::PathSmooth(const vector<pair<float, float>> &path)
{
    vector<pair<float, float>> waypoint;

    int count = path.size()/10 + 1;     //路径的控制点数量
    float step = 1.0/count;             //步长

    for(float t=0; t<=1; t+=step)
    {
        for(auto it:path)
        {
            _queue.push(it);
        }
        BezierCalculate(path.size()-1, t);
        waypoint.push_back(_queue.front());
        _queue.pop();
    }
    
    ROS_INFO("Path-smoothing done!");
    return waypoint;
}

