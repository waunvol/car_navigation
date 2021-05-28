#include "motionCTRL.h"

MotionControl::MotionControl()
{
    cmdPub = n.advertise<geometry_msgs::Twist>("cmd_vel" ,3);
    posSub = n.subscribe("car_position", 1, &MotionControl::updatePosition, this);
    T_PID = n.createTimer(ros::Duration(0.03), &MotionControl::PIDcontrol, this, false, false);      //PID计算间隔为0.1s
    T_step = n.createTimer(ros::Duration(0.006), &MotionControl::STEPcontrol, this, false, false);   //controller输出间隔为0.02s
    piddebug = n.advertise<std_msgs::Float32>("error" ,1);
}

void MotionControl::MoveToStart()
{
    double dev_angle = 1;
    dev_angle = (fabs(route[index_route].pos-Y)<fabs(route[index_route].pos+Y))?(route[index_route].pos-Y):(route[index_route].pos+Y);
    // if(dev_angle>0)
    //     a_w = 4.5*dev_angle;
    // else
    //     a_w = -4.5*dev_angle;
    w_I += dev_angle;
    a_w = -(P_w*dev_angle*10 + I_w*w_I);
    
    msg.data = dev_angle;
    
    // if(fabs(dev_angle) <0.05)
    // {    
    //     index_route++;
    //     // cmd_msgs.angular.z = 0;
    //     }
}

void MotionControl::PIDcontrol(const ros::TimerEvent&)
{
    if(index_route==0)
        MoveToStart();
    else
    {
        double angle = atan2(route[index_route].y - y, route[index_route].x - x);  //期望往目标点移动的方向
        double ds = hypot(route[index_route].x-x,route[index_route].y-y);     //距离的差值

        double v_now;
        for(auto it:v_queue)
            v_now += it;

        v_now /= 3;
        v_I += route[index_route].v-v_now;
        // v_I += 0.1-v_now;
        a_v = P_v2*(route[index_route].v-a_v) + I_v2*v_I;   //速度环
        // a_v = P_v2*(0.1-v_now) + I_v2*v_I;   //速度环
        // ROS_INFO("a_v:%f", a_v);
        a_v*=-0.2;

        double dev_angle = (fabs(angle-Y)<fabs(angle+Y))?(angle-Y):(angle+Y);
        // ROS_INFO("index_route:%d", index_route); 
        

        w_I += dev_angle;
        a_w = P_w*dev_angle + I_w*w_I +D_w*(0 - w_now);               //位置环

        // ROS_INFO("dev_angle%f", dev_angle);
        // ROS_INFO("a_w%f", a_w);
        // ROS_INFO("a_w%f", a_w);

        // std_msgs::Float32 msg;
        // msg.data = dev_angle;
        

        if(ds<0.05)
        {
            if(index_route<(route.size()-1))
            {    
                index_route++;
                w_I = 0;
                v_I = 0;
                t=0;
            }
            else if(index_route>=(route.size()-1))
                stop();

        }
        // piddebug.publish(msg);
    }
    piddebug.publish(msg);
}

void MotionControl::STEPcontrol(const ros::TimerEvent&)
{
    // if(fabs(a_w)>=W_max)
    //     cmd_msgs.angular.z = W_max*(a_w/fabs(a_w));
    // else
        cmd_msgs.angular.z = -a_w;
//     if(index_route!=0)

// cmd_msgs.linear.x +=0;
    // cmd_msgs.linear.x += a_v - cmd_msgs.linear.x*fabs(cmd_msgs.angular.z)*0.74;
    // // ROS_INFO("a_v:%f", a_v);
    // if(cmd_msgs.linear.x>V_max)
    //     cmd_msgs.linear.x = V_max;
    // else if(cmd_msgs.linear.x<-V_max)
    //     cmd_msgs.linear.x = -V_max;
    
    cmdPub.publish(cmd_msgs);

}

void MotionControl::getPATH(vector<pair<float, float>> &path)
{
    //这里是给定期望小车的状态，包括速度，坐标，姿态
    //默认时间间隔为0.1s
    route.resize(path.size());
    int end = path.size()-1;
    //所有路径点都是从终点往原点延伸，所以push路径点时要注意次序

    //起点
    route[0].x = path[end].first;
    route[0].y = path[end].second;
    route[0].w = 0;     //角速度
    route[0].v = 0;     //线速度
    route[0].pos = atan2(path[end-1].second - path[end].second, path[end-1].first - path[end].first);   //姿态(即yaw角)

    //终点
    route[end].x = path[0].first;
    route[end].y = path[0].second;
    route[end].w = 0;     //角速度
    route[end].v = 0;     //线速度
    route[end].pos = atan2(path[0].second - path[1].second, path[0].first - path[1].first);   //姿态(即yaw角)
    
    int i=1;        //i记为
    //从首和尾分别开始赋值(因为起点与终点的速度均要为0)
    float s=0;
    for(i=1;i<end;i++)  //从开始点到最大速度到结束
    {
        route[i].x = path[end-i].first;
        route[i].y = path[end-i].second;
        route[i].pos = atan2(route[i].y - route[i-1].y, route[i].x - route[i-1].x);
        s += hypot(route[i-1].x - route[i].x, route[i-1].y - route[i].y);
        if(route[i-1].v<V_max)
            route[i].v = sqrt(2*av_max*s);
        else
            route[i].v = V_max;
    }
    s=0;
    for(i=end-1;i>=0;i--) //从结束点到最大速度的末点
    {
        s += hypot(route[i+1].x - route[i].x, route[i+1].y - route[i].y);
        route[i].v = sqrt(2*av_max*s);
        if(route[i].v>=V_max || i<=end/2)
            break;
    }

    ROS_INFO("Path confige done!");

}

