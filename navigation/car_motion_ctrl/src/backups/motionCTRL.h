#ifndef MOTION_CTRL_H
#define MOTION_CTRL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include "tf/transform_datatypes.h"
#include <std_msgs/Float32.h>

using namespace std;


//实现小车控制，接受位置，作PID控制（沿规划路线行进）
class MotionControl
{
private:
    ros::NodeHandle n;
    ros::Publisher cmdPub;      //发布控制指令
    ros::Subscriber posSub;     //更新位置信息
    ros::Publisher piddebug;
    geometry_msgs::Twist cmd_msgs;  
    std_msgs::Float32 msg;
    
    // geometry_msgs::Pose rec_msgs;

    ros::Timer T_PID, T_step;
    void PIDcontrol(const ros::TimerEvent&);
    void STEPcontrol(const ros::TimerEvent&);


    

    void updatePosition(const geometry_msgs::Pose::ConstPtr msg)
    {
        // rec_msgs = *msg;
        v_queue.push_back(hypot(x-(*msg).position.x, y-(*msg).position.y)*100);
        if(v_queue.size()>3)
            v_queue.erase(v_queue.begin());
        // ROS_INFO("vnow:%f",v_now);
        x = (*msg).position.x;
        y = (*msg).position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF((*msg).orientation, quat);
        tf::Matrix3x3(quat).getRPY(R,P,Y);



        w_now = min(fabs(_Y - Y), fabs(_Y + Y))*10;     //获取转角
        if(_Y*Y<0)  //不同号时
        {
            if(_Y<0)    //前一个角小于0
                w_now*=-1;
        }
        else if(_Y<Y)
            w_now *=-1;

        _Y = Y;
    }

    void MoveToStart();

public:
    float P_v1=0.00001, D_v1=0.0, P_v2=0.45,I_v2=0.00005;  //pid参数0.14,,,,,2:0.5
    float P_w=3.0, I_w=0.001, D_w=0.000;  //pid参数5.6 00.00002 01.0
    double x=0,y=0,R,P,Y;       //当前的x，y，当前的yaw角
    double w_now,_Y;      //当前的线速度，角速度，上一次的yaw
    // pair<float, float> pos
    double V_max = 0.36;     //线最大角速度
    double W_max = 3.14;     //绕z轴的最大角速度
    double av_max=0.03;     //x向最大加速度
    double _w=0;     //上一阶段的角速度
    double a_w,a_v; //当前加速度
    double v_I=0, w_I=0;      //上一轮的线速度和角速度
    int t=0;
    int w_or_v=0;

    struct waypoint //创建一个结构体记录点与速度
    {
        float x,y,w,v,pos;  //x坐标，y坐标，角速度，线速度，姿态
    };
    vector<waypoint> route;
    vector<waypoint>::iterator it;
    vector<double> v_queue;
    int index_route;

    MotionControl();
    ~MotionControl()
    {};

    void setPID_v(float p, float i, float d)
    {P_v1=p; I_v2=i; D_v1=d;}
    void setPID_w(float p, float i, float d)
    {P_w=p; I_w=i; D_w=d;}

    void start()
    {
        if(route.size()==0)
        {
            ROS_ERROR("Error!Try again.");
            return;
        }
        index_route=0;      
        // MoveToStart();  //先移动至起始点，然后等待就绪
        // index_route++;
        T_PID.start();
        T_step.start();
        ROS_INFO("Action!");
    }
    void stop()
    {
        T_PID.stop();
        T_step.stop();
        cmd_msgs.linear.x = 0;
        cmd_msgs.angular.z =0;
        ROS_INFO("Mission complete!");
    }
    void getPATH(vector<pair<float, float>> &path); //获取路径，作预处理

};















#endif