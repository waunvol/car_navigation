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

    ros::Timer T_PID, T_step;
    void PIDcontrol(const ros::TimerEvent&);
    void STEPcontrol(const ros::TimerEvent&);


    void updatePosition(const geometry_msgs::Pose::ConstPtr msg)
    {
        // rec_msgs = *msg;
        v_queue.push_back(hypot(x-(*msg).position.x, y-(*msg).position.y)*100);
        if(v_queue.size()>5)
            v_queue.erase(v_queue.begin());
        // ROS_INFO("size:%d",v_queue.size());
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
    void clearParam()
    {
        a=0, w=0;
        v_I=0, w_I=0;
        cmd_msgs.linear.x = 0;
        cmd_msgs.angular.z =0;
        route.clear();
        index_route=0;
    }
    void MoveToStart();

public:
    float P_v=0.92,I_v=0.00005,D_v=0;  //线速度pid参数
    float P_w=4.5, I_w=0.0003, D_w=0.000;  //角速度pid参数
    double x=0,y=0,R,P,Y;       //当前的x，y，当前的yaw角
    double w_now,_Y;      //当前的线速度，角速度，上一次的yaw
    double V_max = 0.26;     //线最大角速度
    double W_max = 4.368;     //绕z轴的最大角速度
    double av_max=0.001;     //线速度向最大加速度
    double w,a; //w:输出角速度， a：输出线加速度
    double v_I=0, w_I=0;      //上一轮的线速度和角速度
    int t=1;

    struct waypoint //创建一个结构体记录点与速度
    {
        float x,y,v,pos;  //x坐标，y坐标，角速度，线速度，姿态
    };
    vector<waypoint> route;
    vector<double> v_queue;
    int index_route;

    MotionControl();
    ~MotionControl()
    {};

    void setPID_v(float p, float i, float d)
    {P_v=p; I_v=i; D_v=d;}
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
        T_PID.start();
        T_step.start();
        ROS_INFO("Action!");
    }
    void stop()
    {
        clearParam();   //clear all param before exit
        cmdPub.publish(cmd_msgs);
        T_PID.stop();
        T_step.stop();


  
    }
    void getPATH(vector<pair<float, float>> &path); //获取路径，作预处理

};















#endif