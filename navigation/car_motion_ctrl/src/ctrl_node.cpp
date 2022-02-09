#include "motionCTRL.h"
#include <nav_msgs/Path.h>
#include "dwa_planner.h"

bool rec_flag = 0;
nav_msgs::Path g_path;
geometry_msgs::Pose cur_pose;
geometry_msgs::Twist sped;

void path_pubRECV(const nav_msgs::Path::ConstPtr msg, vector<pair<float, float>> *path)
{
    // path->clear();
    // pair<float, float> tmp;
    // for(auto it:(*msg).poses)
    // {
    //     tmp.first = it.pose.position.x;
    //     tmp.second = it.pose.position.y;
    //     path->push_back(tmp);
    // }
    rec_flag = 1;
    g_path = *msg;
}

void UpdatePose(const geometry_msgs::Pose msg) {
    cur_pose = msg;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ctrl_node");
    ros::NodeHandle n;

    string PathName = argv[1]; 

    vector<pair<float, float>> path;
    ros::Subscriber path_sub = n.subscribe<nav_msgs::Path>(PathName, 1, boost::bind(&path_pubRECV, _1, &path));
    ros::Subscriber pos_sub = n.subscribe("car_position",1,UpdatePose);
    ros::Publisher tra_pub = n.advertise<nav_msgs::Path>("dwa_trajectory", 5);

    MotionControl cmd;
    ros::Rate r(30);
    ros::AsyncSpinner spinner(4); // Use 4 threads

    ROS_INFO("Motion control ready!");

    DWA_planner a;
    spinner.start();

    while(ros::ok())
    {
        
        if(rec_flag)
        {
            ROS_INFO("get nav!");
            if(!a.CalculateSpeed(g_path, cur_pose, sped)){
                ROS_ERROR("Dwa planning failed!");
            }
            tra_pub.publish(a.GetTrajectory());
            // ROS_INFO("Plan received!");
            // cmd.stop();
            // rec_flag=0;
            // cmd.getPATH(path);
            // cmd.start();
            rec_flag=0;
            sped.angular.z=0;
            sped.linear.x = 0;
        }
        r.sleep();
    }
    return 0;
}