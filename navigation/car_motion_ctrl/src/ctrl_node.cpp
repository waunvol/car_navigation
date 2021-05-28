#include "motionCTRL.h"
#include <nav_msgs/Path.h>

bool rec_flag = 0;

void path_pubRECV(const nav_msgs::Path::ConstPtr msg, vector<pair<float, float>> *path)
{
    path->clear();
    pair<float, float> tmp;
    for(auto it:(*msg).poses)
    {
        tmp.first = it.pose.position.x;
        tmp.second = it.pose.position.y;
        path->push_back(tmp);
    }
    rec_flag = 1;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ctrl_node");
    ros::NodeHandle n;

    string PathName = argv[1]; 

    vector<pair<float, float>> path;
    ros::Subscriber path_sub = n.subscribe<nav_msgs::Path>(PathName, 1, boost::bind(&path_pubRECV, _1, &path));

    MotionControl cmd;
    ros::Rate r(30);
    ros::AsyncSpinner spinner(4); // Use 4 threads

    ROS_INFO("Motion control ready!");

    spinner.start();
    while(ros::ok())
    {
        
        if(rec_flag)
        {
            ROS_INFO("Plan received!");
            cmd.stop();
            rec_flag=0;
            cmd.getPATH(path);
            cmd.start();
        }
        r.sleep();
    }
    return 0;
}