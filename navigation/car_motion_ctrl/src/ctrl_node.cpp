#include "motion_controller.h"
#include <nav_msgs/Path.h>
#include "dwa_planner.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

bool rec_flag = 0;
Pose_t cur_pose;

void path_pubRECV(const nav_msgs::Path::ConstPtr msg, std::vector<Pose_t> *path)
{
    rec_flag = true;
    Pose_t pt;
    for(int i=0; i<msg->poses.size()-1; ++i) {
        pt.x = msg->poses[i].pose.position.x;
        pt.y = msg->poses[i].pose.position.y;
        pt.yaw = atan((pt.y - msg->poses[i+1].pose.position.y)/
                    (pt.x - msg->poses[i+1].pose.position.x));
        path->push_back(pt);
    }

    pt.x = msg->poses.back().pose.position.x;
    pt.y = msg->poses.back().pose.position.y;
    pt.yaw = cur_pose.yaw;
    path->push_back(pt);
}

void UpdatePose(const geometry_msgs::Pose msg) {
    tf2::Quaternion quat_tf;
    tf2::fromMsg(msg.orientation, quat_tf);
    cur_pose.x = msg.position.x;
    cur_pose.y = msg.position.y;
    cur_pose.yaw = quat_tf.getAngleShortestPath();
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ctrl_node");
    ros::NodeHandle n;

    string PathName = argv[1]; 

    std::vector<Pose_t> global_path;
    ros::Subscriber path_sub = n.subscribe<nav_msgs::Path>(PathName, 1, boost::bind(&path_pubRECV, _1, &global_path));
    ros::Subscriber pos_sub = n.subscribe("car_position", 1, UpdatePose);

    ros::Publisher speed_pub = n.advertise<geometry_msgs::Twist>("cmd", 1);

    ros::Rate r(20);
    ros::AsyncSpinner spinner(1); // Use another to subscribe position

    ROS_INFO("Motion control ready!");

    spinner.start();
    while(ros::ok())
    {
        MotionController controller;
        if(rec_flag == true)
        {
            geometry_msgs::Twist speed;
            rec_flag = false;
            while(rec_flag == false) // interrupt when get new goal;
            {
                r.sleep();
            }
        }

    }
    ros::waitForShutdown();
    return 0;
}