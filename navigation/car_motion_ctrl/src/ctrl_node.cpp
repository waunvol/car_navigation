#include "motion_controller.h"
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * to do:
 * 1. process global path, make less waypoints (incress internal of each point)
 * 2. rec_flag is not thread saft. Fix it.
 * 3. publish speed.
 */

std::atomic<bool> rec_flag;
Pose_t cur_pose;
std::mutex path_lock;

void path_pubRECV(const nav_msgs::Path::ConstPtr msg, std::vector<Pose_t> *path)
{
    std::lock_guard<std::mutex> lock(path_lock);

    rec_flag.store(true, std::memory_order_relaxed);
    Pose_t pt;
    for(int i=0; i<msg->poses.size()-1; ++i) {
        pt.x = msg->poses[i].pose.position.x;
        pt.y = msg->poses[i].pose.position.y;
        pt.yaw = std::atan2((pt.y - msg->poses[i+1].pose.position.y),
                    (pt.x - msg->poses[i+1].pose.position.x));
        path->push_back(pt);
    }

    pt.x = msg->poses.back().pose.position.x;
    pt.y = msg->poses.back().pose.position.y;
    pt.yaw = cur_pose.yaw;
    path->push_back(pt);
}

void UpdatePose(const geometry_msgs::Pose msg) {
    tf::Pose pose;
    tf::poseMsgToTF(msg, pose);
    cur_pose.x = msg.position.x;
    cur_pose.y = msg.position.y;
    cur_pose.yaw = tf::getYaw(pose.getRotation());
    ROS_INFO("current pose:x %lf, y %lf, yaw %lf", cur_pose.x, cur_pose.y, cur_pose.yaw);
}

// global path need process

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ctrl_node");
    ros::NodeHandle n;

    rec_flag = 0;
    // string PathName = argv[1]; 
    string PathName = "Astar_path"; 
    int frequency = 20;

    std::vector<Pose_t> global_path;
    ros::Subscriber path_sub = n.subscribe<nav_msgs::Path>(PathName, 1, boost::bind(&path_pubRECV, _1, &global_path));
    ros::Subscriber pos_sub = n.subscribe("car_position", 1, UpdatePose);

    ros::Publisher speed_pub = n.advertise<geometry_msgs::Twist>("cmd", 1);

    ros::Rate r(frequency);
    ros::AsyncSpinner spinner(1); // Use another to subscribe position

    ROS_INFO("Motion control ready!");
    double tolerance = 0.05;

    MotionController controller;
    controller.InitController(0.05, 0.05, frequency);
    controller.setPID(0.03, 0.0, 0);
    std::pair<double, double> cur_speed = {0.0, 0.0};

    spinner.start();
    while(ros::ok())
    {

        if(rec_flag.load(std::memory_order_relaxed))
        {
            geometry_msgs::Twist speed;
            rec_flag.store(false, std::memory_order_relaxed);

            path_lock.lock();
            controller.setGlobalPath(global_path);
            path_lock.unlock();

            while(!rec_flag.load(std::memory_order_relaxed)) // interrupt when get new goal;
            {   
                if (sqrt(pow(cur_pose.x - global_path.front().x, 2) +
                        pow(cur_pose.y - global_path.front().y, 2)) < tolerance)
                {
                    speed.linear.x = 0;
                    speed.angular.z = 0;
                    ROS_INFO("Navigation finish!");
                    break;
                }
                cur_speed = controller.CalculateValue(cur_pose, cur_speed.first, cur_speed.second);
                speed.linear.x = cur_speed.first;
                speed.angular.z = cur_speed.second;
                speed_pub.publish(speed);

                r.sleep();
            }
        }
        r.sleep();
    }
    ros::waitForShutdown();
    return 0;
}