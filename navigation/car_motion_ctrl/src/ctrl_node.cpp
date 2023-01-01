#include "motion_controller.h"
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "local_planner/simple_dwa_planner.h"

/**
 * to do:
 * a more generic controller base
 */

std::atomic<bool> rec_flag;
Pose_t cur_pose;
std::mutex path_lock;

void path_pubRECV(const nav_msgs::Path::ConstPtr msg, std::vector<Pose_t> *path)
{
    std::lock_guard<std::mutex> lock(path_lock);
    rec_flag.store(true, std::memory_order_relaxed);
    path->clear();
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
    ROS_INFO_THROTTLE(1.0, "current pose:x %lf, y %lf, yaw %lf", cur_pose.x, cur_pose.y, cur_pose.yaw);
}

// global path need process

int main(int argc, char** argv)
{
    // init
    ros::init(argc, argv, "ctrl_node");
    ros::NodeHandle n;

    rec_flag = 0;
    string PathName = "";   //
    string controller_type = "";
    if (not n.getParam("selected_path", PathName))
        PathName = "Dijkstra_path";
    if (not n.getParam("controller_type", controller_type))
        controller_type = "dwa";
    int frequency = 20;

    std::vector<Pose_t> global_path;
    ros::Subscriber path_sub = n.subscribe<nav_msgs::Path>(PathName, 1, boost::bind(&path_pubRECV, _1, &global_path));
    ros::Subscriber pos_sub = n.subscribe("car_position", 1, UpdatePose);

    ros::Publisher speed_pub = n.advertise<geometry_msgs::Twist>("cmd", 1);
    ros::Publisher local_path_pub = n.advertise<nav_msgs::Path>("local_path", 1);

    ros::Rate r(frequency);
    ros::AsyncSpinner spinner(1); // Use another to subscribe position

    ROS_INFO("Motion control ready!");
    double tolerance = 0.05;

    MotionController controller;
    controller.InitController(0.05, frequency);
    controller.SetLinearPID(0.04, 0.005, 0);
    controller.SetAngularPID(0.08, 0.001, 0);
    
    DWA_planner local_planner;

    // speed set to zero
    std::pair<double, double> cur_speed = {0.0, 0.0};

    geometry_msgs::Twist speed;
    speed.angular.z = 0;
    speed.linear.x = 0;
    speed_pub.publish(speed);

    spinner.start();
    while(ros::ok())
    {
        ROS_INFO_THROTTLE(5.0, "current navigation path: %s, current controller: %s", PathName.c_str(), controller_type.c_str());
        if(rec_flag.load(std::memory_order_relaxed))
        {
            rec_flag.store(false, std::memory_order_relaxed);
            if (controller_type == "dwa")
            {
                path_lock.lock();   
                local_planner.UpdatePlanning(global_path);
                path_lock.unlock();
            }
            else if (controller_type == "pid")
            {
                path_lock.lock();
                controller.setGlobalPath(global_path);
                global_path.clear();
                path_lock.unlock();
            }
            while(!rec_flag.load(std::memory_order_relaxed)) // interrupt when get new goal;
            {
                if (rec_flag.load(std::memory_order_relaxed))
                    break;

                if (sqrt(pow(cur_pose.x - global_path.front().x, 2) +
                        pow(cur_pose.y - global_path.front().y, 2)) < tolerance)
                {
                    speed.linear.x = 0;
                    speed.angular.z = 0;
                    speed_pub.publish(speed);
                    ROS_INFO("Navigation finish!");
                    break;
                }
                if (controller_type == "pid")
                    cur_speed = controller.CalculateValue(cur_pose, cur_speed.first, cur_speed.second);
                else if (controller_type == "dwa")
                {
                    local_planner.CalculateSpeed(cur_pose, cur_speed);
                    local_path_pub.publish(local_planner.GetLastTrajectory());
                }
                speed.linear.x = cur_speed.first;
                speed.angular.z = cur_speed.second;
                ROS_INFO_THROTTLE(2.0, "Robot current speed: angular %f, linear %f", cur_speed.second, cur_speed.first);
                speed_pub.publish(speed);
                ros::spinOnce();
                r.sleep();
            }
        }
        r.sleep();
    }
    ros::waitForShutdown();
    return 0;
}