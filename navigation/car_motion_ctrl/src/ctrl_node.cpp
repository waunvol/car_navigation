#include "PIDcontroller.h"
#include <nav_msgs/Path.h>
#include "dwa_planner.h"

bool rec_flag = 0;
nav_msgs::Path g_path;
geometry_msgs::Pose cur_pose;


void path_pubRECV(const nav_msgs::Path::ConstPtr msg, vector<pair<float, float>> *path)
{
    g_path = *msg;
    rec_flag = true;
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

    // ros::Publisher tra_pub = n.advertise<nav_msgs::Path>("dwa_trajectory", 5);
    ros::Publisher speed_pub = n.advertise<geometry_msgs::Twist>("cmd", 1);

    ros::Rate r(20);
    ros::AsyncSpinner spinner(1); // Use another to subscribe position

    ROS_INFO("Motion control ready!");
    double tolerance = 0.1;

    spinner.start();
    while(ros::ok())
    {
        if(rec_flag == true)
        {
            geometry_msgs::Twist sped;
            rec_flag = false;
            while(rec_flag == false) // interrupt when get new goal;
            {


                // check wether arrive the last goal
                if(sqrt(pow(cur_pose.position.x - g_path.poses.back().pose.position.x,2)+
                        pow(cur_pose.position.y - g_path.poses.back().pose.position.y,2)) <= tolerance) {
                    if(g_path.poses.size()<1) {
                        break;
                    }
                    g_path.poses.pop_back();
                }

                

                r.sleep();
            }
        }

    }
    ros::waitForShutdown();
    return 0;
}