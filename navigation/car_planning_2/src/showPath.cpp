#include <ros/ros.h>
#include "hybridAstar.h"
#include <nav_msgs/Path.h>
#include "car_planning/cost.h"
#include "RRT.h"
#include "tf/transform_datatypes.h"
#include "steepest.h"

using namespace std;

bool rec_flag=0;
// bool rec_flag1=0;

void getGoal(const geometry_msgs::PoseStamped::ConstPtr msg, vector<float> *goal)
{
    (*goal)[0] = (*msg).pose.position.x;
    (*goal)[1] = (*msg).pose.position.y;

    double R,P,Y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF((*msg).pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(R,P,Y);
    (*goal)[2] = Y;

    rec_flag = true;

}

void getPosition(const geometry_msgs::Pose::ConstPtr msg, vector<float> *starting)
{
    (*starting)[0] = (*msg).position.x;
    (*starting)[1] = (*msg).position.y;

    double R,P,Y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF((*msg).orientation, quat);
    tf::Matrix3x3(quat).getRPY(R,P,Y);
    (*starting)[2] = Y;
}

// void showCOST(const geometry_msgs::PointStamped::ConstPtr msg, vector<float> *co)
// {
//     rec_flag1=true;
//     (*co)[0]=(*msg).point.x+1.0;
//     (*co)[1]=(*msg).point.y+13.8;
// }

nav_msgs::Path getPath(const vector<vector<float>> &route)
{
    nav_msgs::Path path;

    ros::Time time = ros::Time::now();
    path.header.frame_id = "map";
    path.header.stamp = time;
    for(auto it:route)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = time;
        pose.header.frame_id = "map";
        pose.pose.position.x = it[0];
        pose.pose.position.y = it[1];
        // pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    return path;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ShowPath");
    ros::NodeHandle n;

    vector<int8_t> *cost;
    vector<float> starting={.0, .0, .0};
    vector<float> goals={.0, .0, .0};
    vector<float> co={.0, .0, .0};
    ros::Subscriber GetGoal = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&getGoal, _1, &goals));
    ros::Subscriber GetPosition = n.subscribe<geometry_msgs::Pose>("car_position", 1, boost::bind(&getPosition, _1, &starting));
    // ros::Subscriber costpub = n.subscribe<geometry_msgs::PointStamped>("clicked_point", 1, boost::bind(&showCOST, _1, &co));
    ros::Publisher pathpub_A = n.advertise<nav_msgs::Path>("hybridAstar",3);
    ros::Publisher pathpub_R = n.advertise<nav_msgs::Path>("RRT",3);

    // ros::Publisher pathpub_R_2 = n.advertise<nav_msgs::Path>("steep",3);

    costmap cost_ha;
    cost_ha.setInterferenceArea(0.12, 1.0);
    cost_ha.calculateCOST();

    //混合A* 设置
    hybridAstar plan_1;
    plan_1.setCOST(cost_ha.hig, cost_ha.wid, &cost_ha.COST);

    //RRT 设置
    RRT_planner plan_2;
    plan_2.setCOSTMAP(cost_ha.hig, cost_ha.wid, &cost_ha.COST);

    ros::Rate r(100);
    ROS_INFO("now start!");

    nav_msgs::Path Amsg;    
    nav_msgs::Path Rmsg;
    // nav_msgs::Path Rmsg_2;

    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        // if(rec_flag1)
        // {
        //     ROS_INFO("cost:%d", cost_ha.getCOST(co[0], co[1]));
        //     rec_flag1=0;
        // }

        if(rec_flag)
        {
            rec_flag = false;
            ROS_INFO("Got new goal!");
            vector<vector<float>> route_1;
            vector<vector<float>> route_2;
            if(plan_1.calculateRoute(&starting, &goals))
            {
                route_1 = plan_1.getRoute();
                Amsg = getPath(route_1);
                pathpub_A.publish(Amsg);
            }
            if(plan_2.searchRoute(&starting, &goals))
            {
                route_2 = plan_2.getRoute();
                // Rmsg = getPath(route_2);
                // pathpub_R.publish(Rmsg);

                //梯度下降优化路径
                steepest exp;
                // route_2 = exp.steepest_descent(40, route_2);
                // Rmsg_2 = getPath(route_2);
                // pathpub_R_2.publish(Rmsg_2);

                route_2 = exp.steepest_descent(40, route_2);
                Rmsg = getPath(route_2);
                pathpub_R.publish(Rmsg);

            }
            route_1.clear();
            route_2.clear();
            r.sleep();
        }

    }

    return 0;
}