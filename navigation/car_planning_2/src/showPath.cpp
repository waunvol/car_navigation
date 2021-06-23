#include <ros/ros.h>
#include "hybridAstar.h"
#include <nav_msgs/Path.h>
#include "car_planning/cost.h"
#include "tf/transform_datatypes.h"

using namespace std;

bool rec_flag=0;
// bool rec_flag1=0;

void getGoal(const geometry_msgs::PoseStamped::ConstPtr msg, vector<float> *goal)
{
    (*goal)[0] = (*msg).pose.position.x+1.0;
    (*goal)[1] = (*msg).pose.position.y+13.8;

    double R,P,Y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF((*msg).pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(R,P,Y);
    (*goal)[2] = Y;

    rec_flag = true;

}

void getPosition(const geometry_msgs::Pose::ConstPtr msg, vector<float> *starting)
{
    (*starting)[0] = (*msg).position.x+1.0;
    (*starting)[1] = (*msg).position.y+13.8;

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
        pose.pose.position.x = it[0] - 1;
        pose.pose.position.y = it[1] - 13.8;
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

    costmap cost_ha;
    cost_ha.setInterferenceArea(0.12, 1.0);
    cost_ha.calculateCOST();

    hybridAstar plan_1;
    plan_1.setCOST(cost_ha.hig, cost_ha.wid, &cost_ha.COST);

    ros::Rate r(100);
    ROS_INFO("now start!");

    nav_msgs::Path Pmsg;

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
            vector<vector<float>> route;
            if(plan_1.calculateRoute(&starting, &goals))           
                route = plan_1.getRoute();
            else continue;

            Pmsg = getPath(route);
            pathpub_A.publish(Pmsg);
            route.clear();
            r.sleep();
        }

    }

    return 0;
}