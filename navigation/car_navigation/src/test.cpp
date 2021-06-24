#include <ros/ros.h>
#include <map_server/image_loader.h>
#include <nav_msgs/GetMap.h>
#include "Dijkstra.h"
#include "Astar.h"


using namespace std;

//用于测试代码
ros::Publisher pathresult_D;
ros::Publisher pathresult_A;



int main(int argc, char** argv )
{
    ros::init(argc, argv, "testfn");
    ros::NodeHandle n;

    nav_msgs::GetMap mapRev;
    nav_msgs::Path result_D;
    nav_msgs::Path result_A;

    ros::ServiceClient recMap = n.serviceClient<nav_msgs::GetMap>("static_map");
    pathresult_D = n.advertise<nav_msgs::Path>("pathD", 5);
    pathresult_A = n.advertise<nav_msgs::Path>("pathA", 5);

    if(recMap.call(mapRev))
    {
        ROS_INFO("server has been connectted!");
    }
    else
    {
        ROS_INFO("May something worrid");
        return 0;
    }

    dijkstra test_Dij(mapRev);
    astar test_Astar(mapRev);
    double x, y;
    // while(1)
    // {
        
    //     ROS_INFO("please enter target x and target y");
    //     cout << "x = " ;
    //     cin >> x;
    //     cout << endl;
    //     cout << "y = ";
    //     cin >> y;
    //     cout << endl;
    //     test_Dij.setTargetPoint(x, y);
    //     if(!test_Dij.search())
    //     {
    //         ROS_INFO("search failed, please try agin");
    //     }
    //     else
    //     {
    //         break;
    //     }
        
    // }
    x=3;
    y=13.8;
    test_Dij.setTargetPoint(x, y);
    test_Astar.setTargetPoint(x, y);

    if(!test_Dij.search())
    {
        ROS_INFO("Dijkstra search failed, please try agin");
    }
    else
    {
        ROS_INFO("Dijkstra succeed!");
    }
    waypoint* wayPTR = test_Dij.map[test_Dij.target_y][test_Dij.target_x]->way;
    geometry_msgs::PoseStamped wp;
    wp.pose.position.z = 0;
    wp.pose.orientation.x =0;
    wp.pose.orientation.y=0;
    wp.pose.orientation.z=0;
    wp.pose.orientation.w=1;
    wp.header.frame_id="base_footprint";
    result_D.header.stamp = ros::Time::now();
    result_D.header.frame_id = "base_footprint";
    while (wayPTR != nullptr)
    {
        wp.pose.position.x = (wayPTR->x + 1)*0.05 - 1;
        wp.pose.position.y = (wayPTR->y + 1)*0.05 - 13.8;
        wp.header.stamp = ros::Time::now();
        result_D.poses.insert(result_D.poses.begin(), wp);
        wayPTR = wayPTR->front;
    }

    if(!test_Astar.search())
    {
        ROS_INFO("Astar search failed, please try agin");
    }
    else
    {
        ROS_INFO("Astar succeed!");
    }
    wayPTR = test_Astar.map[test_Dij.target_y][test_Dij.target_x]->way;
    wp.header.frame_id="base_footprint";
    result_A.header.stamp = ros::Time::now();
    result_A.header.frame_id = "base_footprint";
    while (wayPTR != nullptr)
    {
        wp.pose.position.x = (wayPTR->x + 1)*0.05 - 1;
        wp.pose.position.y = (wayPTR->y + 1)*0.05 - 13.8;
        wp.header.stamp = ros::Time::now();
        result_A.poses.insert(result_A.poses.begin(), wp);
        wayPTR = wayPTR->front;
    }

    ros::Rate r(1);
    while (ros::ok())
    {
        pathresult_D.publish(result_D);
        pathresult_A.publish(result_A);
        r.sleep();
    }
    



    return 0;
}
