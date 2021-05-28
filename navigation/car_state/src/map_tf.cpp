#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>




int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_broadcaster");
    ros::NodeHandle n;

    ros::Rate rate(60);

    //设置地图的tf
    tf::TransformBroadcaster mapbr;
    tf::Transform transform;
    
    transform.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setW(1);
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    transform.setRotation(q);
    
    while (n.ok())
    {
        mapbr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map")); 
        rate.sleep();
    }
    
    return 0;
}
