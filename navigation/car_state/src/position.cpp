#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

//接受tf，发布小车当前的位置信息
int main(int argc, char** argv){
    ros::init(argc, argv, "position_listener");
    ros::NodeHandle node;

    ros::Publisher positiion_pub = node.advertise<geometry_msgs::Pose>("car_position",3);
    geometry_msgs::Pose msg;

    tf::TransformListener listener;

    ros::Rate rate(100);
    while (node.ok())
    {
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        msg.position.x = transform.getOrigin().getX();
        msg.position.y = transform.getOrigin().getY();
        msg.position.z = transform.getOrigin().getZ();
        msg.orientation.w = transform.getRotation().w();
        msg.orientation.x = transform.getRotation().x();
        msg.orientation.y = transform.getRotation().y();
        msg.orientation.z = transform.getRotation().z();
        positiion_pub.publish(msg);

        rate.sleep();
    }
    return 0;
};