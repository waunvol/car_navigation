#pragma once
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

struct Pose_t{
    double x, y ,yaw;
};

Pose_t GeomsgToPose(const geometry_msgs::Pose &input)
{
    Pose_t out;
    out.x = input.position.x;
    out.y = input.position.y;
    tf::Pose pose;
    tf::poseMsgToTF(input, pose);
    out.yaw = tf::getYaw(pose.getRotation());
    return out;
}

geometry_msgs::Pose PoseToGeomsg(const Pose_t &input)
{
    geometry_msgs::Pose out;
    out.position.x = input.x;
    out.position.y = input.y;
    out.orientation = tf::createQuaternionMsgFromYaw(input.yaw);
    return out;
}

nav_msgs::Path PoseArrayToPath(const std::vector<Pose_t> &input, const std::string &frame_id)
{
    nav_msgs::Path out;
    out.header.frame_id = frame_id;
    out.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = ros::Time::now();
    for (const auto &pt:input)
    {
        pose.pose = PoseToGeomsg(pt);
        out.poses.emplace_back(pose);
    }
    return out;
}

double getDist(const geometry_msgs::Pose &pose_1, const geometry_msgs::Pose &pose_2)
{
    return hypot(pose_1.position.x - pose_2.position.x, pose_1.position.y - pose_2.position.y);
}

double getDist(const Pose_t &pose_1, const Pose_t &pose_2)
{
    return hypot(pose_1.x - pose_2.x, pose_1.y - pose_2.y);
}