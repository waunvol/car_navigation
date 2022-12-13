#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

class PathMarkers
{
private:
    ros::NodeHandle nh;
    ros::Publisher path_pub;
    const int queue_size = 5;

public:
    PathMarkers(/* args */)
    {
    }
    PathMarkers(const std::string &topic_name)
    {
        path_pub = nh.advertise<visualization_msgs::MarkerArray>(topic_name, queue_size);
    }
    ~PathMarkers()
    {
    }

    void ShowPathMarkers(const std::vector<nav_msgs::Path> &paths)
    {
        visualization_msgs::MarkerArray path_markers;
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 10.0;
        marker.color.g = 10.0;
        marker.color.b = 255.0;
        for(const auto &path:paths)
        {
            marker.header.frame_id = path.header.frame_id;
            marker.points.clear();
            for (const auto &point:path.poses)
            {
                geometry_msgs::Point pt;
                pt.x = point.pose.position.x;
                pt.y = point.pose.position.y;
                marker.points.emplace_back(pt);
            }
            path_markers.markers.emplace_back(marker);
        }
        path_pub.publish(path_markers);
    }
};
