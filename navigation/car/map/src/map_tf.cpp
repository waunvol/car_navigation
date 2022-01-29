#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_tf_broadcaster");
    ros::NodeHandle n;


    ros::Rate r(50);

    tf::TransformBroadcaster br;
    while (ros::ok())
    {
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(1.0, 13.8, 0.0) );
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
      r.sleep();
    }
    
}