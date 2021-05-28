#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "odom_tf");
  ros::NodeHandle n;

  ros::Rate r(60);

  tf::TransformBroadcaster broadcaster;

  //map到odom转换
  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"map", "odom"));
    r.sleep();
  }
}