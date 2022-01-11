#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

tf::TransformBroadcaster br;
tf::Transform transform;

geometry_msgs::Pose cur_pose;

void SpeedListener(const geometry_msgs::Twist msg) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(cur_pose.orientation, quat);

    double angle = quat.getAngle();

    angle += msg.angular.z*0.02;
    cur_pose.position.x += msg.linear.x*0.02;
    cur_pose.position.y += msg.linear.y*0.02;

    cur_pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    
    transform.setRotation(tf::Quaternion(cur_pose.orientation.x,cur_pose.orientation.y, cur_pose.orientation.z,cur_pose.orientation.w));
    transform.setOrigin(tf::Vector3(cur_pose.position.x,cur_pose.position.y,cur_pose.position.z));
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "world", "base_footprint"));
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "car_tf_broadcaster");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd", 1000, SpeedListener);
    cur_pose.orientation.w =1;

    double x,y = 0.0;
    double yaw = 0;

    ros::Rate r(50);

    ros::Subscriber speed_listener;

    ros::spin();
    return 0;
};

