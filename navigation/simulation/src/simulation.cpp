#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// simple simulator
// to do: add accleration limitation. (both angular and linear)

geometry_msgs::Twist cur_speed;
geometry_msgs::Pose cur_pose;

void SpeedListener(const geometry_msgs::Twist msg) {
    cur_speed = msg;
}

void PoseReset(const geometry_msgs::PoseWithCovarianceStamped msg) {
    cur_pose = msg.pose.pose;
}

// tf should be push actively
// speed should be subscribe
// speed should send to tf br

int main(int argc, char** argv) {
    ros::init(argc, argv, "car_tf_broadcaster");
    ros::NodeHandle n;

    ros::Subscriber speed_listener = n.subscribe("car/cmd", 1000, SpeedListener);
    ros::Subscriber pose_resetter = n.subscribe("/initialpose", 10, PoseReset);
    tf::TransformBroadcaster br;
    


    cur_pose.orientation.w =1;  //init current pose.

    ros::Rate r(50);
    while(ros::ok()) {
        ros::spinOnce();

        //  convertion between yaw angle and quaternion.
        tf::Transform transform;

        tf::Pose pose;
        tf::poseMsgToTF(cur_pose, pose);
        double yaw = tf::getYaw(pose.getRotation());

        tf::Quaternion q;
        q.setRPY(0,0,yaw + cur_speed.angular.z*0.02);
        tf::quaternionTFToMsg(q, cur_pose.orientation);

        cur_pose.position.x += cur_speed.linear.x*0.02*cos(yaw);
        cur_pose.position.y += cur_speed.linear.x*0.02*sin(yaw);

        transform.setRotation(q);
        transform.setOrigin(tf::Vector3(cur_pose.position.x,cur_pose.position.y,cur_pose.position.z));
        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "world", "base_footprint"));

        // ROS_INFO("cur_pos:%f,%f,%f ,%f,%f,%f,%f!",cur_pose.position.x,
        //                                         cur_pose.position.y,
        //                                         cur_pose.position.z,
        //                                         cur_pose.orientation.x,
        //                                         cur_pose.orientation.y,
        //                                         cur_pose.orientation.z,
        //                                         cur_pose.orientation.w);
        r.sleep();
    }
    return 0;
};

