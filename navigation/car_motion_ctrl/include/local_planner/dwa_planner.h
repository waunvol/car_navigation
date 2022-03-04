#pragma once
#include <iostream>
#include <limits>
#include <math.h>
#include <tuple>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <costmap_2d/costmap_2d_ros.h>

// property: 1. sampling period; 2. max acceleration; 3. have a y direction speed or not; 4. predict period
/*  get current velocity =>
    calculate all probable velocity according to max accerlation and take them into vector => 
    foreach vector to get all probable trajectory => 
    evaluate their score according to trajectory and accerlation =>
    get the max score as speed output.
*/

//reesolution should be match with model.

//two car model: 1.differential drive; 2. steer drive => we got steer first.
// add a funtion to show trajectory

using namespace std;

class DWA_planner {
private:
    double max_linear_a=0.5;
    double max_linear_v=1;
    double max_angular_a=0.78;
    double max_angular_v=1.57;
    int predict_cycle = 5;
    int sample_size = 7;
    double frequency = 0.1;
    nav_msgs::Path last_trajectory;

    // this funtion should match motion model
    inline nav_msgs::Path GetTrajectory(const tuple<double, double, double> &cur_pos, const double &v, const double &w) {
        nav_msgs::Path trajectory;
        double direction = get<2>(cur_pos);
        geometry_msgs::PoseStamped waypoint; 
        waypoint.pose.position.x = get<0>(cur_pos);
        waypoint.pose.position.y = get<1>(cur_pos);
        waypoint.header.frame_id = "world";

        ros::Time time = ros::Time::now();
        waypoint.header.stamp = time;
        for (size_t i = 0; i < predict_cycle; i++) {
            direction += w*frequency;
            waypoint.pose.position.x = waypoint.pose.position.x + v*frequency*cos(direction);
            waypoint.pose.position.y = waypoint.pose.position.y + v*frequency*sin(direction);
            waypoint.pose.orientation = tf::createQuaternionMsgFromYaw(direction);
            trajectory.poses.push_back(waypoint);
        }
        return trajectory;
    }


    int Evaluate(const vector<nav_msgs::Path> &trajectorys, const nav_msgs::Path &global_path) {
        //only current or behind global path point can be evaluate, not for front point.
        int index;

        // the scorl here just for global path matching.
        double min_score = numeric_limits<double>::max();
        for (size_t i = 0; i < trajectorys.size(); i++) {
            // each trajectory
            double cur_score = 0;
            size_t useful_point_index = global_path.poses.size()-1;
            for (size_t j = 0; j < trajectorys[i].poses.size(); j++) {
                // each trajectory point
                double min_dist = numeric_limits<double>::max();
                for (size_t k = useful_point_index; k >0; k--) {
                    // each path point
                    
                    double dist = sqrt(pow((trajectorys[i].poses[j].pose.position.x - global_path.poses[k].pose.position.x), 2)
                                         + pow((trajectorys[i].poses[j].pose.position.y - global_path.poses[k].pose.position.y), 2));
                    if (dist < min_dist) {
                        min_dist = dist;
                        useful_point_index = k-1;
                        // ROS_INFO("get k: %d", useful_point_index);
                    }
                }
                
                cur_score += min_dist;
            }
            // ROS_INFO("get a scorll: %f", cur_score);
            if (min_score > cur_score) {
                min_score = cur_score;
                index = i;
            }
        }

        return index;
    }

public:
    DWA_planner() {
        
    }
    DWA_planner(int local_costmap, double max_acceleration, double max_velocity) {
    }
    ~DWA_planner() {
    }
    // to do: show planning path; add param to control the fit
    nav_msgs::Path GetTrajectory() {
        last_trajectory.header.frame_id = "world";
        ros::Time time = ros::Time::now();
        last_trajectory.header.stamp = time;
        return last_trajectory;
    }

    bool CalculateSpeed(const nav_msgs::Path &global_path, const geometry_msgs::Pose &cur_pose_, geometry_msgs::Twist &cur_speed) {
        tuple<double, double, double> cur_pose = {cur_pose_.position.x, cur_pose_.position.y, tf::getYaw(cur_pose_.orientation)};

        vector<double> linear_speed_sample, angular_speed_sample;
        double v = cur_speed.linear.x - max_linear_a,
               w = cur_speed.angular.z - max_angular_a;
        for (size_t i = 0; i <= sample_size; i++) {
            // ROS_INFO("Get v:%f, w:%f", v, w);
            if (v > 0 && v <= max_linear_v) {
                linear_speed_sample.push_back(v);
            }
            if (abs(w) <= max_angular_v) {
                angular_speed_sample.push_back(w);
            }
            v += (2.0f / sample_size) * max_linear_a;
            w += (2.0f / sample_size) * max_angular_a;
        }

        if(linear_speed_sample.size()<1 or angular_speed_sample.size()<1){
            return false;
        }

        vector<nav_msgs::Path> trajectorys_set;
        for (size_t i = 0; i < linear_speed_sample.size(); i++) {
            for (size_t j = 0; j < angular_speed_sample.size(); j++) {
                trajectorys_set.push_back(GetTrajectory(cur_pose, 
                                                        linear_speed_sample[i] ,
                                                        angular_speed_sample[j]));
            }
        }

        int best = Evaluate(trajectorys_set, global_path);
        last_trajectory = trajectorys_set[best];
        cur_speed.linear.x = linear_speed_sample[best%linear_speed_sample.size()];
        cur_speed.angular.z = angular_speed_sample[best/linear_speed_sample.size()];

        return true;
    }

    void SetSpeedConfig(double max_lin_v, double max_lin_a, double max_ang_v, double max_ang_a){
        max_linear_v = max_lin_v;
        max_linear_a = max_lin_a;
        max_angular_v = max_ang_v;
        max_angular_a = max_ang_a;
    }
};
