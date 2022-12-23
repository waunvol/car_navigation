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
#include "type.h"
#include "utility/visualizaion.h"

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
    vector<Pose_t> last_trajectory;
    PathMarkers path_markers = PathMarkers("show_cal_paths");   // for debug

    // this funtion should match motion model
    inline vector<Pose_t> GetTrajectory(const Pose_t &cur_pos, const double &v, const double &w) {
        vector<Pose_t> trajectory;
        Pose_t waypoint = cur_pos; 
        trajectory.push_back(waypoint);

        for (size_t i = 0; i < predict_cycle; i++) {
            waypoint.yaw+= w*frequency;
            waypoint.x = waypoint.x + v*frequency*cos(waypoint.yaw);
            waypoint.y = waypoint.y + v*frequency*sin(waypoint.yaw);
            trajectory.push_back(waypoint);
        }
        return trajectory;
    }


    int DistEvaluate(const vector<vector<Pose_t>> &trajectorys, const vector<Pose_t> &global_path) {
        //only current or behind global path point can be evaluate, not for front point.
        int index;

        // the scorl here just for global path matching.
        double min_score = DBL_MAX;
        /*
        for (int i = 0; i < trajectorys.size(); i++) {
            // each trajectory
            double cur_score = 0;
            size_t useful_point_index = global_path.size()-1;
            for (int j = 0; j < trajectorys[i].size(); j++) {
                // each trajectory point
                double min_dist = DBL_MAX;
                for (int k = useful_point_index; k >= 0; k--) {
                    // each path point
                    
                    double dist = sqrt(pow((trajectorys[i][j].x - global_path[k].x), 2)
                                         + pow((trajectorys[i][j].y - global_path[k].y), 2));
                    if (dist < min_dist) {
                        min_dist = dist;
                        k-1<=0 ? useful_point_index = 0 : useful_point_index = k-1;
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
        */

        for (int index_=0; index_<trajectorys.size(); ++index_) {
            // cal Dtw distance
            vector<vector<double>> dist_mtr;
            for (auto pt:trajectorys[index_]) {
                vector<double> dist_array;
                for (int i = global_path.size()-1; i >=0; i--)
                {
                    dist_array.emplace_back(getDist(global_path[i], pt));
                }
                dist_mtr.emplace_back(dist_array);
            }

            // calculate the less value form d[0][0] to d[i][j]
            vector<vector<double>> dp(dist_mtr.size(), vector<double>(dist_mtr[0].size(), DBL_MAX));
            dp[0][0] = dist_mtr[0][0];
            for (int i=0; i<dist_mtr.size(); i++) {
                for (int j=0; j<dist_mtr[i].size(); j++) {
                    if (j+1 < dist_mtr[i].size() and dp[i][j+1] > dp[i][j] + dist_mtr[i][j+1])
                        dp[i][j+1] = dp[i][j] + dist_mtr[i][j+1];
                    if (i+1 < dist_mtr.size() and dp[i][j+1] > dp[i][j] + dist_mtr[i][j+1])
                        dp[i+1][j] = dp[i][j] + dist_mtr[i+1][j];
                    if (j+1 < dist_mtr[i].size() and i+1 < dist_mtr.size()
                        and dp[i+1][j+1] > dp[i][j] + dist_mtr[i+1][j+1])
                        dp[i+1][j+1] = dp[i][j] + dist_mtr[i+1][j+1];
                }
            }
            if (dp.back().back() < min_score) {
                index = index_;
                min_score = dp.back().back();
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
    nav_msgs::Path GetLastTrajectory() {
        return PoseArrayToPath(last_trajectory, "world");
    }

    bool CalculateSpeed(const vector<Pose_t> &global_path, const Pose_t &cur_pose_, std::pair<double, double> &cur_speed) {
        /*To do:
        1. cut global path from current pose (provide sample to adjust);
        2. invert global path when cut;
        3. fix evaluate bugs. */

        vector<double> linear_speed_sample, angular_speed_sample;
        double v = cur_speed.first - max_linear_a,
               w = cur_speed.second - max_angular_a;
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

        vector<vector<Pose_t>> trajectorys_set;
        for (size_t i = 0; i < linear_speed_sample.size(); i++) {
            for (size_t j = 0; j < angular_speed_sample.size(); j++) {
                trajectorys_set.push_back(GetTrajectory(cur_pose_, 
                                                        linear_speed_sample[i] ,
                                                        angular_speed_sample[j]));
            }
        }
        ROS_INFO("Size of trajectory %d", trajectorys_set.size());
        path_markers.ShowPathMarkers(trajectorys_set);

        int best = DistEvaluate(trajectorys_set, global_path);
        last_trajectory = trajectorys_set[best];
        cur_speed.first = linear_speed_sample[best%linear_speed_sample.size()];
        cur_speed.second = angular_speed_sample[best/linear_speed_sample.size()];

        return true;
    }

    void SetSpeedConfig(double max_lin_v, double max_lin_a, double max_ang_v, double max_ang_a){
        max_linear_v = max_lin_v;
        max_linear_a = max_lin_a;
        max_angular_v = max_ang_v;
        max_angular_a = max_ang_a;
    }
};
