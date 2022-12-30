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
    double max_linear_a=0.2;
    double max_linear_v=1;
    double max_angular_a=0.2;
    double max_angular_v=1.57;
    int predict_cycle = 5;
    int sample_size = 7;
    double frequency = 0.1;

    const int global_path_sample_size = 6;
    vector<Pose_t> cur_global_path;     //update while navigation executing.

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

    void CutGlobalPath(const Pose_t &cur_pose)
    {
        vector<Pose_t> result;
        // get cloest index
        double min_dist = DBL_MAX;
        int min_index;
        for (int i=0; i<cur_global_path.size(); ++i)
        {
            double cur_dist = getDist(cur_pose, cur_global_path[i]);
            if (cur_dist < min_dist)
            {
                min_dist = cur_dist;
                min_index = i;
            }
        }

        for (int i=0; i<=min_index; ++i)
        {
            result.emplace_back(cur_global_path[i]);
        }
        cur_global_path = result;
    }

    int DistEvaluate(const vector<vector<Pose_t>> &trajectorys) {
        //only current or behind global path point can be evaluate, not for front point.
        int index;

        // the scorl here just for global path matching.
        double min_score = DBL_MAX;

        for (int index_=0; index_<trajectorys.size(); ++index_) {
            // cal Dtw distance
            vector<vector<double>> dist_mtr;
            for (auto pt:trajectorys[index_]) {
                vector<double> dist_array;
                int end_index = max(0, int(cur_global_path.size() - global_path_sample_size));
                for (int i=cur_global_path.size()-1; i>=end_index; --i) {
                    dist_array.emplace_back(getDist(cur_global_path[i], pt));
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
                    if (i+1 < dist_mtr.size() and dp[i+1][j] > dp[i][j] + dist_mtr[i+1][j])
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
        // ROS_INFO("The best match index is %d of the trajectory set[%d]", index, trajectorys.size());
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

    void UpdatePlanning(const vector<Pose_t> &global_path)
    {
        ROS_INFO("Receive new path, continue planning.");
        cur_global_path = global_path;
    }

    bool CalculateSpeed(const Pose_t &cur_pose_, std::pair<double, double> &cur_speed) {
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

        CutGlobalPath(cur_pose_);

        vector<vector<Pose_t>> trajectorys_set;
        for (size_t i = 0; i < linear_speed_sample.size(); i++) {
            for (size_t j = 0; j < angular_speed_sample.size(); j++) {
                trajectorys_set.push_back(GetTrajectory(cur_pose_, 
                                                        linear_speed_sample[i] ,
                                                        angular_speed_sample[j]));
            }
        }
        // path_markers.ShowPathMarkers(trajectorys_set);

        int best = DistEvaluate(trajectorys_set);
        last_trajectory = trajectorys_set[best];
        cur_speed.first = linear_speed_sample[best/(angular_speed_sample.size())];
        cur_speed.second = angular_speed_sample[best%angular_speed_sample.size()];

        return true;
    }

    void SetSpeedConfig(double max_lin_v, double max_lin_a, double max_ang_v, double max_ang_a){
        max_linear_v = max_lin_v;
        max_linear_a = max_lin_a;
        max_angular_v = max_ang_v;
        max_angular_a = max_ang_a;
    }
};
