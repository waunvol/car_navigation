#pragma once
#include "PIDcontroller.h"
#include <nav_msgs/Path.h>

struct Pose_t{
    double x, y ,yaw;
};


class MotionController {
public:
    MotionController() {
        error_pub = n.advertise<geometry_msgs::Vector3>("cont_err", 1);
    }

    void setGlobalPath(const std::vector<Pose_t> &g_path) {
        remaining_path = g_path;
        ResetController();
    }

    void InitController(double l_tol, double freq) {
        linear_tol = l_tol;
        t = 1.0f/freq;
    }

    void SetLinearPID(double p, double i, double d) {
        linear_speed_controller.setPID(p, i, d);
    }

    void SetAngularPID(double p, double i, double d) {
        angular_speed_controller.setPID(p, i, d);
    }

    // calculate error and return pid control value.
    std::pair<double, double> CalculateValue(const Pose_t &cur_pose, const double vl_0, const double va_0) {
        double line_err = sqrt(pow(remaining_path.back().x - cur_pose.x, 2) +
                                pow(remaining_path.back().y - cur_pose.y, 2));

        if (line_err < linear_tol) {
            ROS_INFO("cur pt finish, next pt!");
            ResetController(); // enter next loop
            remaining_path.pop_back();
            line_err = sqrt(pow(remaining_path.back().x - cur_pose.x, 2) +
                            pow(remaining_path.back().y - cur_pose.y, 2));
        }

        double exp_yaw = std::atan2(remaining_path.back().y - cur_pose.y,
                                        remaining_path.back().x - cur_pose.x);

        double angle_err = exp_yaw - cur_pose.yaw;
        
        if (abs(angle_err + 2*M_PI) < angle_err) {
            angle_err = angle_err + 2*M_PI;
        } else if (abs(angle_err - 2*M_PI) < angle_err) {
            angle_err = angle_err - 2*M_PI;
        }
        // ROS_INFO("exp: %lf, cur: %lf, err: %lf", exp_yaw, cur_pose.yaw, angle_err);
        // v0*t + a*t*t/2 = err; ===> a = 2*(err - v0*t)/(t*t)
        // v_expt = v0 + a*t ===> v_err = a*t
        double vl_err = t * (2 * (line_err - vl_0 * t) / (t * t));
        double va_err = t * (2 * (angle_err - va_0 * t) / (t * t));

        // debug pub
        geometry_msgs::Vector3 msg;
        msg.x = vl_err;
        msg.y = va_err;
        error_pub.publish(msg);

        return {linear_speed_controller.getPIDcontrolValue(vl_err),
                angular_speed_controller.getPIDcontrolValue(va_err)};
    }


private:
    ros::NodeHandle n;
    ros::Publisher error_pub;
    PIDcontroller linear_speed_controller;
    PIDcontroller angular_speed_controller;
    double linear_tol = 0.05;
    double t;
    std::vector<Pose_t> remaining_path;

    void ResetController() {
        linear_speed_controller.reset();
        angular_speed_controller.reset();
    }

};
