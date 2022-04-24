#pragma once
#include "PIDcontroller.h"
#include <nav_msgs/Path.h>

struct Pose_t{
    double x, y ,yaw;
};


class MotionController:public PIDcontroller {
public:
    void setGlobalPath(const std::vector<Pose_t> &g_path) {
        remaining_path = g_path;
        reset();
    }

    void setTolerance(double a_tol,  double l_tol) {
        angular_tol = a_tol, linear_tol = l_tol;
    }

    std::pair<double, double> CalculateValue(const Pose_t &cur_pose, const double va_0, const double vl_0) {
        double angle_err = remaining_path.back().yaw - cur_pose.yaw;
        double line_err = sqrt(pow(remaining_path.back().x - cur_pose.x, 2) +
                                pow(remaining_path.back().y - cur_pose.y, 2));

        if (line_err < linear_tol && angle_err < angular_tol)
        {
            reset(); // enter next loop
            remaining_path.pop_back();
            angle_err = remaining_path.back().yaw - cur_pose.yaw;
            line_err = sqrt(pow(remaining_path.back().x - cur_pose.x, 2) +
                            pow(remaining_path.back().y - cur_pose.y, 2));
        }
        // v0*t + a*t*t/2 = err; ===> a = 2*(err - v0*t)/(t*t)
        // v_expt = v0 + a*t ===> v_err = a*t
        double vl_err = t * (2 * (line_err - vl_0 * t) / (t * t));
        double va_err = t * (2 * (angle_err - vl_0 * t) / (t * t));
        
        return {getPIDcontrolValue(vl_err), getPIDcontrolValue(va_err)};
    }


private:
    double angular_tol = 0.1, linear_tol = 0.05;
    double t;
    std::vector<Pose_t> remaining_path;
    
    double getAngleErr() {
        return 0.0;
    }
    double getLineErr(const geometry_msgs::Pose &cur_pose) {
        return 0.0;
    }

};
