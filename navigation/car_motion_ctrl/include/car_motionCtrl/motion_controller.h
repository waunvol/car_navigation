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

    void CalculateValue(const Pose_t &cur_pose) {
        double angle_err;
        double line_err = sqrt(pow(cur_pose.x - remaining_path.back().x,2)+
                                pow(cur_pose.y - remaining_path.back().y,2));


        if (line_err < linear_tol && angle_err < angular_tol)
        {
            reset(); // enter next loop
            remaining_path.pop_back();
        }
    }


private:
    double angular_tol = 0.1, linear_tol = 0.05;
    std::vector<Pose_t> remaining_path;
    
    double getAngleErr() {
        return 0.0;
    }
    double getLineErr(const geometry_msgs::Pose &cur_pose) {
        return 0.0;
    }

};
