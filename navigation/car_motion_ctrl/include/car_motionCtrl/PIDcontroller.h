#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include "tf/transform_datatypes.h"
#include <std_msgs/Float32.h>

using namespace std;

class PIDcontroller
{
public:
    PIDcontroller() {

    }
    ~PIDcontroller(){

    }

    double getPIDcontrolValue(double err) {
        integral += err;
        return P*err + I*integral + D*(err - pre_err);
    }
    void setPID(double p, double i, double d) {
        P = p, I = i, D = d;
    }
    void reset() {
        integral = 0;
        pre_err = 0;
    }

private:
    double P = 1.0, I = 0.0, D = 0.0;
    double integral = 0;
    double pre_err = 0;

};
