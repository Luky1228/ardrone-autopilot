#include <string>
#include <iostream>
#include "imgHelper.h"

/*
    controlHelper.h
  ----------------------------------------------------------------------------
  | Contains some utility classes and functions for the controller.cpp.      |
  ----------------------------------------------------------------------------

*/


// PID controller realization.

class PID {
 public:
    float kP, kD, kI;
    float integralX, integralY, integralZ, integralRot;
    float prevErrorX, prevErrorY, prevErrorZ, prevErrorRot;
    float dt;

    PID(float Dt, float Kp, float Ki, float Kd):
        dt(Dt),
        kP(Kp),
        kD(Kd),
        kI(Ki),
        prevErrorX(0),
	    prevErrorY(0),
        prevErrorZ(0),
        prevErrorRot(0),
        integralX(0.0),
        integralY(0.0),
        integralZ(0.0),
        integralRot(0.0) {}

    // PID calculation. Takes the error and the flag.
    //	axis = 0 for x, 1 for y, 2 for z, 3 for rotation.

    float calculate(float error, int axis) {
        float prevError, integral;
        switch (axis) {
            case 0:
                prevError = prevErrorX;
                integral = integralX;
                break;
            case 1:
                prevError = prevErrorY;
                integral = integralY;
                break;
            case 2:
                prevError = prevErrorZ;
                integral = integralZ;
                break;
            default:
                prevError = prevErrorRot;
                integral = integralRot;
                break;
        }
        float outP = kP * error;

        integral += error * dt;
        float outI = kI * integral;

        float derivative;

        if (prevError != 0) {
            derivative = (error - prevError) / dt;
        } else {
            derivative = 0;
        }

        float outD = kD * derivative;

        float output = outP + outI + outD;

        /*std::cout << "--------------PID------------------\n";
        std::cout << "Error: " << error << '\n';
        std::cout << "prevError: " << prevError << '\n';

        std::cout << "dt: : " << dt << '\n';
        std::cout << "IntegralX: " << integralX << '\n';
        std::cout << "IntegralY: " << integralY << '\n';

        std::cout << "kP: " << kP << "| P: " << outP << '\n';
        std::cout << "kI: " << kI << "| I: " << outI << '\n';
        std::cout << "kD: " << kD << "| D: " << outD << '\n';*/

        switch (axis) {
            case 0:
                prevErrorX = error;
                integralX = integral;
                break;
            case 1:
                prevErrorY = error;
                integralY = integral;
                break;
            case 2:
                prevErrorZ = error;
                integralZ = integral;
                break;
            default:
                prevErrorRot  = error;
                integralRot = integral;
                break;
        }
        return output;
    }

};


class Circle {
 public:
    float x, y, width, height;
    bool inTheBox;

    Circle() {}

    Circle(float _x, float _y, float _width, float _height, bool b):
        x(_x),
        y(_y),
        width(_width),
        height(_height),
        inTheBox(b)
         {}
};

// Main class of the controller. Contains flags, target information, PID controller and other info
class ControlCenter {
 public:
    bool enabled;
    bool isBottomCamera;
    float imgRows, imgCols;
    float triCenterX, triCenterY, rotError;
    Box box;
    std::vector<Circle> targ;
    ros::Publisher cmdPublisher;
    PID frontPid;  //Pid for velocity, Pid2 for acceleration
    PID frontPid2;
    PID bottomPid;
    PID bottomPid2;
    int leftCounter = 0;
    int rightCounter = 0;
    ros::Time lastLoop;
    // Initial PID coefficients. Can be changed.
    ControlCenter(): 
        frontPid(0.06, 0.07, 0.004, 0.02), 
        frontPid2(0.3, 0.25, 0.02, 0.1),
        bottomPid(0.06, 0.07, 0.004, 0.02), 
        bottomPid2(0.3, 0.25, 0.02, 0.1) {}
};



// Circle output
std::ostream &operator<<(std::ostream &os, Circle& c) {
    std::string output;
    output = "X: " + std::to_string(c.x) + '\n';
    output += "Y: " + std::to_string(c.y) + '\n';
    output += "Width: " + std::to_string(c.width) + '\n';
    output += "Height: " + std::to_string(c.height) + '\n';
    output += "InTheBox: " + std::to_string(c.inTheBox) + '\n';
    return os << output;
}


