//TEST
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
    float integralX, integralY, integralZ;
    float prevErrorX, prevErrorY, prevErrorZ;
    float dt;

    PID(float Dt, float Kp, float Ki, float Kd):
        dt(Dt),
        kP(Kp),
        kD(Kd),
        kI(Ki),
        prevErrorX(0),
	prevErrorY(0),
        prevErrorZ(0),
        integralX(0),
        integralY(0),
        integralZ(0) {}

    // PID calculation. Takes the error and the flag.
    //	axis = 0 for x, 1 for y, 2 for z.

    float calculate(float error, int axis) {
	float prevError, integral;
        switch (axis)
        {
        case 0:
	    prevError = prevErrorX;
	    integral = integralX;
            break;
        case 1:
            prevError = prevErrorY;
            integral = integralY;
            break;
        case 2:
        default:
            prevError = prevErrorZ;
            integral = integralZ;
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

	std::cout << "--------------PID------------------\n";
	std::cout << "Error: " << error << '\n';
	std::cout << "prevError: " << prevError << '\n';

	std::cout << "dt: : " << dt << '\n';
	std::cout << "IntegralX: " << integralX << '\n';
	std::cout << "IntegralY: " << integralY << '\n';

	std::cout << "kP: " << kP << "| P: " << outP << '\n';
	std::cout << "kI: " << kI << "| I: " << outI << '\n';
	std::cout << "kD: " << kD << "| D: " << outD << '\n';

        switch (axis)
        {
        case 0:
	    prevErrorX = error;
	    integralX = integral;
            break;
        case 1:
            prevErrorY = error;
            integralY = integral;
            break;
        case 2:
        default:
            prevErrorZ = error;
            integralZ = integral;
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

// Main class of the controller. Contains flags, target information, PID controller and other info.

class ControlCenter
{
    public:
        bool enabled;
        bool isBottomCamera;
        float imgRows, imgCols;
        float triCenterX, triCenterY;
        Box box;
        std::vector<Circle> targ;
        ros::Publisher cmdPublisher;
        PID pid;
        ros::Time lastLoop;
        ControlCenter(): pid(0.06, 0.07, 0, 0.02) {} // Started PID coefficients. Can be changed.
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


