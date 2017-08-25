
/*
   controller.cpp
   ----------------------------------------------------------------------------
   | Receives the image from the drone and sends it to the compVision.cpp,    |
   | that processes it and extracts information about the circles and the box.|
   |                                                                          |
   | Then receives processed information and sends it to the controller.      |
   | Some utility classes and functions can be found in the controlHelper.h.  |
   ----------------------------------------------------------------------------

*/


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "controlHelper.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "constants.h"


// Global control object to have an easy access to some parameters
struct ControlCenter control;

// Enable/disable controller by button 'N'
void onEnableCtrl(const std_msgs::Empty& toggle_msg) {
    control.enabled = !control.enabled;
    if (control.enabled)
        std::cout << "Target autopilot enabled.\n";
    else
        std::cout << "Target autopilot disabled.\n";
}


// Extract information from incoming message to the control object
void parseArray(const std_msgs::Float32MultiArray& msg) {
    if (msg.data.size() != 0) {
        size_t i = 0;
        Circle circle;
        std::cout << "parse" << std::endl;
        while (i < msg.data.size()) {
            circle.x = msg.data[i++];
            circle.y = msg.data[i++];
            circle.width = msg.data[i++];
            circle.height = msg.data[i++];
            circle.inTheBox = !msg.data[i++];
            control.targ.push_back(Circle(circle));
        }
    }
}


// Drone autopilot control
void controller(geometry_msgs::Twist& msg) {
    std::vector<float> errorsX;
    std::vector<float> errorsY;
    float circleRadius = 0.0;
    float middleX = (control.box.right+control.box.left) / 2;
    float middleY = (control.box.top+control.box.bottom) / 2;

    // Calculate X and Y errors of the circles that are not in the box
    for (const auto& circle : control.targ) {
        if (!circle.inTheBox) {
            float Xerr = circle.x - middleX;
            errorsX.push_back(Xerr);

            float Yerr = circle.y - middleY;
            errorsY.push_back(Yerr);

            std::cout << "YERR : " << Yerr << " XERR: " << Xerr << "\n";
        }
        circleRadius = circle.width;
    }
    // Controlling
    // X part

    if (errorsX.size() != 0) {
        // Calculate average X error and normalize it to the [-1, 1] by dividing to the max error
        float summError = 0, avError;
        size_t numError = errorsX.size();
        for (const auto& error : errorsX) {
            summError += error;
            ++numError;
        }
        avError = (summError / numError) / control.box.left;

        /*  It's also possible to change the PID coefficient with
            the size of the target to make drone more accurate.

            Not used, optionally. Also should be added in the Y part.
            control.pid.kP = std::max((float)0.05, (320 - control.targ[0].width) / 1800);
            */

        // Calculate necessary acceleration (velocity indeed, but it requires double PID)
        float acc = control.pid.calculate(avError, 0);

        // Send the command to the drone (the command changes drone's tilt)
        msg.linear.y = -acc;
    }

    // The Y part is similar to the X part.
    if (errorsY.size() != 0) {
        float summError = 0, avError;
        size_t numError = errorsY.size();
        for (const auto& error : errorsY) {
            summError += error;
            ++numError;
        }
        avError = (summError / numError) / control.box.top;
        float vel = control.pid.calculate(avError, 1);
        if (control.isBottomCamera) {
            msg.linear.x = -vel;
        } else {
            msg.linear.z = std::min(1.0, std::max(-1.0, -vel * 10.0));
        }
    }

    if (circleRadius >= 1.0) {
        if (control.isBottomCamera) {
            float vel = control.pid.calculate(std::max(-1.0, std::min(1.0, ((circleRadius - (float)BC_IDEAL_RADIUS) / (float)BC_MAXIMUM_RADIUS) * 5.0)), 2);
            msg.linear.z = std::min(1.0, std::max(-1.0, vel * 10.0));
        } else {
            float vel = control.pid.calculate(std::max(-1.0, std::min(1.0, ((circleRadius - (float)FC_IDEAL_RADIUS) / (float)FC_MAXIMUM_RADIUS) * 5.0)), 2);
            msg.linear.x = -vel;
        }
    }

    // Output sent values to the console.
    std::cout << "Command sent!\n";
    std::cout << "Vx -> " << msg.linear.y << '\n';
    std::cout << "Vy -> " << msg.linear.x << '\n';
}

// Recieve information and runs the function that controls the drone.
void onTarget(const std_msgs::Float32MultiArray& msg) {
    if (control.enabled) {
        // Runs every 0.06 ms that approximately corresponds to every 2 frame
        // This value can be changed
        geometry_msgs::Twist message;

        if ((ros::Time::now() - control.lastLoop).toSec() >= 0.06) {
            control.targ.clear();
            // Handle received information.
            parseArray(msg);

            // Calculate delta time.
            control.pid.dt = (ros::Time::now() - control.lastLoop).toSec();
            control.lastLoop = ros::Time::now();
            if (control.pid.dt > 1.0) {
                return;
            }
            // Run the controller.
            controller(message);

            // Circle and box information output
            std::cout << "-----------------------\n";
            std::cout << "BOX LEFT: " << control.box.left << '\n';
            std::cout << "BOX RIGHT: " << control.box.right << '\n';
            std::cout << "BOX TOP: " << control.box.top << '\n';
            std::cout << "BOX BOTTOM: " << control.box.bottom << '\n' << '\n';
            std::cout << "-----------------------\n";
            for (auto& circle : control.targ) {
                std::cout << circle << '\n';
            }
            std::cout << "-----------------------\n";

            // Send the message.
            control.cmdPublisher.publish(message);
        }
    } else {
        // Reset saved information if the controller was turned off
        control.pid.integralX = 0;
        control.pid.integralY = 0;
        control.pid.prevErrorX = 0;
        control.pid.prevErrorY = 0;
    }
}

// Extract box information from incoming message.
void onBox(const std_msgs::Float32MultiArray& msg) {
    control.box.left = msg.data[0];
    control.box.right = msg.data[1];
    control.box.top = msg.data[2];
    control.box.bottom = msg.data[3];
    control.imgRows = msg.data[4];
    control.imgCols = msg.data[5];
}


// Increase or decrease PID coefficients using keyboard
void onPidI(const std_msgs::String& str) {
    switch (str.data[0]) {
        case 'p': control.pid.kP -= 0.0005; // 'F2'
                  break;
        case 'i': control.pid.kI -= 0.0005; // 'F4'
                  break;
        case 'd': control.pid.kD -= 0.0005; // 'F6'
                  break;
    }
}

void onPidD(const std_msgs::String& str) {
    switch (str.data[0]) {
        case 'p': control.pid.kP += 0.005; // 'F1'
                  break;
        case 'i': control.pid.kI += 0.001; // 'F3'
                  break;
        case 'd': control.pid.kD += 0.005; // 'F5'
                  break;
    }
}


// Let different functions (i.e. image processing) know which camera is used
void onBottomCamera(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    control.isBottomCamera = true;
}

void onFrontCamera(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    control.isBottomCamera = false;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle node;

    ros::Subscriber boxSub =
        node.subscribe("box", 1, onBox);

    ros::Subscriber enableSub =
        node.subscribe("controller/enable", 5, onEnableCtrl);

    ros::Subscriber targetSub =
        node.subscribe("target", 5, onTarget);

    ros::Subscriber pidDecreaseSub =
        node.subscribe("pid/decrease", 5, onPidD);

    ros::Subscriber pidIncreaseSub =
        node.subscribe("pid/increase", 5, onPidI);

    ros::Subscriber bottomCameraSub =
        node.subscribe("ardrone/bottom/camera_info", 5, onBottomCamera);

    ros::Subscriber frontCameraSub =
        node.subscribe("ardrone/front/camera_info", 5, onFrontCamera);

    control.cmdPublisher =
        node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::spin();

    return 0;
}
