
/*
   imgHandler.cpp
   ----------------------------------------------------------------------------
   | Receives the image from the drone and sends it to the compVision.cpp,    |
   | that processes it and extracts information about the circles and the box.|
   |                                                                          |
   | Then receives processed information and sends it to the controller.      |
   ----------------------------------------------------------------------------

*/


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgHelper.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point32.h>
#include "constants.h"

// Initialize global objects to access them easily
struct ImageHandler imgHandler;
static CirclesMessage cMessage;


// Convert CircleMessage to Float32MultiArray (information about the box)
// This allows using a static box.
void cmsg2BoxMultiArray(CirclesMessage& cmsg, std_msgs::Float32MultiArray& boxToSend) {
    std::vector<float> vec1 = {
        cmsg.box.left,
        cmsg.box.right,
        cmsg.box.top,
        cmsg.box.bottom,
        imgHandler.imgRows, imgHandler.imgCols
    };

    boxToSend.data.insert(boxToSend.data.end(), vec1.begin(), vec1.end());
}


// Convert CircleMessage to Float32MultiArray (information about circles)
void cmsg2MultiArray(CirclesMessage& cmsg, std_msgs::Float32MultiArray& msg) {
    std::vector<float> vec1;
    for (size_t i = 0; i != cmsg.inTheBox.size(); ++i) {
        vec1.clear();
        vec1 = {
            cmsg.circles[i].center.x,
            cmsg.circles[i].center.y,
            cmsg.circles[i].size.width,
            cmsg.circles[i].size.height,
            cmsg.inTheBox[i],
            cmsg.rotError
        };

        msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());
    }
}



// Get and process the image from camera.
// This function runs when the image has been received.
void onImage(const sensor_msgs::Image::ConstPtr& image)
{
    // Convert the image to the OpenCV format.
    // Copied from ROS wiki's cv_bridge tutorial - goo.gl/o4zV41 (4)

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        try {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_8UC3);
        }
        catch	 (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    // Image processing
    if (imgHandler.cv_enabled) {

        // Process the image and get circle information to cMessage
        // The image is sent to the compVision.cpp
        processImage(cv_ptr->image, cMessage, imgHandler);

        // Convert information from compVision.cpp

        // It's possible to recieve box information only once
        //  using a bool flag in order to get static box

        std_msgs::Float32MultiArray msg;
        std_msgs::Float32MultiArray sendBox;

        cmsg2BoxMultiArray(cMessage, sendBox);
        cmsg2MultiArray(cMessage, msg);

        imgHandler.imgRows = cv_ptr->image.rows;
        imgHandler.imgCols = cv_ptr->image.cols;


        // Publish messages to the ROS topics
        imgHandler.circlePublisher.publish(msg);
        imgHandler.boxSendler.publish(sendBox);
    }

    // Publish the image to the ROS topic
    imgHandler.imgPublisher.publish(cv_ptr->toImageMsg());

}


// Get camera information.
void onCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    imgHandler.cameraDistortion = cam_info->D;
    imgHandler.cameraMatrix = cam_info->K;
}


// Toggle image processing by pressing button 'M'.
void onEnable(const std_msgs::Empty& toggle_msg) {
    imgHandler.cv_enabled = !imgHandler.cv_enabled;
    if (imgHandler.cv_enabled) {
        std::cout << "Image processing enabled.\n";
    } else {
        std::cout << "Image processing disabled.\n";
    }
}


// Change some parameters depending on which camera is used
void onBottomCamera(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    imgHandler.minRadius = BC_IDEAL_RADIUS;
    imgHandler.threshold = BC_THRESHOLD;
    imgHandler.maxRadius = BC_MAXIMUM_RADIUS;
    imgHandler.blurCoeff = BC_BLUR_COEFF;
}

void onFrontCamera(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    imgHandler.minRadius = FC_IDEAL_RADIUS;
    imgHandler.threshold = FC_THRESHOLD;
    imgHandler.maxRadius = FC_MAXIMUM_RADIUS;
    imgHandler.blurCoeff = FC_BLUR_COEFF;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "imgproc");
    ros::NodeHandle node;

    // Button signal subscriber

    ros::Subscriber enableSub =
        node.subscribe("cv/enable", 1, onEnable);

    // Information publishers

    imgHandler.imgPublisher =
        node.advertise<sensor_msgs::Image>("/out/image", 5);

    imgHandler.boxSendler =
        node.advertise<std_msgs::Float32MultiArray>("box", 1);

    imgHandler.circlePublisher =
        node.advertise<std_msgs::Float32MultiArray>("target", 5);


    // Camera subscribers

    ros::Subscriber sub1 =
        node.subscribe("/in/image", 5, onImage);
    ros::Subscriber sub2 =
        node.subscribe("/ardrone/image_raw", 5, onImage);
    ros::Subscriber sub3 =
        node.subscribe("/ardrone/camera_info", 5, onCameraInfo);
    ros::Subscriber bottomCameraSub =
        node.subscribe("ardrone/bottom/camera_info", 5, onBottomCamera);
    ros::Subscriber frontCameraSub =
        node.subscribe("ardrone/front/camera_info", 5, onFrontCamera);

    ros::spin();

    return 0;
}
