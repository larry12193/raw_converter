/*******************************************************************************
 * see3cam_cu51_driver.cpp - ROS driver for See3CAM_CU51 12-bit monochrome
 *                           camera
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 * Adapted from econ-Systems OpenCV examples found here,
 * https://www.e-consystems.com/blog/camera/accessing-see3cam-custom-format-with-opencv/
 *
 ******************************************************************************/
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <unistd.h>
#include <termios.h>

using namespace cv;
using namespace std;

int main (int argc, char **argv) {

  ros::init(argc, argv, "see3cam_cu51_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  int id;
  double width, height;
  priv_nh.param<int>("camera_id", id,     1);
  priv_nh.param<double>("height", height, 480);
  priv_nh.param<double>("width",  width,  640);

	VideoCapture _CameraDevice;
	Mat ResultImage, InputImage;

  ROS_INFO("Attempting to open camera at ID %d with %0.0fx%0.0f resolution",id,width,height);
	//Open the device at the ID 0
	_CameraDevice.open(id);
	if( !_CameraDevice.isOpened()) //Check for the device
	{
		ROS_ERROR("No camera found with ID %d", id);
		return -1;
	}

	//Set up the width and height of the camera
	_CameraDevice.set(CV_CAP_PROP_FRAME_WIDTH,  width);
	_CameraDevice.set(CV_CAP_PROP_FRAME_HEIGHT, height);

  // Define image message
  cv_bridge::CvImage out_msg;

  // Initialize publisher
  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/camera/image", 100);

  // Set loop rate
  ros::Rate loop_rate(60);

	while(ros::ok())
	{

    _CameraDevice >> InputImage; //Read the input image

		if(!InputImage.empty()) //Check for the vlid image
		{
      //Convert to 8 Bit:
                  //Scale the 12 Bit (4096) Pixels into 8 Bit(255) (255/4096)= 0.06226
  		convertScaleAbs(InputImage, ResultImage, 0.06226);


      out_msg.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
      out_msg.encoding     = sensor_msgs::image_encodings::MONO8; // Or whatever
      out_msg.image        = ResultImage; // Your cv::Mat

      img_pub.publish(out_msg.toImageMsg());
		}
    ros::spinOnce();
    loop_rate.sleep();
	}

	//Release the devices
	_CameraDevice.release();

	return 1;
}
