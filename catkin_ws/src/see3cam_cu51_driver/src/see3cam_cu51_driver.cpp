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

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <unistd.h>
#include <termios.h>

using namespace cv;
using namespace std;

#define CU40	0
#define CU51_12CUNIR	1 //Both CU51 or 12CUNIR

#define ImageWidth	640
#define	ImageHeight 480

char getch(){
    /*#include <unistd.h>   //_getch*/
    /*#include <termios.h>  //_getch*/
    char buf=0;
    struct termios old={0};
    fflush(stdout);
    if(tcgetattr(0, &old)<0)
        perror("tcsetattr()");
    old.c_lflag&=~ICANON;
    old.c_lflag&=~ECHO;
    old.c_cc[VMIN]=1;
    old.c_cc[VTIME]=0;
    if(tcsetattr(0, TCSANOW, &old)<0)
        perror("tcsetattr ICANON");
    if(read(0,&buf,1)<0)
        perror("read()");
    old.c_lflag|=ICANON;
    old.c_lflag|=ECHO;
    if(tcsetattr(0, TCSADRAIN, &old)<0)
        perror ("tcsetattr ~ICANON");
    printf("%c\n",buf);
    return buf;
 }

// Actual Data format BGIRR after conversion BGGR - IR is replaced with the G
//IR data is collected as a separate image
bool ConvertRGIR2RGGB(Mat BayerRGIR, Mat &BayerRGGB, Mat &IRimage)
{
	//Result image after replacing the IR pixel with the G data
	BayerRGGB = BayerRGIR.clone();

	//IR data will be half the size of Bayer Image
	IRimage = Mat(BayerRGIR.size().height / 2, BayerRGIR.size().width / 2, CV_8UC1);

	//copying the IR data and replacing the IR data with G
	for (int Row = 0; Row < BayerRGIR.rows; Row += 2)
	{
		for (int Col = 0; Col < BayerRGIR.cols; Col += 2)
		{
                   //Set the IR Data with Nearby Green
		   BayerRGGB.at<uchar>(Row + 1, Col) = BayerRGIR.at<uchar>(Row, Col + 1);
                   //Set the IR Data
		   IRimage.at<uchar>(Row / 2, Col / 2) = BayerRGIR.at<uchar>(Row + 1, Col);
		}
	}

	return true;
}

// Main Function
int main()
{
	char keyPressed;
	VideoCapture _CameraDevice;
	Mat ResultImage, InputImage;
	Mat BayerFrame8, IRImage, BGRImage;
  cout << "Opending device" << endl;
	//Open the device at the ID 0
	_CameraDevice.open(1);

	if( !_CameraDevice.isOpened()) //Check for the device
	{
		cout << endl << "\tCamera Device not Initialised Successfully" << endl << endl;
		cout << endl << "Press any Key to exit the application" << endl << endl;

		getch();
		return 0;
	}
  cout << "Setting width" << endl;
	//Set up the width and height of the camera
	_CameraDevice.set(CV_CAP_PROP_FRAME_WIDTH,  ImageWidth);
	_CameraDevice.set(CV_CAP_PROP_FRAME_HEIGHT, ImageHeight);

	cout << endl << "Press 'Q / q /Esc' key on the image winodw to exit the application" << endl << endl;

	while(1)
	{
		_CameraDevice >> InputImage; //Read the input image

		if(InputImage.empty()) //Check for the vlid image
		{
			cout << "No frame grabbed!!, check whether the camera is free!!" << endl << endl;
			break;
		}

#if CU51_12CUNIR

		//Convert to 8 Bit:
                //Scale the 12 Bit (4096) Pixels into 8 Bit(255) (255/4096)= 0.06226
		convertScaleAbs(InputImage, ResultImage, 0.06226);

		namedWindow("Y16 to Y8", WINDOW_AUTOSIZE);
		imshow("Y16 to Y8", ResultImage);

#elif CU40
	        //Convert to 8 Bit:
                //Scale the 10 Bit (1024) Pixels into 8 Bit(255) (255/1024)= 0.249023
		convertScaleAbs(InputImage, BayerFrame8, 0.249023);

		//Filling the missing G -channel bayer data
		ConvertRGIR2RGGB(BayerFrame8, BayerFrame8, IRImage);

		//Actual Bayer format BG but Opencv uses BGR & Not RGB So taking RG Bayer format
		cvtColor(BayerFrame8, BGRImage, COLOR_BayerRG2BGR);

		namedWindow("Camera BGR Frame", WINDOW_AUTOSIZE);
		imshow("Camera BGR Frame", BGRImage);

		namedWindow("Camera IR Frame", WINDOW_AUTOSIZE);
		imshow("Camera IR Frame", IRImage);

#else //10CUG and other camera's

		namedWindow("Camera Frame", WINDOW_AUTOSIZE);
		imshow("Camera Frame", InputImage);
#endif

		keyPressed = waitKey(1); //Waits for a user input to quit the application

		if(keyPressed == 27 || keyPressed == 'q'  || keyPressed == 'Q' )
		{
			destroyAllWindows();
			break;
		}
	}

	//Release the devices
	_CameraDevice.release();

	cout << endl << "Press any Key to exit the application" << endl << endl;
	getch();

	return 1;
}
