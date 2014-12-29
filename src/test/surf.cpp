//*******************surf.cpp******************//
//********** SURF implementation in OpenCV*****//
//**loads video from webcam, grabs frames computes SURF keypoints and descriptors**//  //** and marks them**//

//****author: achu_wilson@rediffmail.com****//

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
//Included because SURF is part of nonfree
#include <opencv2/nonfree/nonfree.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

IplImage* fr = NULL;
cv_bridge::CvImagePtr cv_ptr;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		//ROS_INFO("I heard something");
		IplImage buf;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    buf = cv_ptr->image;
	    fr = &buf;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

using namespace std;
int main(int argc, char** argv)
{
	CvMemStorage* storage = cvCreateMemStorage(0);
	cvNamedWindow("Image", 1);
	int key = 0;
	static CvScalar red_color[] ={0,0,255};
	//CvCapture* capture = cvCreateCameraCapture(0);
	CvMat* prevgray = 0, *image = 0, *gray =0;

	//For the nonfree problem
	cv::initModule_nonfree();

	
	ros::init(argc, argv, "surf");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	image_transport::ImageTransport it_ = image_transport::ImageTransport(n);
	//ros::Subscriber sub = n.subscribe("camera/rgb/image_color", 1000, imageCallback);
	image_transport::Subscriber image_sub_ = it_.subscribe("camera/rgb/image_color", 1, imageCallback);
	image_transport::Publisher pub = it_.advertise("image_repub", 1);
	ROS_INFO("1");
	while(fr==NULL)
	{
		ros::spinOnce();
		loop_rate.sleep();
		ROS_INFO("waiting");
	}
	/*while(key!='q')
	{
		pub.publish(cv_ptr->toImageMsg());
		loop_rate.sleep();
		ros::spinOnce();
	}*/

	while (ros::ok())
	{
		int firstFrame = gray == 0;
		//IplImage* frame = cvQueryFrame(capture);
		ros::spinOnce();
		IplImage* frame = new IplImage(cv_ptr->image);
		while(!frame)
		{
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("waiting");
			IplImage* frame = fr;
			//break;
		}
		if(!gray)
		{
			image = cvCreateMat(frame->height, frame->width, CV_8UC1);
		}
		//Convert the RGB image obtained from camera into Grayscale
		cvCvtColor(frame, image, CV_BGR2GRAY);
		//Define sequence for storing surf keypoints and descriptors
		CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
		int i;

		//Extract SURF points by initializing parameters
		CvSURFParams params = cvSURFParams(500, 1);
		cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );
		ROS_INFO("Image Descriptors: %d\n", imageDescriptors->total);

		//draw the keypoints on the captured frame
		for( i = 0; i < imageKeypoints->total; i++ )
		{
			CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, i );
			CvPoint center;
			int radius;
			center.x = cvRound(r->pt.x);
			center.y = cvRound(r->pt.y);
			radius = cvRound(r->size*1.2/9.*2);
			cvCircle( frame, center, radius, red_color[0], 1, 8, 0 );
		}
		cvShowImage( "Image", frame );

		cvWaitKey(30);
		ROS_INFO("2");
		loop_rate.sleep();
	}
	cvDestroyWindow("Image");
	return 0;
}
