#include <stdio.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <track.h>
#include <vars.h>

int check=0, im_check=0;
cv::Mat fr, depth;
pcl::PointCloud<pcl::PointXYZRGB> tmp_pc;
cv::VideoWriter writer;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    fr = cv_ptr->image;
	    im_check=1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void pointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	fromROSMsg(*msg, tmp_pc);
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	    depth = cv_ptr->image;
	    check=1;
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
	cvNamedWindow("Depth", 1);
	int key = 0;
	static CvScalar red_color[] ={0,0,255};
	CvMat* prevgray = 0, *image = 0, *gray =0;

	
	ros::init(argc, argv, "blob");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	image_transport::ImageTransport it_ = image_transport::ImageTransport(n);
	image_transport::Subscriber image_sub_ = it_.subscribe("camera/rgb/image_color", 1, imageCallback);
	image_transport::Subscriber point_sub_ = it_.subscribe("camera/depth_registered/image_raw", 1, depthCallback);
	ros::Subscriber depth_sub_ = n.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, pointCallback);
	image_transport::Publisher pub = it_.advertise("image_repub", 1);
	ROS_INFO("1");
	while(im_check==0 || check==0)
	{
		ros::spinOnce();
		loop_rate.sleep();
		ROS_INFO("waiting");
	}
	/*writer.open("estimator.avi",CV_FOURCC('x', 'v', 'i', 'd'),15.0,cv::Size(640,480));
	if (!writer.isOpened())
    {
        cout  << "Could not open the output video for write: " << endl;
        return -1;
    }*/
	while(ros::ok())
	{
		//track();
		//kmns();
		//handseg();
		bagseg();
		cvWaitKey(20);
		loop_rate.sleep();
		ros::spinOnce();
	}

	cvDestroyWindow("Image");
	cvDestroyWindow("Depth");
	return 0;
}
