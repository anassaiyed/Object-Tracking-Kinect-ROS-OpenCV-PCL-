//*******************surf.cpp******************//
//********** SURF implementation in OpenCV*****//
//**loads video from webcam, grabs frames computes SURF keypoints and descriptors**//  //** and marks them**//

//****author: achu_wilson@rediffmail.com****//

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


//IplImage* fra = NULL;
int check=0, im_check=0,i=1;
cv::Mat fr, depth;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB> tmp_pc;
//pcl::visualization::PCLVisualizer viewer("3D Viewer");

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		//ROS_INFO("I heard something");
		//IplImage buf;
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    fr = cv_ptr->image;
	    im_check=1;
	    //buf = cv_ptr->image;
	    //fra = &buf;
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
	//pcl::visualization::PCLVisualizer viewer("3D Viewer");
	/*if(i==1)
	{
		viewer.addPointCloud(tmp_pc);
		i=2;
	}
	viewer.updatePointCloud(tmp_pc);*/
    //viewer.addCoordinateSystem(1.0f);
    //viewer.addPointCloud(tmp_pc, "point cloud");
	//pcl::io::savePLYFileASCII("test_ply.ply", tmp_pc);//358,52
	//printf("%f\n",tmp_pc[300*70].z);
	//std::cout << tmp_pc.width;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(&tmp_pc);
	//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   	//viewer.showCloud (ptrCloud);
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		//ROS_INFO("I heard something");
		//IplImage buf;
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	    depth = cv_ptr->image;
	    check=1;
	    //buf = cv_ptr->image;
	    //fra1 = &buf;
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
	//CvCapture* capture = cvCreateCameraCapture(0);
	CvMat* prevgray = 0, *image = 0, *gray =0;

	
	ros::init(argc, argv, "blob");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	image_transport::ImageTransport it_ = image_transport::ImageTransport(n);
	//ros::Subscriber sub = n.subscribe("camera/rgb/image_color", 1000, imageCallback);
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
	/*while(key!='q')
	{
		pub.publish(cv_ptr->toImageMsg());
		loop_rate.sleep();
		ros::spinOnce();mask
	}*/

	int no=7;
	cv::Mat hsv,hsv1,mask,mask1,depth_image,gaussian, depth_average[no];
	cv::RNG rng(12345);
	const static int SENSITIVITY_VALUE = 25;
	//size of blur used to smooth the intensity image output from absdiff() function
	const static int BLUR_SIZE = 6;
	int i=0;

	while(ros::ok())
	{
		cv::Mat frame1, frame2, grayImage1, grayImage2, gray, differenceImage, thresholdImage, gaussian1, gaussian2, dst1, dst, final, mean_depth;
		vector<vector<cv::Point> > contours;
		vector<cv::Vec4i> hierarchy;

		//Background subtraction from depth image
		//depth_average[i]=depth;
		//mean_depth=(depth_average[0]+depth_average[1]+depth_average[2]+depth_average[3]+depth_average[4])/5;
		//i++;
		//if(i>4) i=0;
		//depth_image=mean_depth;
		depth_image=depth;
		cv::Scalar tempVal = mean(depth_image);
		float depth_mean = tempVal.val[0];
		//cout << depth_mean << "\n";
		cv::inRange(depth_image, 1, depth_mean+300, mask);
		//cv::blur(mask,mask,cv::Size(10,10));
		//cv::threshold(mask,mask,130,255,cv::THRESH_BINARY);
		//cv::bitwise_and(depth, mask, dst, mask);
		depth_average[i]=mask.clone();
		//mean_depth=depth_average[0];
		mean_depth = depth_average[0].clone();
		for(int j=1;j<no;j++)
		{
			mean_depth = mean_depth + depth_average[j];
		}
		mean_depth = mean_depth / no;
		i++;
		if(i>no-1)
		{
			i=0;
		}
		cv::threshold(mean_depth,mean_depth,16,255,cv::THRESH_BINARY);


		//read first frame
		frame1=fr;
		im_check=0;
		//convert frame1 to gray scale for frame differencing
		GaussianBlur( frame1, gaussian1, cv::Size( 3, 3 ), 0, 0 );
		cv::cvtColor(gaussian1,grayImage1,CV_BGR2GRAY);
		//copy second frame
		while(im_check==0 && ros::ok())
		{
			ros::spinOnce();
			//loop_rate.sleep();
			//printf("waiting 1\n");
		}
		frame2=fr;
		//convert frame2 to gray scale for frame differencing
		GaussianBlur( frame2, gaussian2, cv::Size( 3, 3 ), 0, 0 );
		cv::cvtColor(gaussian2,grayImage2,CV_BGR2GRAY);
		//perform frame differencing with the sequential images. This will output an "intensity image"
		//do not confuse this with a threshold image, we will need to perform thresholding afterwards.
		cv::absdiff(grayImage1,grayImage2,differenceImage);
		//threshold intensity image at a given sensitivity value
		cv::threshold(differenceImage,thresholdImage,SENSITIVITY_VALUE,255,cv::THRESH_BINARY);
		//show the difference image and threshold image
		//cv::imshow("Image",differenceImage);
		//cv::imshow("Threshold Image", thresholdImage);
		//blur the image to get rid of the noise. This will output an intensity image
		cv::blur(thresholdImage,thresholdImage,cv::Size(BLUR_SIZE,BLUR_SIZE));
		//threshold again to obtain binary image from blur output
		cv::threshold(thresholdImage,thresholdImage,SENSITIVITY_VALUE,255,cv::THRESH_BINARY);

		cv::cvtColor(gaussian1, hsv1, CV_BGR2HSV);
		cv::inRange(hsv1, cv::Scalar(0, 20, 20), cv::Scalar(25, 255, 255), mask1);
		cv::blur(mask1,mask1,cv::Size(3,3));
		cv::threshold(mask1,mask1,130,255,cv::THRESH_BINARY);

		//cv::bitwise_and(thresholdImage, mask1, dst1, mask1);
		cv::bitwise_and(mean_depth, mask1, dst1, mean_depth);


		cv::bitwise_and(gaussian1, gaussian1, final, dst1);

		cv::cvtColor(final, gray, CV_BGR2GRAY);
		cv::findContours( gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		vector<vector<cv::Point> > contours_poly( contours.size() );
		vector<cv::Rect> boundRect( contours.size() );
		vector<cv::Moments> ContArea(contours.size());
		for( int i = 0; i < contours.size(); i++ )
		{
			cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
			boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
			ContArea[i] = cv::moments(contours[i], false);
		}

		/// Draw polygonal contour + bonding rects + circles
		cv::Mat drawing = cv::Mat::zeros( gray.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ )
		{
			//cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			//cv::drawContours( drawing, contours_poly, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
       		if(ContArea[i].m00 > 1500)
       		{
				cv::rectangle( frame1, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0,255,255), 2, 8, 0 );
				cv::Point center = cv::Point(boundRect[i].x+(boundRect[i].width/2), boundRect[i].y+(boundRect[i].height/2));

				//cout<<"Depth: "<<depth_image.at<cv::Vec2b>(center.y,center.x)<<endl;
				cv::rectangle( frame1, cvPoint(center.x-2/2,center.y-2/2), cvPoint(center.x+2/2,center.y+2/2/2), cv::Scalar(0,255,255), 2, 8, 0 );
				std::cout << "Depth x: "<< tmp_pc[(center.y*tmp_pc.width)+center.x].x << endl;//(center.y*tmp_pc.width)+center.x
				std::cout << "Depth y: "<< tmp_pc[(center.y*tmp_pc.width)+center.x].y << endl;
				std::cout << "Depth z: "<< tmp_pc[(center.y*tmp_pc.width)+center.x].z << endl;

			}
		}

		//show our captured frame
		cv::imshow("mask1",mean_depth);
		cv::imshow("Depth",mask1);
		//cv::imshow("Depth",thresholdImage);
		cv::imshow("Image",mask);
		cv::imshow("Final Blob",frame1);
		cvWaitKey(30);

		ros::spinOnce();
		loop_rate.sleep();
	}

	/*while (ros::ok())
	{
		cv::Mat dst, dst1, gray, canny_output;
		cv::Mat grad_x, grad_y, grad_x1;
  		cv::Mat abs_grad_x, abs_grad_y, abs_grad_x1;

  		vector<vector<cv::Point> > contours;
		vector<cv::Vec4i> hierarchy;

		depth.convertTo(depth_image, CV_8U);
		GaussianBlur( fr, gaussian, cv::Size( 3, 3 ), 0, 0 );

		//cv::cvtColor(fr, hsv, CV_BGR2HSV);
		cv::cvtColor(gaussian, hsv1, CV_BGR2HSV);
		//cv::inRange(hsv, cv::Scalar(100, 120, 180), cv::Scalar(115, 255, 255), mask);
		cv::inRange(hsv1, cv::Scalar(110, 80, 80), cv::Scalar(120, 255, 255), mask1);
		//cv::bitwise_and(fr, fr, dst, mask);

		cv::Sobel( depth_image, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  		cv::convertScaleAbs( grad_x, abs_grad_x );

  		cv::Sobel( gaussian, grad_x1, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  		cv::convertScaleAbs( grad_x1, abs_grad_x1 );

		cv::bitwise_and(gaussian, gaussian, dst1, mask1);

		cv::cvtColor(dst1, gray, CV_BGR2GRAY);
		cv::findContours( gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		vector<vector<cv::Point> > contours_poly( contours.size() );
		vector<cv::Rect> boundRect( contours.size() );
		vector<cv::Moments> ContArea(contours.size());
		for( int i = 0; i < contours.size(); i++ )
		{
			cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
			boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
			ContArea[i] = cv::moments(contours[i], false);
		}

		/// Draw polygonal contour + bonding rects + circles
		cv::Mat drawing = cv::Mat::zeros( gray.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ )
		{
			//cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			//cv::drawContours( drawing, contours_poly, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
       		if(ContArea[i].m00 > 30)
       		{
				cv::rectangle( fr, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255,0,0), 2, 8, 0 );
			}
		}

		cv::imshow( "Image", dst1 );
		cv::imshow( "Depth", fr );
		cvWaitKey(30);

		ros::spinOnce();
		loop_rate.sleep();
	}*/
	cvDestroyWindow("Image");
	cvDestroyWindow("Depth");
	return 0;
}
