#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
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

#include <vars.h>
#include <cmath>

#include <X11/Xlib.h>
#include <X11/Xutil.h>



//#include <opencv2\imgproc\imgproc.hpp>

//#include <opencv2\video\background_segm.hpp>
using namespace std;
using namespace cv;
int IndexOfBiggestContour;

int track()
{
	int no=7;
	Mat hsv,hsv1,mask,mask1,depth_image,gaussian, depth_average[no];
	RNG rng(12345);
	const static int SENSITIVITY_VALUE = 25;
	//size of blur used to smooth the intensity image output from absdiff() function
	const static int BLUR_SIZE = 6;
	int i=0;
	Mat frame1, frame2, grayImage1, grayImage2, gray, differenceImage, thresholdImage, gaussian1, gaussian2, dst1, dst, final, mean_depth;
	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;

	depth_image=depth;
	Scalar tempVal = mean(depth_image);
	float depth_mean = tempVal.val[0];
	//cout << depth_mean << "\n";
	inRange(depth_image, 1, depth_mean+1000, mask);

	depth_average[i]=mask.clone();
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
	threshold(mean_depth,mean_depth,16,255,THRESH_BINARY);


	//read first frame
	frame1=fr;
	im_check=0;
	//convert frame1 to gray scale for frame differencing
	GaussianBlur( frame1, gaussian1, Size( 3, 3 ), 0, 0 );
	cvtColor(gaussian1,grayImage1,CV_BGR2GRAY);
	//copy second frame
	while(im_check==0 && ros::ok())
	{
		ros::spinOnce();
	}
	frame2=fr;
	//convert frame2 to gray scale for frame differencing
	GaussianBlur( frame2, gaussian2, Size( 3, 3 ), 0, 0 );
	cvtColor(gaussian2,grayImage2,CV_BGR2GRAY);
	//perform frame differencing with the sequential images. This will output an "intensity image"
	//do not confuse this with a threshold image, we will need to perform thresholding afterwards.
	absdiff(grayImage1,grayImage2,differenceImage);
	//threshold intensity image at a given sensitivity value
	threshold(differenceImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);
	//show the difference image and threshold image
	//imshow("Image",differenceImage);
	//imshow("Threshold Image", thresholdImage);
	//blur the image to get rid of the noise. This will output an intensity image
	blur(thresholdImage,thresholdImage,Size(BLUR_SIZE,BLUR_SIZE));
	//threshold again to obtain binary image from blur output
	threshold(thresholdImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);

	cvtColor(gaussian1, hsv1, CV_BGR2HSV);
	inRange(hsv1, Scalar(3, 20, 20), Scalar(12, 255, 255), mask1);
	blur(mask1,mask1,Size(3,3));
	threshold(mask1,mask1,130,255,THRESH_BINARY);

	bitwise_and(mean_depth, mask1, dst1, mean_depth);

	bitwise_and(gaussian1, gaussian1, final, dst1);

	cvtColor(final, gray, CV_BGR2GRAY);
	findContours( gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	std::vector<std::vector<Point> > contours_poly( contours.size() );
	std::vector<Rect> boundRect( contours.size() );
	std::vector<Moments> ContArea(contours.size());
	for( int i = 0; i < contours.size(); i++ )
	{
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );
		ContArea[i] = moments(contours[i], false);
	}

	/// Draw polygonal contour + bonding rects + circles
	Mat drawing = Mat::zeros( gray.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
   		if(ContArea[i].m00 > 1000)
   		{
			rectangle( frame1, boundRect[i].tl(), boundRect[i].br(), Scalar(0,255,255), 2, 8, 0 );
			Point center = Point(boundRect[i].x+(boundRect[i].width/2), boundRect[i].y+(boundRect[i].height/2));
			rectangle( frame1, cvPoint(center.x-2/2,center.y-2/2), cvPoint(center.x+2/2,center.y+2/2/2), Scalar(0,255,255), 2, 8, 0 );
			cout << "Depth x: "<< tmp_pc[(center.y*tmp_pc.width)+center.x].x << endl;//(center.y*tmp_pc.width)+center.x
			cout << "Depth y: "<< tmp_pc[(center.y*tmp_pc.width)+center.x].y << endl;
			cout << "Depth z: "<< tmp_pc[(center.y*tmp_pc.width)+center.x].z << endl;

		}
	}

	//show our captured frame
	imshow("mask1",mean_depth);
	imshow("Depth",gray);
	//imshow("Depth",thresholdImage);
	//imshow("Image",mask);
	imshow("Image",final);
	imshow("Final Blob",frame1);
	
	return 0;
}

/*int kmns()
{
	/*std::vector<Mat> imgRGB;
    split(fr,imgRGB);
    int k=5;
    int n = fr.rows *fr.cols;
    Mat img3xN(n,3,CV_8U);
    for(int i=0;i!=3;++i)  
      imgRGB[i].reshape(1,n).copyTo(img3xN.col(i));
    img3xN.convertTo(img3xN,CV_32F);
    //printf("hi\n");
    Mat bestLables;
    kmeans(img3xN,k,bestLables,TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),2,KMEANS_RANDOM_CENTERS );
    //printf("hello\n");
    bestLables= bestLables.reshape(0,fr.rows);
    convertScaleAbs(bestLables,bestLables,int(255/k));
    imshow("result",bestLables);
    //printf("123\n");	//end comment
    Mat grey;
    cvtColor ( fr, grey, CV_BGR2GRAY );
    //Mat temp ;
	//GaussianBlur(grey,temp, Size(0,0), 105) ; //hardcoded filter size, to be tested on 50 mm lens
	//addWeighted(grey, 1.8, temp, -0.8,0,grey) ;
    int board_w = 5, board_h = 8; // Board height and width in number of squares
	int board_n = board_w * board_h;
	std::vector<Point2f> corners;
	bool found = findChessboardCorners( grey, Size(8,5), corners,CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FAST_CHECK + CV_CALIB_CB_NORMALIZE_IMAGE);
	if(found)
	{
		rectangle( grey, cvPoint(corners[2].x-2/2,corners[2].y-2/2), cvPoint(corners[2].x+2/2,corners[2].y+2/2), Scalar(255), 2, 8, 0 );
		cout << "Depth x: "<< tmp_pc[(corners[2].y*tmp_pc.width)+corners[2].x].x << endl;
		cout << "Depth y: "<< tmp_pc[(corners[2].y*tmp_pc.width)+corners[2].x].y << endl;
		cout << "Depth z: "<< tmp_pc[(corners[2].y*tmp_pc.width)+corners[2].x].z << endl;
	}
	//drawChessboardCorners(grey, Size(8,5), Mat(corners), found);
	imshow("result",grey);
    return 0;
}*/
/*int lowThreshold=25;
void CannyThreshold(int, void*)
{
	int kernel_size=3, ratio=3;
	Mat mask1,clon;
	blur( depth, mask1, Size(3,3) );
	mask1=mask1-500;
	blur( mask1, mask1, Size(3,3) );
	mask1.convertTo(mask1, CV_8U);
	//Laplacian(mask1, mask1, CV_8UC1);
	clon=mask1.clone();
	Mat kernel = (Mat_<float>(3,3) << 
        1,  1, 1,
        1, -8, 1,
        1,  1, 1); 
	filter2D(mask1, mask1, -1, kernel);
	mask1=clon+(mask1);
	/// Canny detector
	Canny( mask1, mask1, lowThreshold, lowThreshold*ratio, kernel_size );
	blur(mask1,mask1,Size(5,5));
	threshold(mask1,mask1,50,255,THRESH_BINARY);
	//morphologyEx(mask1,mask1,MORPH_OPEN,getStructuringElement( MORPH_ELLIPSE,Size(7,7)));
	std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours( mask1, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    cvtColor ( mask1, mask1, CV_GRAY2BGR );
    Scalar color( 255, 255, 255 );
    std::vector<Point> approxShape;
    for(int i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxShape, arcLength(Mat(contours[i]), true)*0.04, true);
        drawContours(mask1, contours, i, color, CV_FILLED);   // fill BLUE
    }
	//drawContours( mask1, contours, -1, color,CV_FILLED);
	imshow( "result", mask1 );
}

int handseg()
{
	int max_lowThreshold=100;
	//inRange(depth, Scalar(1000), Scalar(2000), mask1);
	//cvtColor ( mask1, mask1, CV_GRAY2BGR );
	//writer << mask1;
	namedWindow( "result", CV_WINDOW_AUTOSIZE );
	/// Create a Trackbar for user to enter threshold
	createTrackbar( "Min Threshold:", "result", &lowThreshold, max_lowThreshold, CannyThreshold );
	CannyThreshold(0, 0);
	//waitKey(0);
	return 0;
}*/

/*void mouseTo(int x,int y)
{
	 Display *display = XOpenDisplay(0);
	  Window root = DefaultRootWindow(display);
	  XWarpPointer(display, None, root, 0, 0, 0, 0, x, y);
	  XFlush(display);
	  XCloseDisplay(display);
}

Mat depth_initial, depth_subtracted,mask1,mask,mask2,mask3,hsv1,range;
int initial_count=0;
void bagseg()
{
	int i,j;
	RNG rng(12345);
	ros::Rate loop_rate(10);
	if(initial_count<10)
	{
		if(initial_count==0)
		{
			depth_initial=depth.clone();
			//inRange(depth_initial, Scalar(0), Scalar(2000), depth_initial);
			blur( depth_initial, depth_initial, Size(10,10) );
			initial_count++;
		}
		else
		{
			blur( depth, depth, Size(3,3) );
			//inRange(depth, Scalar(0), Scalar(2000), depth);
			depth_initial = depth_initial + depth;
			initial_count++;
			if(initial_count==10)
			{
				depth_initial = depth_initial/initial_count;
			}
		}
	}
	else
	{
		//inRange(depth, Scalar(0), Scalar(2000), depth);
		blur( depth, depth, Size(3,3) );
		depth_subtracted=depth_initial-depth;
		inRange(depth_subtracted, Scalar(5), Scalar(2000), mask1);

		//Create white image with black borders 50 pixels thick to use as a mask
		uchar* p;
		Mat abc=Mat::zeros(480,640,CV_16UC1);
	    for( i = 0; i < abc.rows; ++i)
	    {
	    	if(i>50 && i<430)
	    	{
		        p = abc.ptr<uchar>(i);
		        for ( j = 0; j < abc.cols; ++j)
		        {
		        	if(2*j>100 && 2*j<1180)
		        	{
		        		p[2*j]=255;
		        	}
		        }
		    }
	    }
		abc.convertTo(abc, CV_8U);
		threshold(abc,abc,2, 255,THRESH_BINARY);

		//hand color detection in color image
		cvtColor(fr, hsv1, CV_BGR2HSV);
		inRange(hsv1, Scalar(0, 0, 0), Scalar(100, 255, 255), mask2);
		blur(mask2,mask2,Size(3,3));
		threshold(mask2,mask2,130,255,THRESH_BINARY);

		//remove border areas of image and then filter by color
		bitwise_and(mask1, abc, abc);
		//bitwise_and(abc, mask2, abc);

		int n=1;
		Mat element1 = getStructuringElement(MORPH_RECT, Size(n*2+1, n*2+1), Point(n, n) );
	    erode(abc, abc, Mat());	//erode(abc, abc, element);
	    dilate(abc, abc, element1);
	    //bitwise_and(abc, mask1, abc);

		//find contours
	    mask=abc.clone();
	    std::vector<std::vector<Point> > contours;
		std::vector<Vec4i> hierarchy;
	    findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

		Mat drawing = Mat::zeros( mask1.size(), CV_8UC3 );
	    if(contours.size()>0)
    	{
	        vector<std::vector<int> >hull( contours.size() );
	        vector<vector<Vec4i> > convDef(contours.size() );
	        vector<vector<Point> > hull_points(contours.size());
	        vector<vector<Point> > defect_points(contours.size());

	        for( int i = 0; i < contours.size(); i++ )
	        {
	            if(contourArea(contours[i])>3500)
	            {
	                convexHull( contours[i], hull[i], false );
	                convexityDefects( contours[i],hull[i], convDef[i]);

	                for(int k=0;k<hull[i].size();k++)
	                {           
	                    int ind=hull[i][k];
	                    hull_points[i].push_back(contours[i][ind]);
	                }

	                for(int k=0;k<convDef[i].size();k++)
	                {
	                	// Calculate the distance between two points surrounding convexity defects
	                	int dist=sqrt(pow(contours[i][convDef[i][k][0]].x-contours[i][convDef[i][k][1]].x,2)+pow(contours[i][convDef[i][k][0]].y-contours[i][convDef[i][k][1]].y,2));
	                	cout<<dist<<endl;
	                    if(convDef[i][k][3]>35*256 && dist<90) // filter defects by depth and distance between start and end points (originally convDef[i][k][3]>20*256)
	                    {
		                    int ind_0=convDef[i][k][0];
		                    int ind_1=convDef[i][k][1];
		                    int ind_2=convDef[i][k][2];
		                    defect_points[i].push_back(contours[i][ind_2]);
		                    cv::circle(drawing,contours[i][ind_0],5,Scalar(255,0,0),-1);
		                    cout << "Depth x: "<< tmp_pc[(contours[i][ind_0].y*tmp_pc.width)+contours[i][ind_0].x].x << endl;
							cout << "Depth y: "<< tmp_pc[(contours[i][ind_0].y*tmp_pc.width)+contours[i][ind_0].x].y << endl;
							cout << "Depth z: "<< tmp_pc[(contours[i][ind_0].y*tmp_pc.width)+contours[i][ind_0].x].z << endl;
		                    cv::circle(drawing,contours[i][ind_1],5,Scalar(0,255,0),-1);
		                    //cv::circle(drawing,contours[i][ind_2],5,Scalar(0,0,255),-1);
		                    cv::line(drawing,contours[i][ind_2],contours[i][ind_0],Scalar(0,0,255),1);
		                    cv::line(drawing,contours[i][ind_2],contours[i][ind_1],Scalar(0,0,255),1);
		                    mouseTo(contours[i][ind_2].x,contours[i][ind_2].y);
		                    break;
	                    }
	                }

	                drawContours( drawing, contours, i, Scalar(0,255,0), 1, 8, vector<Vec4i>(), 0, Point() );
	                //drawContours( drawing, hull_points, i, Scalar(255,0,0), 1, 8, vector<Vec4i>(), 0, Point() );
	            }
	        }
    	}

		//imshow("depth2",depth_initial);
		imshow("Depth",drawing);
		imshow("Image",abc);
		//imshow("Image1",element);
		/*mask2=depth.clone();
		mask2=mask2-500;
		mask2.convertTo(mask2, CV_8U);
		mask3=depth_initial.clone();
		mask3=mask3-500;
		mask3.convertTo(mask3, CV_8U);
		imshow("color",fr);
		imshow("hsv",hsv1);
		imshow("depth",mask2);
		imshow("initial",mask3);
		imshow("abc",abc);	//end comment
		//imshow("Image1",hpe);

	}
}
*/