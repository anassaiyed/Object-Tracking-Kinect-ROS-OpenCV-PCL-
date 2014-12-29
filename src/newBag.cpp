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



int keys[26][2]={116, 305,
153, 305,
192, 305,
228, 305,
265, 305,
303, 305,
340, 305,
379, 305,
416, 305,
451, 305,
125, 342,
163, 342,
200, 342,
237, 342,
274, 342,
311, 342,
349, 342,
386, 342,
474, 342,
145, 380,
183, 380,
220, 380,
258, 380,
295, 380,
333, 380,
370, 380
};
char key[26]={'q','w','e','r','t','y','u','i','o','p','a','s','d','f','g','h','j','k','l','z','x','c','v','b','n','m'};

//#include <opencv2\imgproc\imgproc.hpp>

//#include <opencv2\video\background_segm.hpp>
using namespace std;
using namespace cv;
int IndexOfBiggestContour;

int handseg()
{
	Mat mask1;
	inRange(depth, Scalar(1000), Scalar(2000), mask1);
	cvtColor ( mask1, mask1, CV_GRAY2BGR );
	imshow("depth",mask1);
}

void mouseTo(int x,int y)
{
	 Display *display = XOpenDisplay(0);
	  Window root = DefaultRootWindow(display);
	  XWarpPointer(display, None, root, 0, 0, 0, 0, x, y);
	  XFlush(display);
	  XCloseDisplay(display);
}

char findkey(Point current)
{
	int i,j;
	float dist,min_dist=1000.0;
	char pressed_key='1';
	for(i=0;i<26;i++)
	{
		dist = sqrt ( pow(current.x - keys[i][0] ,2) +
					  pow(current.y - keys[i][1] ,2) );
		if (dist<min_dist)
		{
			min_dist=dist;
			pressed_key=key[i];
		}
	}
	if (min_dist<45)
		return pressed_key;
	else
		return 0;
}

/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * @param  im    Binary image with range = 0-1
 * @param  iter  0=even, 1=odd
 */
/*void thinningIteration(cv::Mat& im, int iter)
{
    cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

    for (int i = 1; i < im.rows-1; i++)
    {
        for (int j = 1; j < im.cols-1; j++)
        {
            uchar p2 = im.at<uchar>(i-1, j);
            uchar p3 = im.at<uchar>(i-1, j+1);
            uchar p4 = im.at<uchar>(i, j+1);
            uchar p5 = im.at<uchar>(i+1, j+1);
            uchar p6 = im.at<uchar>(i+1, j);
            uchar p7 = im.at<uchar>(i+1, j-1);
            uchar p8 = im.at<uchar>(i, j-1);
            uchar p9 = im.at<uchar>(i-1, j-1);

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + 
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) + 
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i,j) = 1;
        }
    }

    im &= ~marker;
}*/

/**
 * Function for thinning the given binary image
 *
 * @param  im  Binary image with range = 0-255
 */
/*void thinning(cv::Mat& im)
{
    im /= 255;

    cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(im, 0);
        thinningIteration(im, 1);
        cv::absdiff(im, prev, diff);
        im.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    im *= 255;
}*/

Mat depth_initial, depth_subtracted,mask1,mask,mask2,mask3,hsv1,range;
int initial_count=0;
pcl::PointCloud<pcl::PointXYZRGB> pc;
void bagseg()
{
	int i,j;
	RNG rng(12345);
	double minVal,maxVal;
	Point minLoc,maxLoc;

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
			depth_initial = depth_initial + depth;
			initial_count++;
			if(initial_count==10)
			{
				depth_initial = depth_initial/initial_count;
				pc=tmp_pc;
				initial_count++;
			}
		}
	}
	else
	{
		//inRange(depth, Scalar(0), Scalar(2000), depth);
		mask3=depth.clone();
		inRange(depth,Scalar(1),Scalar(1200),mask3);
		mask3.convertTo(mask3,CV_8U);
		//cvtColor ( mask3, mask3, CV_GRAY2BGR );
		//erode(mask3, mask3, Mat());
		dilate(mask3, mask3, getStructuringElement(MORPH_RECT, Size(33*2+1, 33*2+1), Point(33, 33) ));
		threshold(mask3,mask3,1,255,THRESH_BINARY);

		//blur( depth, depth, Size(3,3) );
		depth_subtracted=depth_initial-depth;
		inRange(depth_subtracted, Scalar(10), Scalar(1300), mask1);

		//Create white image with black borders 50 pixels thick to use as a mask
		uchar* p;
		Mat abc=Mat::zeros(480,640,CV_16UC1);
	    for( i = 0; i < abc.rows; ++i)
	    {
	    	//if(i>200 && i<440)
	    	if(i>50 && i<430)
	    	{
		        p = abc.ptr<uchar>(i);
		        for ( j = 0; j < abc.cols; ++j)
		        {
		        	//if(2*j>100 && 2*j<1140)
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
		int n=3;
		Mat element1 = getStructuringElement(MORPH_RECT, Size(n*2+1, n*2+1), Point(n, n) );
	    //erode(mask2, mask2, Mat());	//erode(abc, abc, element);
	    dilate(mask2, mask2, element1);

		//remove border areas of image and then filter by color
		bitwise_and(mask1, abc, abc);
		bitwise_and(abc, mask2, abc);
		n=4;
		element1 = getStructuringElement(MORPH_RECT, Size(n*2+1, n*2+1), Point(n, n) );
		erode(abc, abc, element1);

	    //bitwise_and(mask3, abc, abc);
	    //bitwise_and(abc, mask1, abc);

	    Mat dist_transform;
	    distanceTransform(abc,dist_transform,CV_DIST_L2 ,3);
	    normalize(dist_transform,dist_transform,0,1.,NORM_MINMAX);
	    minMaxLoc(dist_transform,&minVal,&maxVal,&minLoc,&maxLoc, Mat());
	    //blur(dist_transform, dist_transform, Size(3,3));
	    erode(dist_transform, dist_transform, Mat());

		Mat src1 = imread("a.png");
		double alpha = 0.7; double beta;
		beta = 1.0 - alpha;
 		addWeighted( fr, alpha, src1, beta, 0.0, fr);
 		
		//find contours
	    mask=abc.clone();
	    std::vector<std::vector<Point> > contours;
		std::vector<Vec4i> hierarchy;
	    findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

		Mat drawing = Mat::zeros( mask1.size(), CV_8UC3 );
		//circle(dist_transform,maxLoc,5,Scalar(0,255,255),-1);
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
	                	float dist=sqrt(pow(contours[i][convDef[i][k][0]].x-contours[i][convDef[i][k][1]].x,2)+pow(contours[i][convDef[i][k][0]].y-contours[i][convDef[i][k][1]].y,2));
	                	//cout<<dist<<endl;
	                    if(convDef[i][k][3]>35*256 && dist<90) // filter defects by depth and distance between start and end points (originally convDef[i][k][3]>20*256)
	                    {
		                    int ind_0=convDef[i][k][0];
		                    int ind_1=convDef[i][k][1];
		                    int ind_2=convDef[i][k][2];
		                    defect_points[i].push_back(contours[i][ind_2]);
		                    cv::circle(drawing,contours[i][ind_0],5,Scalar(255,0,0),-1);

		                    Point center=contours[i][ind_0];
		                    int neighFlag=1;
		                    int countFing=0;
		                    int pixDist;
		                    int minPixDist=100;
		                    Point markNew;
		                    int diam=7;
		                    for(int xi=-diam;xi<=diam;xi++)
		                    {
		                    	for(int xj=-diam;xj<=diam;xj++)
		                    	{
		                    		Point current=Point(center.x+xi,center.y+xj);
		                    		for(int yi=-2;yi<=2;yi++)
		                    		{
		                    			for(int yj=-2;yj<=2;yj++)
		                    			{
		                    				Point neighbour=Point(current.x+yi,current.y+yj);
		                    				int valPix=int( abc.at<uchar>(neighbour.y,neighbour.x));
		                    				if(valPix!=255)
		                    				{
		                    					neighFlag=0;
		                    				}

		                    			}
		                    		}

		                    		if((neighFlag==1)&&(countFing==0))
		                    		{
		                    			pixDist = sqrt( pow(center.x - current.x ,2) + pow(center.y - current.y ,2) );
		                    			if( pixDist < minPixDist )	//dist less than 4 pixels
		                    			{
		                    				minPixDist = pixDist;
		                    				//countFing++;
		                    				markNew=current;
		                    			}
		                    			//cout << "hi" << endl;
		                    		}

		                    		neighFlag=1;
		                    	}
		                    }
		                    /*int max_pix=0;
		                    for(int xi=-15;xi<=15;xi++)
		                    {
		                    	for(int xj=-15;xj<=15;xj++)
		                    	{
		                    		Point current=Point(center.x+xi,center.y+xj);
		                    		/*for(int yi=-8;yi<=-8;yi++)
		                    		{
		                    			for(int yj=-8;yj<=8;yj++)
		                    			{
		                    				Point neighbour=Point(current.x+yi,current.y+yj);
		                    				int valPix=int( abc.at<uchar>(neighbour.y,neighbour.x));
		                    				if(valPix!=255)
		                    				{
		                    					neighFlag=0;
		                    				}

		                    			}
		                    		} //end comment

		                    		int valPix=int( dist_transform.at<uchar>(current.x,current.y));
                    				if(valPix>max_pix)
                    				{
                    					max_pix=valPix;
                    					markNew=current;
                    					cout << max_pix << endl;
                    				}
		                    	}
		                    }*/




		                    cv::circle(fr,markNew,5,Scalar(0,255,255),-1);
		                    //cv::circle(dist_transform,markNew,3,Scalar(255,255,255),-1);
		                    //cv::circle(dist_transform,center,2,Scalar(255,255,255),-1);
		                    cv::circle(abc,center,2,Scalar(255,255,0),-1);
		                    //rectangle( fr, Point(center.x-diam,center.y-diam), Point(center.x+diam,center.y+diam), Scalar(255,255,255), 1, 8, 0 );

		                    /*cout << "Depth x: "<< tmp_pc[(contours[i][ind_0].y*tmp_pc.width)+contours[i][ind_0].x].x << endl;
							cout << "Depth y: "<< tmp_pc[(contours[i][ind_0].y*tmp_pc.width)+contours[i][ind_0].x].y << endl;
							cout << "Depth z: "<< tmp_pc[(contours[i][ind_0].y*tmp_pc.width)+contours[i][ind_0].x].z << endl;*/

		                    cv::circle(drawing,contours[i][ind_1],5,Scalar(0,255,0),-1);
		                    cv::circle(drawing,contours[i][ind_2],5,Scalar(0,0,255),-1);
		                    cv::line(fr,contours[i][ind_2],contours[i][ind_0],Scalar(0,0,255),3);
		                    cv::line(fr,contours[i][ind_2],contours[i][ind_1],Scalar(0,0,255),3);

		                    cv::line(drawing,contours[i][ind_2],contours[i][ind_0],Scalar(0,0,255),1);
		                    cv::line(drawing,contours[i][ind_2],contours[i][ind_1],Scalar(0,0,255),1);

		                    //mouseTo(contours[i][ind_0].x,contours[i][ind_0].y);
		                    int o=markNew.x;
		                    int p=markNew.y;
		                    int min_l,min_m;
		                    float min_dist=100;

		                    //cv::circle(fr,Point(320,240),5,Scalar(255,255,0),-1);

		                    //cout << "x: " << pc[(p*pc.width)+o].x << "\tx_tmp: " << tmp_pc[(p*pc.width)+o].x << endl;
							//cout << "y: " << pc[(p*pc.width)+o].y << "\ty_tmp: " << tmp_pc[(p*pc.width)+o].y << endl;
							//cout << "z: " << pc[(markNew.y*pc.width)+markNew.x].z << "\tz_tmp: " << tmp_pc[(markNew.y*pc.width)+markNew.x].z << endl;
							//cout << "o: " << o << "p: " << p << endl;
							//cout << "z1: " << pc[(240*pc.width)+320].z << "\t" << tmp_pc[(240*pc.width)+320].z << endl;
							//cout << int( abc.at<uchar>(p,o) );

		                    for(int l=markNew.x-5;l<markNew.x+5;l++)
		                    {
		                    	for(int m=markNew.y-5;m<markNew.y+5;m++)
		                    	{
		                    		if( ((m*pc.width)+l) < (pc.width*pc.height) )
			                    	{
			                    		dist = sqrt ( pow(pc[(m*pc.width)+l].x - tmp_pc[(p*pc.width)+o].x ,2) +
			                    					  pow(pc[(m*pc.width)+l].y - tmp_pc[(p*pc.width)+o].y ,2) +
			                    					  pow(pc[(m*pc.width)+l].z - tmp_pc[(p*pc.width)+o].z ,2) );
			                    		if(dist<min_dist)
			                    		{
			                    			min_dist=dist;
			                    			min_l=l;
			                    			min_m=m;
			                    		}
			                    	}
		                    	}
		                    }
		                    /*cout << "x_n: " << pc[(min_m*pc.width)+min_l].x << endl;
							cout << "y_n: " << pc[(min_m*pc.width)+min_l].y << endl;
							cout << "z_n: " << pc[(min_m*pc.width)+min_l].z << endl;*/
							//cout << "z_n: " << min_dist << endl;
							if(min_dist<0.014)
							{
								//cout << "touch!!\t" << markNew.x << " " << markNew.y << endl;
								cout << findkey(markNew) << endl;
							}
		                    //break;
	                    }
	                }

	                drawContours( drawing, contours, i, Scalar(0,255,0), 1, 8, vector<Vec4i>(), 0, Point() );
	                drawContours( fr, hull_points, i, Scalar(255,0,0), 1, 8, vector<Vec4i>(), 0, Point() );
	            }
	        }
    	}

    	/*Mat depth_500;
    	depth_500=depth.clone();
    	depth_500=depth_500-500;
    	depth_500.convertTo(depth_500,CV_8U);

    	Mat depth_501=depth_initial.clone();
    	depth_501=depth_501-500;
    	depth_501.convertTo(depth_501,CV_8U);
		imshow("depth2",depth_501);
		imshow("depth_500",depth_500);*/
		//thinning(abc);
		imshow("Depth",drawing);
		imshow("Image",abc);
		imshow("Image1",fr);
		imshow("Image2",dist_transform);
		//imshow("hsv1",hsv1);
		//imshow("Image1",depth_500);
		//imshow("Image1",dist_transform);

	}
}
