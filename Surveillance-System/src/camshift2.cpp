#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <ctype.h>
#include <time.h>

#define MAX_DATE 12
using namespace cv;
CvHistogram *hist;
bool marker;
///////////////new parameter/////////////////////
Mat diff1,diff2,motion;
Mat kernel_ero=getStructuringElement(MORPH_RECT,Size(2,2));
Mat prev_mframe,current_mframe,next_mframe,matriximage;
Mat result;
int x_start, x_stop;
int y_start, y_stop;
int min_x, min_y;
int max_x, max_y;
int max_deviation;
int number_of_changes,number_of_sequence=0;
double dWidth, dHeight,fps;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//																																 //
//												capture image here!!!															 //
//																																 //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//VideoCapture cap("rtsp://192.168.1.1/MJPG?W=640H=360Q=1BR=640");  //This is Action Camera
VideoCapture cap(0);
//VideoCapture cap(2); //Webcam
//VideoCapture cap("http://192.168.1.2:8080/video?x.mjpeg"); //Ip webcam
using namespace std;

class Tracker {
	
public:
Tracker(void)
{
	//check if camera worked
	if(!cap.isOpened())
	{
		cout<<"cannot open the Video cam"<<endl;
	}
	cout<<"camera is opening"<<endl;
	
	dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	fps = cap.get(CV_CAP_PROP_FPS);
	
	cout<<"Frame size:"<<dWidth<<"x"<<dHeight<<"--fps"<<fps<<endl;
	
	cap>>prev_mframe;
	cvtColor(prev_mframe,prev_mframe,CV_RGB2GRAY);  // capture 3 frame and convert to grayscale
	cap>>current_mframe;
	cvtColor(current_mframe,current_mframe,CV_RGB2GRAY);  
	cap>>next_mframe;
	cvtColor(next_mframe,next_mframe,CV_RGB2GRAY);  
	
	// number_of_changes is the amount of changes in the resut matrix...
	number_of_changes = 0;
	
	// Detect motion in window...
	x_start = 10, x_stop= dWidth-10;
	y_start = 10, y_stop = dHeight-10;
	
	// Maximum deviation in the image, the  higher the value, the more motion is allowed...
	max_deviation = 20;
	
}

~Tracker(void)
{
	cout<<"shutting down camera and closing files"<<endl;
	cap.release();
	destroyAllWindows();
}

Mat	captureImg(void)
{
        
		cap>>matriximage;
		result=matriximage;
		cvtColor(matriximage,matriximage,CV_RGB2GRAY);  //grayscale
			
		// Calculate differences between 3 consecutive frames...
		diffImg(prev_mframe, current_mframe, next_mframe);
		imshow("Motion Indicator", result);		// Display the current frame...

	
		//rellocate image in right order
		current_mframe.copyTo(prev_mframe);
		next_mframe.copyTo(current_mframe);
		matriximage.copyTo(next_mframe);
		
		motion = diffImg(prev_mframe, current_mframe, next_mframe);
		return result;
}

Mat diffImg(Mat prev_mframe,Mat current_mframe,Mat next_mframe)
{	
		absdiff(prev_mframe,next_mframe,diff1);
		absdiff(next_mframe,current_mframe,diff2);
		bitwise_and(diff1,diff2,motion);
		imshow("motion", motion);		// Display the current frame...
		imshow("diff1",diff1);
		imshow("diff2",diff2);
		threshold(motion,motion,35,255,CV_THRESH_BINARY);
		erode(motion,motion,kernel_ero);
		return motion;
	
}
	
int detectMotion (void)
{
	Scalar mean, stddev;
	meanStdDev(motion,mean,stddev);

		if(stddev[0] < max_deviation)
	{
		number_of_changes = 0;
		min_x = motion.cols, max_x=0;
		min_y = motion.rows, max_y=0;
		
		for(int j=y_start; j < y_stop; j+=2)  {		
			for(int i=x_start; i < x_stop; i+=2)  {	
				
				if(static_cast<int>(motion.at<uchar>(j,i)) == 255)
				{
					number_of_changes++;
					if(min_x>i) min_x = i;
					if(max_x<i) max_x = i;
					if(min_y>j) min_y = j;
					if(max_y<j) max_y = j;
				}
			}
		}
		if(number_of_changes) 
		{
			
			if(min_x-10 > 0) min_x -= 10;
			if(min_y-10 > 0) min_y -= 10;
			if(max_x+10 < matriximage.cols-1) max_x += 10;
			if(max_y+10 < matriximage.rows-1) max_y += 10;
			
			Point x(min_x,min_y);
			Point y(max_x,max_y);
			Rect rect(x,y);
			rectangle(result,rect,Scalar(0,255,255),1,4);
			imshow("Motion Indicator", result);
		}
		return number_of_changes;	
	}		
	 			
	return 0;
  
}

	
inline bool saveImg(Mat oneFrame)
{
	const string DIRECTORY = "/home/sky/Detect/"; // directory where the images will be stored
    const string EXTENSION = ".jpg"; // extension of the images
  
    // Format of directory
    string DIR_FORMAT = "%d%h%Y"; // 1Jan1970
    string FILE_FORMAT = DIR_FORMAT + "_" + "%H%M%S"; // 1Jan1970/1Jan1970_12153
	stringstream ss;
    time_t seconds;
    struct tm * timeinfo;
    char TIME[80];
    time (&seconds);
    // Get the current time
    timeinfo = localtime (&seconds);
	
    // Create name for the image
    strftime (TIME,80,FILE_FORMAT.c_str(),timeinfo);
    ss.str("");	
	ss << DIRECTORY << TIME << EXTENSION;
	
	cout<<ss.str().c_str()<<endl;
	return imwrite(ss.str().c_str(),oneFrame);
	
}

int closeImg(void)
{	
	return 0;
}


protected:
    char** args;

};

int main(int argc, char** argv)
{
	
	Tracker cam1;
	int number_of_changes = 0;
	//const int frameBuffer = 50;
	const int lengthThreshold = 5;
	int frameSequence=0;
	ros::init(argc,argv,"openCV_sender");
	ros::NodeHandle n;
	ros::Publisher chatter_publisher = n.advertise<std_msgs::String>("medium_man",1000);
	ros::Rate loop_rate(1.0);

	int there_is_motion = 5;
	int saveImg = 0;
	Mat frame;
	int count = 1;	
	//Endless lopp to capture image
	while (1)
	{
		
		
		frame = cam1.captureImg();
		number_of_changes=cam1.detectMotion();
	
		if(number_of_changes>=there_is_motion)
		{
			cout<<"Motion detected!!"<<endl;
			frameSequence++;
			
			if(frameSequence > lengthThreshold)
			{
				saveImg = 1;
				
				if (saveImg == 1)
		{
			
			std_msgs::String msg;
			std::stringstream ss;
			ss<<"motion been detetcted "<<count;
			msg.data = ss.str();
			
			saveImg = 0;
			frameSequence=0;
			cout<<"saving Img"<<endl;
			cam1.saveImg(frame);		
			
			ROS_INFO("[openCV_sender] This is %s ",msg.data.c_str());
			chatter_publisher.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
			count++;
			
		}
			}
		}	
		else
		{
			frameSequence=0;
		}
		
	 	
		
		if (waitKey(30) == 27)
		{
			cap.release();
			destroyAllWindows();
			cout<<"turning off"<<endl;
			break;
		}
		
		
	}
	
	return 0;
}
