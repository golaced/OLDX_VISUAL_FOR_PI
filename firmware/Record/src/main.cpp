#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <serial/serial.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <wiringSerial.h>


#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
using namespace cv;
using namespace std;
using namespace aruco;
using namespace Eigen;
int fd;


std::ifstream video_num_read;
std::ofstream video_num_write;
std::string video_num_path("/home/pi/QT/Record/video_num.txt");
std::string writer_path("/home/pi/QT/Record/Video/");
int video_num=0;

string intToString(int v)
{
    char buf[32] = {0};
    snprintf(buf, sizeof(buf), "%u", v);

    string str = buf;
    return str;
}

void startWriteVideo(Mat in,std::ifstream &video_num_read,
		cv::VideoWriter &video_writer)
{
   std::stringstream ss;
   string video_name;
	video_num_read.open(video_num_path.c_str());
	video_num_read >> video_num;
	video_num_read.close();
	if (video_num_write.is_open())
		video_num_write.close();
        video_num=video_num+1;
	video_num_write.open(video_num_path.c_str());
	video_num_write << (video_num);
	video_num_write.close();

	if (video_writer.isOpened())
	{
		video_writer.release();
	}

	ss << video_num;
	ss >> video_name;
	video_name += ".avi";
       cout << "Write Video: "<<video_name << endl;
	video_writer.open(
			writer_path + video_name,
            CV_FOURCC('X', 'V', 'I', 'D'), 25, Size(320,240));
            
    if(!video_writer.isOpened())
    {
 	cout<< "Error : fail to open video writer\n"<<endl;

    }
}

int main()
{
    VideoCapture cap;
    static int flag,save_video,cnt;
    cv::Size InImage_size(320,240);
    cap.open(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    VideoWriter video_writer;

    while (1)
    {
        Mat frame;
        cap >> frame;
        if(!frame.empty()){
         resize(frame, frame, InImage_size);
         //flip(frame, frame, -1);
        }
        else{
         cout<<"Camera Fail!!"<<endl;
         break;
        }
        
        
       if (save_video&&!flag)
       {
           video_writer.release();
           startWriteVideo(frame,video_num_read, video_writer);
           flag = 1;
           cout<<"Write_video at: "<<video_num<<endl;
       }
       if(flag){
           cout<<"Frame: "<<cnt++<<endl;
	       video_writer << frame;
       }


        imshow("Marker", frame);
        char c = (char)waitKey(20);
        if( c == 27 )
            break;
        else if(c==' '&&save_video==0)
           save_video=1;
        else if(c==' '&&save_video==1)
        {
           flag=save_video=cnt=0;
           video_writer.release();
           cout<<"Write Stop!!"<<endl;
        }
    } 
    video_writer.release();
    cout << "video_writer.release();" << endl;
}
