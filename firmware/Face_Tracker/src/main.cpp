#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <serial/serial.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <wiringSerial.h>


#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
using namespace cv;
using namespace std;
using namespace Eigen;
int fd;
serial::Serial serial_port("/dev/ttyS0",256000 , serial::Timeout::simpleTimeout(100));


Point3f Target_Pos,Target_Att;
int Target_check_num=0;

void uartSent()
{  unsigned char data_to_send[50];
    int _cnt = 0, i = 0, sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;
//map
    data_to_send[_cnt++] = int(Target_check_num)>>8;
    data_to_send[_cnt++] = int(Target_check_num)%256;

    data_to_send[_cnt++] = int(Target_Pos.x*100)>>8;
    data_to_send[_cnt++] = int(Target_Pos.x*100)%256;
    data_to_send[_cnt++] = int(Target_Pos.y*100)>>8;
    data_to_send[_cnt++] = int(Target_Pos.y*100)%256;
    data_to_send[_cnt++] = int(Target_Pos.z*100)>>8;
    data_to_send[_cnt++] = int(Target_Pos.z*100)%256;

    data_to_send[_cnt++] = int(Target_Att.x*100)>>8;
    data_to_send[_cnt++] = int(Target_Att.x*100)%256;
    data_to_send[_cnt++] = int(Target_Att.y*100)>>8;
    data_to_send[_cnt++] = int(Target_Att.y*100)%256;
    data_to_send[_cnt++] = int(Target_Att.z*100)>>8;
    data_to_send[_cnt++] = int(Target_Att.z*100)%256;
    
    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    int Length = _cnt;
    serial_port.write(data_to_send, Length);
}


int main()
{
    VideoCapture cap;
    while(( fd = serialOpen ("/dev/ttyS0",115200))<0)
    {
            cout<<"serial err"<<endl;
            usleep(2000*1000);
    }

    cap.open(0);
    string xmlPath = "/home/pi/QT/Face_Tracker/haarcascade_frontalface_alt2.xml";
	CascadeClassifier ccf;  

    if (!ccf.load(xmlPath))  
	{
		cout << "Load file fail!" << endl;
		return 0;
	}

    while (1)
    {
        Mat frame,gray;
        cv::Size InImage_size(320,240);
        cap >> frame;
        if(!frame.empty()){
         resize(frame, frame, InImage_size);
         //flip(frame, frame, -1);
        }
        else{
         cout<<"Camera Fail!!"<<endl;
         return 0;
        }
        
        vector<Rect> faces;  

        if (frame.channels() == 3)
        {
            cvtColor(frame, gray, CV_BGR2GRAY);
        }
        else{
            gray = frame;
        }
        equalizeHist(gray, gray); 
        ccf.detectMultiScale(gray, faces, 1.1, 3, 0); 
        for (vector<Rect>::const_iterator iter = faces.begin(); iter != faces.end(); iter++)
        {
            rectangle(frame, *iter, Scalar(0, 0, 255), 2, 8); 
             Point  center;  
             int radius;  
            // center.x = cvRound((faces[iter].x + faces[iter].width * 0.5));  
            // center.y = cvRound((faces[iter].y + faces[iter].height * 0.5));  

        }
        imshow("faces", frame);

        uartSent();
        char c = (char)waitKey(20);
        if( c == 27 )
            break;
    }
}
