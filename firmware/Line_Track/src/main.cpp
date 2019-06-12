
#include <iostream>
#include <sstream>
#include<opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#define USE_PI 1
#if USE_PI
#include <serial/serial.h>
#include <wiringSerial.h>
#endif
using namespace std;
using namespace cv;
#if USE_PI
int fd=0;
serial::Serial serial_port("/dev/ttyS0",115200 , serial::Timeout::simpleTimeout(1000));
#endif
std::ifstream video_num_read;

Point2f Line_center;
float      Line_angle;
int         Line_num;
void uartSent()
{  unsigned char data_to_send[50];
    int _cnt = 0, i = 0, sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x04;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = Line_num;

    data_to_send[_cnt++] = int(Line_center.x)>>8;
    data_to_send[_cnt++] = int(Line_center.x)%256;
    data_to_send[_cnt++] = int(Line_center.y)>>8;
    data_to_send[_cnt++] = int(Line_center.y)%256;
  
    data_to_send[_cnt++] = int(Line_angle*100)>>8;
    data_to_send[_cnt++] = int(Line_angle*100)%256;

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    int Length = _cnt;
#if USE_PI
    serial_port.write(data_to_send, Length);
#endif
}


void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle)
{
    CV_Assert(!inputIm.empty());
    Mat inputImg;
    inputIm.copyTo(inputImg);
    float radian = (float) (angle / 180.0 * CV_PI);
    int uniSize = (int) (max(inputImg.cols, inputImg.rows) * 1.414);
    int dx = (int) (uniSize - inputImg.cols) / 2;
    int dy = (int) (uniSize - inputImg.rows) / 2;
    copyMakeBorder(inputImg, tempImg, dy, dy, dx, dx, BORDER_CONSTANT);
    Point2f center((float) (tempImg.cols / 2), (float) (tempImg.rows / 2));
    Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);
    warpAffine(tempImg, tempImg, affine_matrix, tempImg.size());
    float sinVal = fabs(sin(radian));
    float cosVal = fabs(cos(radian));
    Size targetSize((int) (inputImg.cols * cosVal + inputImg.rows * sinVal),
            (int) (inputImg.cols * sinVal + inputImg.rows * cosVal));
    int x = (tempImg.cols - targetSize.width) / 2;
    int y = (tempImg.rows - targetSize.height) / 2;
    Rect rect(x, y, targetSize.width, targetSize.height);
    tempImg = Mat(tempImg, rect);
}

void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat,int Ds, float T)
{
    // accept only char type matrices
    CV_Assert(!inputMat.empty());
    CV_Assert(inputMat.depth() == CV_8U);
    CV_Assert(inputMat.channels() == 1);
    CV_Assert(!outputMat.empty());
    CV_Assert(outputMat.depth() == CV_8U);
    CV_Assert(outputMat.channels() == 1);

    // rows -> height -> y
    int nRows = inputMat.rows;
    // cols -> width -> x
    int nCols = inputMat.cols;

    // create the integral image
    cv::Mat sumMat;
    cv::integral(inputMat, sumMat);

    CV_Assert(sumMat.depth() == CV_32S);
    CV_Assert(sizeof(int) == 4);

    int S = MAX(nRows, nCols)/Ds;

    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;

    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;

    for( int i = 0; i < nRows; ++i)
    {
        y1 = i-s2;
        y2 = i+s2;

        if (y1 < 0){
            y1 = 0;
        }
        if (y2 >= nRows) {
            y2 = nRows-1;
        }

        p_y1 = sumMat.ptr<int>(y1);
        p_y2 = sumMat.ptr<int>(y2);
        p_inputMat = inputMat.ptr<uchar>(i);
        p_outputMat = outputMat.ptr<uchar>(i);

        for ( int j = 0; j < nCols; ++j)
        {
            // set the SxS region
            x1 = j-s2;
            x2 = j+s2;

            if (x1 < 0) {
                x1 = 0;
            }
            if (x2 >= nCols) {
                x2 = nCols-1;
            }

            count = (x2-x1)*(y2-y1);

            // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];

            if ((int)(p_inputMat[j] * count) < (int)(sum*(1.0-T)))
                p_outputMat[j] = 0;
            else
                p_outputMat[j] = 255;
        }
    }
}



void thread_line(void)
{

    Mat InImage;
    cout<<"CV_VERSION"<<CV_VERSION<<endl;
    int SW=320,SH=240;
    cv::Size InImage_size(SW,SH);
    ostringstream ostr_pos;
    ostringstream ostr_angle;
   
    cv::TickMeter tm,tm_uart;
    #if USE_PI
       VideoCapture cap1(0);
   #else
      string video_read_s("/home/exbot/SLAM/Line_Tracker/video_r/6.avi");
      cout<<"read_video"<<video_read_s<<endl;
      VideoCapture cap1(video_read_s);
   #endif

    
    int iLowH = 0;
    int iHighH = 176;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 25;
    int Min_RectS=15,		Min_RectW=88,		TOP_good_rate=0.66*100;
    cvNamedWindow("Line Result",CV_WINDOW_AUTOSIZE);
    createTrackbar("Min_RectS", "Line Result", &Min_RectS, 30);
    createTrackbar("Min_RectW", "Line Result", &Min_RectW, SW/2);
    createTrackbar("TOP_good_rate", "Line Result", &TOP_good_rate, 100);
    
    namedWindow("Control", CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    for(;;)
    {
        int bad_frame=0;
        tm.reset();
        tm.start();
        cap1>>InImage;
        if(!InImage.empty())
        cv::resize(InImage, InImage, InImage_size);
        else
        bad_frame=1;
   if(!bad_frame){    
        int DEAD_W=35;
	float H_SIZE=0.5;
	Rect rect_bottom1(DEAD_W,SH*H_SIZE,    		SW-DEAD_W*2,SH*H_SIZE);
	Mat ROI;
	InImage(rect_bottom1).copyTo(ROI); 
	//imshow("Origin", InImage); 
	//imshow("In", ROI); 
	Line_center.x=Line_center.y=0;
	Line_angle=0;
	Line_num=0;
	
        Mat imgHSV;
        vector<Mat> hsvSplit;
        cvtColor(ROI, imgHSV, COLOR_BGR2HSV);
        split(imgHSV, hsvSplit);
        equalizeHist(hsvSplit[2],hsvSplit[2]);
        merge(hsvSplit,imgHSV);
        Mat imgThresholded;
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
        morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
	int Bolder_W;
        Rect bolder(0,0,    		SW-DEAD_W*2,SH*H_SIZE);
        rectangle(imgThresholded,bolder, Scalar(0, 0, 0), 3);
        imshow("Control", imgThresholded); 
	
	
	//寻找最外层轮廓  
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

	Mat Draw = Mat::zeros(imgThresholded.size(), CV_8UC1); //最小外接矩形画布  
	//cout<<"contours.size():  "<<contours.size()<<endl;
	for (int i = 0; i<contours.size(); i++)
	{
		//绘制轮廓  
		drawContours(Draw, contours, i, Scalar(255), 1, 8, hierarchy);
		//绘制轮廓的最小外结矩形  
		RotatedRect rect = minAreaRect(contours[i]);
		Point2f center=rect.center;
		float angle=rect.angle;
		Size2f size=rect.size;
		
		//Point2f P1[4];rect.points(P1);for (int j = 0; j <= 3; j++)line(Draw, P1[j], P1[(j + 1) % 4], Scalar(155), 1);
		      
		if(size.area()/SH>Min_RectS&&(size.width<Min_RectW||size.height<Min_RectW)){
		
		  Point2f P[4];
		  rect.points(P);
		  
		  Point2f Top,Mid,Bot;
		  if(size.height>size.width){
		  Top=Point2f((P[1].x+P[2].x)/2,(P[1].y+P[2].y)/2);
		  Mid=center;
		  Bot=Point2f((P[0].x+P[3].x)/2,(P[0].y+P[3].y)/2);
		  }else
		  {
		  angle=90+angle;
		  Top=Point2f((P[2].x+P[3].x)/2,(P[2].y+P[3].y)/2);
		  Mid=center;
		  Bot=Point2f((P[1].x+P[0].x)/2,(P[1].y+P[0].y)/2);
		  }
		  //cout<<size.width<<" "<<size.height<<"  "<<angle<<endl;
		  if(fabs(Top.y)<SH*H_SIZE*TOP_good_rate/100.&&
		     fabs(Bot.y)>SH*H_SIZE*(100-TOP_good_rate)/100.){
		      cout<<"Find_Line: "<<"Size: "<<size.area()/SH<<"  Angle: "<<angle<<endl;
		      circle(Draw, Top, 5,Scalar(255,255,0),-1);	
		      circle(Draw, Mid, 3,Scalar(111),-1);
		      circle(Draw, Bot, 10,Scalar(255,255,0),-1);
		      for (int j = 0; j <= 3; j++)
		      {
			      line(Draw, P[j], P[(j + 1) % 4], Scalar(111), 2);
		      } 
		      
		      line(ROI, Mid,Bot, Scalar(125,0,255), 2);
		      circle(ROI, Mid, 6,Scalar(0,255,255),-1);
		      circle(ROI, Bot, 3,Scalar(255,255,0),-1);
		      
		      Line_center=Mid;
		      Line_angle=angle;
		      Line_num++;    
		  }
		}
	}
	//imshow("Line Result", Draw);
	imshow("In", ROI);       
        tm_uart.reset();tm_uart.start();
        uartSent();
        tm_uart.stop();
   
	static float time;
        time+=(float)tm.getTimeMilli()/1000.;
}//bad_frame
else{
  cout<<"bad_frame"<<endl;
}
        char c = (char)waitKey(20);
      if(c == 27)  
            break;
    }
}


int main(void)
{
    try
    {
#if USE_PI
        while((fd = serialOpen ("/dev/ttyS0",115200))<0)
        {
            cout<<"serial err"<<endl;
            usleep(2000*1000);
        }
#endif       
        thread_line();
        return 0;
    }
    catch (std::exception &ex)
    {
        cout << "Exception :" << ex.what() << endl;
    }
}
