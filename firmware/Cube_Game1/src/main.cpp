#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#define USE_PI 1
#if USE_PI
#include <serial/serial.h>
#include <wiringSerial.h>
#endif
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
std::string video_num_path("../video_num.txt");
std::string writer_path("../video/");
int video_num=0;

#if USE_PI
serial::Serial serial_port("/dev/ttyS0",115200 , serial::Timeout::simpleTimeout(1000));
#endif
#define MARKER_SIZE 0.05
std::vector< aruco::Marker > Markers;
Point3f Target_Pix,Target_Pos,Target_Att;
int Target_check_num=0;


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


void uartSent()
{  unsigned char data_to_send[50];
    int _cnt = 0, i = 0, sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = Target_check_num;

    data_to_send[_cnt++] = int(Target_Pix.x)>>8;
    data_to_send[_cnt++] = int(Target_Pix.x)%256;
    data_to_send[_cnt++] = int(Target_Pix.y)>>8;
    data_to_send[_cnt++] = int(Target_Pix.y)%256;
    data_to_send[_cnt++] = int(Target_Pix.z)>>8;
    data_to_send[_cnt++] = int(Target_Pix.z)%256;
    
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
    #if USE_PI
    serial_port.write(data_to_send, Length);
    #endif
}

void getAttitudea(aruco::Marker marker, Point3f &attitude)
{
    double pos[3] = { 0 };
    double ori[4] = { 0 };

    double q0, q1, q2, q3;

    marker.OgreGetPoseParameters(pos, ori);
    pos[0] = -pos[0];
    pos[1] = -pos[1];

    q0 = ori[0]; q1 = ori[1]; q2 = ori[2]; q3 = ori[3];

    attitude.x = atan2(2 * (q0 * q1 + q2 * q3), -1 + 2 * (q1 * q1 + q2 * q2)) *57.3f;
    attitude.y = asin(2 * (q1 * q3 - q0 * q2)) *57.3f;
    attitude.z = -atan2(2 * (-q1 * q2 - q0 * q3), 1 - 2 * (q0 * q0 + q1 * q1)) *57.3f;
}

void getCameraPosa(cv::Mat Rvec, cv::Mat Tvec, cv::Point3f &pos)
{
    vector<Eigen::Vector3d> landmarks_pointXYZ;
	Mat Rot(3, 3, CV_32FC1);
    Rodrigues(Rvec, Rot);
    Eigen::Matrix3d eigen_r;
    cv2eigen(Rot,eigen_r);
    Rot = Rot.t();  // rotation of inverse
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(eigen_r);
    T = angle.inverse();
    Eigen::Matrix<double,3,1> t;
    cv::cv2eigen(Tvec, t);
    t = -1 * angle.inverse().matrix() *t;
    T(0, 3) = t(0);
    T(1, 3) = t(1);
    T(2, 3) = t(2);
    pos.x = t(0)*100;
    pos.y = -t(1)*100;
    pos.z = t(2)*100;
 }


int Mark_Need_Find[3]={1,5,7};
int mark_find_cnt=0;
int main()
{

    #if USE_PI
        while((fd = serialOpen ("/dev/ttyS0",115200))<0)
        {
            cout<<"serial err"<<endl;
            usleep(2000*1000);
        }
    #endif      
      #if USE_PI
      VideoCapture cap;
      cap.open(0);
     #else
      string video_read_s("../51.avi");
      cout<<"read_video"<<video_read_s<<endl;
      VideoCapture cap(video_read_s);
    #endif

    string cameraParamFileName("../rasp320.yml");
    int W=320,H=240;
    cv::Size InImage_size(W,H);
    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile(cameraParamFileName);
    cout << CamParam.CameraMatrix << endl;
    cout << CamParam.Distorsion << endl;
    MarkerDetector MDetector;
    float MarkerSize = MARKER_SIZE;
    ostringstream ostr_pos;
    ostringstream ostr_att;
    ostringstream ostr_s1,ostr_s2,ostr_s3;
    int state_game=0;
    int mark_find_flag[3],flag;
    float mark_check_cnt[3];
    float mark_check_reset_cnt[3];
    float cnt1=0;
    float time_cost;
    cv::TickMeter tm;
 VideoWriter video_writer;
    while (1)
    {
        tm.reset();
        tm.start();
        Mat frame;
	static float dt;
        char key=0;
	 Point  st,se;
        int mss=H*0.2;
        cap >> frame;
        if(!frame.empty()){
         resize(frame, frame, InImage_size);
        }
        else{
         cout<<"Camera Fail!!"<<endl;
         return 0;
        }
        
	switch(state_game)
	{
	  case 0:
	        ostr_pos.clear();
		ostr_pos.str("");
		ostr_pos << "Press Space to Start !";
		putText(frame, ostr_pos.str(), Point(W/2-W*0.35,H/2), 
		CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
		
	     key = (char)waitKey(50);
	      if(key==' ')
	      { state_game++;cnt1=0;time_cost=mark_find_flag[0]=mark_find_flag[1]=mark_find_flag[2]=0;
		mark_check_cnt[0]=mark_check_cnt[1]=mark_check_cnt[2]=0;
		mark_check_reset_cnt[0]=mark_check_reset_cnt[1]=mark_check_reset_cnt[2]=0;
 		video_writer.release();
          	startWriteVideo(frame,video_num_read, video_writer);
     		flag=1;
         	cout<<"Write_video at: "<<video_num<<endl;

	      }
	  break;
	  case 1:

	        ostr_pos.clear();
		ostr_pos.str("");
		ostr_pos << "--Game Start--" ;
		putText(frame, ostr_pos.str(), Point(W/2-W*0.3,H/2), 
		CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
	        cnt1+=dt;
		if(cnt1>1.5)  { state_game++;cnt1=0;}
	  break;
	  case 2:
	    	ostr_pos.clear();
		ostr_pos.str("");
		ostr_pos << "--Find Cube--" ;
		putText(frame, ostr_pos.str(), Point(W/2-W*0.3,H/2-H*0.2), 
		CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
	   
		ostr_att.clear();
		ostr_att.str("");
		ostr_att << Mark_Need_Find[0]<<"  -  "<<Mark_Need_Find[1]<<"  -  "<<Mark_Need_Find[2];
		putText(frame, ostr_att.str(), Point(W/2-W*0.3,H/2+H*0.05), 
		CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
		
		 cnt1+=dt;
		if(cnt1>2)  { state_game++;cnt1=0;}
	  break;
	  case 3:
	     ostr_pos.clear();
	     ostr_pos.str("");
	     ostr_pos << "Time: "<<(int)time_cost<<" s" ;
	     putText(frame, ostr_pos.str(), Point(W*0.025,H*0.1), 
             CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
	      
	    
	     //1
	     st.x=W/2-W*0.4;
	     st.y=H/2+H*0.2;
	     if(mark_find_flag[0])
	     rectangle( frame, st,st+Point(mss,mss) , Scalar(0, 255, 0), 6, 4, 0 );   
	     else  
	     rectangle( frame, st,st+Point(mss,mss) , Scalar(0, 0, 255), 2, 4, 0 ); 
	     ostr_s1.clear();
	     ostr_s1.str("");
	     ostr_s1 <<Mark_Need_Find[0] ;
	     putText(frame, ostr_s1.str(), Point(st.x+mss*0.35,st.y+mss*0.55), 
             CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
	     
	     //2
	     st.x=W/2-W*0.4+mss*2;
	     if(mark_find_flag[1])
	     rectangle( frame, st,st+Point(mss,mss) , Scalar(0, 255, 0), 6, 4, 0 );   
	     else  
	     rectangle( frame, st,st+Point(mss,mss) , Scalar(0, 0, 255), 2, 4, 0 ); 
	     ostr_s2.clear();
	     ostr_s2.str("");
	     ostr_s2 <<Mark_Need_Find[1] ;
	     putText(frame, ostr_s2.str(), Point(st.x+mss*0.35,st.y+mss*0.55), 
             CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
	     
	     //3
	     st.x=W/2-W*0.4+mss*2*2;
	     if(mark_find_flag[2])
	     rectangle( frame, st,st+Point(mss,mss) , Scalar(0, 255, 0), 6, 4, 0 );   
	     else  
	     rectangle( frame, st,st+Point(mss,mss) , Scalar(0, 0, 255), 2, 4, 0 ); 
	     ostr_s3.clear();
	     ostr_s3.str("");
	     ostr_s3 <<Mark_Need_Find[2] ;
	     putText(frame, ostr_s3.str(), Point(st.x+mss*0.35,st.y+mss*0.55), 
             CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
	     
	    MDetector.detect(frame, Markers, CamParam, MarkerSize);
	    
	      for (unsigned int i = 0; i < Markers.size(); i++)
	    {
                  int id=Markers[i].id;
                  Markers[i].draw(frame, Scalar(0, 0, 255), 4);
                  Point2f center=Markers[i].getCenter() ;
		  
		  for(int j=0;j<3;j++){
		    if(abs(center.x-W/2)+abs(center.y-H/2)<H*0.25&&id==Mark_Need_Find[j]&&mark_find_flag[j]==0){
		       mark_check_cnt[j]+=dt;
		       mark_check_reset_cnt[j]=0;
		  
		       if(mark_check_cnt[j]>1)
			 mark_find_flag[j]=1;
		       circle(frame,center,mark_check_cnt[j]*60,Scalar(255,0,255),4);
		     }
		  }
	    }
	    
	    for(int i=0;i<3;i++)
	    {
	      mark_check_reset_cnt[i]+=dt;
	      if(mark_check_reset_cnt[i]>2)
	         mark_check_cnt[i]=0;
	    }
	    time_cost+=dt;
	    
	    if(mark_find_flag[0]&&mark_find_flag[1]&&mark_find_flag[2])
	       state_game++;
	  break;
	  case 4:
	        ostr_pos.clear();
		ostr_pos.str("");
		ostr_pos << "--Game Over--" ;
		putText(frame, ostr_pos.str(), Point(W/2-W*0.3,H/2-H*0.2), 
		CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
	   
		ostr_att.clear();
		ostr_att.str("");
		ostr_att << "Time: "<<time_cost;
		putText(frame, ostr_att.str(), Point(W/2-W*0.3,H/2+H*0.05), 
		CV_FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(55, 255, 0), 2);
		cnt1+=dt;
		if(cnt1>2&&flag==1)
			flag=2;
	  break;
	}
        
        
	if(flag==1)
	video_writer << frame;
	else if(flag==2)
{ 	video_writer.release();
           cout<<"Write Stop!!"<<endl;
flag=3;
}	

        imshow("Marker", frame);
        uartSent();
        char c = (char)waitKey(20);
        if( c == 27 )
            break;
	else if(state_game>0&&c == 'r')
	   state_game=0;
	tm.stop();
	dt=tm.getTimeMilli()/1000.;
	//cout<<"Dt:"<<tm.getTimeMilli()<<"ms"<<endl;
    }
    video_writer.release();
    cout << "video_writer.release();" << endl;
}
