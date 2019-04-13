    #include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <serial/serial.h>
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
serial::Serial serial_port("/dev/ttyS0",115200 , serial::Timeout::simpleTimeout(1000));


Point3f Target_Pix,Target_Pos,Target_Att;
int Target_check_num=0;

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
    serial_port.write(data_to_send, Length);
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

#define MARKER_SIZE 0.05
#define SET_ID 10
std::vector< aruco::Marker > Markers;
int main()
{
    VideoCapture cap;
    while(( fd = serialOpen ("/dev/ttyS0",115200))<0)
    {
            cout<<"serial err"<<endl;
            usleep(2000*1000);
    }

    cap.open(0);

    string cameraParamFileName("/home/pi/QT/Cube_Tracker/rasp320.yml");
    cv::Size InImage_size(320,240);
    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile(cameraParamFileName);
    cout << CamParam.CameraMatrix << endl;
    cout << CamParam.Distorsion << endl;
    MarkerDetector MDetector;
    float MarkerSize = MARKER_SIZE;
    ostringstream ostr_pos;
    ostringstream ostr_att;
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
         return 0;
        }
        
        Target_check_num=0;
        MDetector.detect(frame, Markers, CamParam, MarkerSize);
        
        for (unsigned int i = 0; i < Markers.size(); i++)
         {
                  int id=Markers[i].id;
                  if(id=SET_ID){
                  Markers[i].draw(frame, Scalar(0, 0, 255), 2);
                  Point2f center=Markers[i].getCenter() ;
                  circle(frame,center,4,Scalar(255,0,255),4);
                  Point3f Pos,Att;
                  getCameraPosa(Markers[i].Rvec,Markers[i].Tvec,Pos);
                  getAttitudea(Markers[i], Att);
                  Target_Pix.x=center.x;
                  Target_Pix.y=center.y;
                  Target_Pix.z=Markers[i].getArea()/100;
                  Target_Pos=Pos;
                  Target_Att=Att;
                  Target_check_num=id;
                  cout<<"ID: "<<id<<" POS: "<<Pos<<" "<<" Att: "<<Att<<endl;
                  cout<<"X: "<<Target_Pix.x<<" Y: "<<Target_Pix.y<<" "<<" S: "<<Target_Pix.z<<endl;
                  cout<<endl;
                  break;
                  }
         }

        if(Target_check_num){
        ostr_pos.clear();
        ostr_pos.str("");
        ostr_pos << "X=" << (int)Target_Pos.x << " Y=" << (int)Target_Pos.y<< " Z=" << (int)Target_Pos.z;
        putText(frame, ostr_pos.str(), Point(Target_Pix.x+20, Target_Pix.y+20), 
        CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);
        ostr_att.clear();
        ostr_att.str("");
        ostr_att << "P=" << (int)Target_Att.x << " R=" << (int)Target_Att.y<< " Y=" << (int)Target_Att.z;
        putText(frame, ostr_att.str(), Point(Target_Pix.x+20, Target_Pix.y-20), 
        CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);
        }
        imshow("Marker", frame);
        uartSent();
        char c = (char)waitKey(20);
        if( c == 27 )
            break;
    }
}
