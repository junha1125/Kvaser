//autoware_msgs라는 패키지에 있는 k7이라는 해더파일을 가져온다. 이 해더파일에는 
//typedef struct k7{header ~ int ~ float ~ float ~ } k7;과 같은 구조체 정의가 하나 있을 뿐이다. 
#include "autoware_msgs/K7.h"
#include "autoware_msgs/Mobileye.h"
#include "autoware_msgs/Niro.h"

#include <ros/ros.h>
#include "autoware_can_msgs/CANPacket.h"
#include "can_msgs/Frame.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>					// 몇가지 상수(e pi와 같은 값)들을 저장해 놓은 STL 해더파일
//#include <iostream>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>


// 각각의 topic에 정보를 저장하기 위해 필요한 변수를 선언하였다. 왼쪽이 구조체이름. 오른쪽이 변수 이름.
// 여기에 변화된 데이터를 저장해, 마지막에 Pulish맴버함수의 매개변수로 넣을 예정이다.
autoware_msgs::K7			k7_dat;
autoware_msgs::Mobileye		        Mobileye_dat;
autoware_msgs::Niro			Niro_dat;

autoware_can_msgs::CANPacket		raw_data;



void chatterCallback(const can_msgs::Frame::ConstPtr& msg)
{
	raw_data.header = msg->header; 
	raw_data.count = 0;
	raw_data.id = msg->id;
	raw_data.len = msg->dlc;
	raw_data.dat = msg->data;
	raw_data.flag = 0;
	raw_data.time = 0;
/*
	if(raw_data.id == 0x669 ||
		raw_data.id ==0x700 ||
		raw_data.id ==0x727 ||
		raw_data.id ==0x728 ||
		raw_data.id ==0x737 ||
		raw_data.id ==0x738 ||
		raw_data.id ==0x760 ||
		raw_data.id ==0x720 ||
		raw_data.id ==0x726 ||
		raw_data.id ==0x739 ||
		raw_data.id ==0x738 ) {

		std::cout << "Mobileye Data is incoming!! :  " ;
		printf("0x%05X  \n ",raw_data.id);
	}


	if(raw_data.id == 0x541 ||
		raw_data.id ==0x553 ||
		raw_data.id ==0x559 ||
		raw_data.id ==0x541 ||
		raw_data.id ==0x340 ||
		raw_data.id ==0x394||
		raw_data.id ==0x372 ||
		raw_data.id ==0x371 ||
		raw_data.id ==0x381 ) {

		std::cout << "Niro Data is incoming!! :  " ;
		printf("0x%05X  \n ",raw_data.id);
	}
*/

}


int main (int argc, char *argv[]){
  autoware_can_msgs::CANPacket candat;						// 사실 이건 필요없음. 선배들이 잘 모르고 넣어놓은 것 같음.

  ros::init(argc, argv, "can_tx_to_raw");					// Node이름이 can_converter이다. 
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("can_tx", 1, chatterCallback);
  ros::Publisher raw_pub = n.advertise<autoware_can_msgs::CANPacket>("can_raw", 1);						
  while(ros::ok()){  
    raw_pub.publish(raw_data);		
    ros::spinOnce();
  }
}
