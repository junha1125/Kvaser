/*this node has a role converting / can_tx(real kvaser data) into / can_raw(important datas among these).*/


#include "autoware_msgs/K7.h"
#include "autoware_msgs/Mobileye.h"
#include "autoware_msgs/Niro.h"

#include <ros/ros.h>
#include "autoware_can_msgs/CANPacket.h"
#include "can_msgs/Frame.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>					
//#include <iostream>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>



autoware_msgs::K7			k7_dat;
autoware_msgs::Mobileye	    Mobileye_dat;
autoware_msgs::Niro			Niro_dat;

autoware_can_msgs::CANPacket		raw_data;



// this function subscrive can_tx Topic, Save important data on can_raw Topic And publish can_raw Topic .  
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

// Make Node, Topic
int main (int argc, char *argv[]){
  autoware_can_msgs::CANPacket candat;						

  ros::init(argc, argv, "can_tx_to_raw");					
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("can_tx", 1, chatterCallback);
  ros::Publisher raw_pub = n.advertise<autoware_can_msgs::CANPacket>("can_raw", 1);						
  while(ros::ok()){  
    raw_pub.publish(raw_data);		
    ros::spinOnce();
  }
}
