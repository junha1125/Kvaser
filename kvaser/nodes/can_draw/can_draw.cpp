#include <ros/ros.h>
#include "autoware_can_msgs/CANPacket.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// rviz에서 사용하는 패키지로써, 메세지(토픽)들을 시각화하는데 사용한다. 
// http://wiki.ros.org/visualization_msgs


ros::Publisher pub;				// 특이하게 Publisher 객체를 전역변수로 선언했다. 
double g_steer,g_speed;

void chatterCallback(const autoware_can_msgs::CANPacket::ConstPtr& msg)
// 이 함수는 subscribe했던, /can_raw 구조체 형식의 데이터를 매개변수로 받아들인다.
// 받아들인 데이터를 아래와 같이 분석하여, 1. 전역변수로 선언했었던 g_steer, g_speed 값을 바꾸고,
//										2. txt파일을 만들어서 그 내부에 차량 정보를 한줄한줄 써내려간다.
// 즉 아래의 과정은 /can_raw 데이터를 Decoding하는 과정이라고 할 수 있다. 
{
  // 차량의 정보들을 다음과 같은 변수에 저장한다.
  unsigned short w;		// w == temp와 같은 변수
  static int enc_sum;
  short diff;
  static short steer,shift,speed2,speed3,brake;
  static char  speed;
  static short enc,enc_p,enc_diff;
  static short wheel1,wheel2,wheel3,wheel4;
  static short gyro,accy,accz;
  static char accx; 
  // txt파일을 새로 쓰기 위한 File Handler를 선언해준다. 
  static FILE* log_fp;

  // decoding과정 여기서 부터 -------------------------
  int changed=0;   // 차량 상태의 변화가 있는가? 를 저장
  // NI Code에서는 이것을 switch case로 처리했었다.
  // switch(msg->id){  case : 0x24 ~~ case : 0x25   }
  if(msg->id==0x24){ 
    w=msg->dat[0]*256+msg->dat[1];
    gyro=w;
    w=msg->dat[2]*256+msg->dat[3];
    accy=w;
    w=msg->dat[4]*256+msg->dat[5];
    accz=w;
    w=msg->dat[7];
    accx=w;
    changed=1;
  }
  if(msg->id==0x25){
    w=msg->dat[0]*4096+msg->dat[1]*16;
    steer=w;
    steer=steer/16;
    changed=1;
  }

  if(msg->id==0xaa){
    w=msg->dat[0]*256+msg->dat[1];
    wheel1=w;
    w=msg->dat[2]*256+msg->dat[3];
    wheel2=w;
    w=msg->dat[4]*256+msg->dat[5];
    wheel3=w;
    w=msg->dat[6]*256+msg->dat[7];
    wheel4=w;
    changed=1;
  }
  if(msg->id==182){
    w=msg->dat[0]*256+msg->dat[1];
    speed3=w;
    changed=1;
  }
  if(msg->id==0x127){
    shift=msg->dat[3];
    speed=msg->dat[4];
    changed=1;
  }
  if(msg->id==0x230){
    w=msg->dat[0]*256+msg->dat[1];
    enc_p=enc;
    enc=w;
    diff=enc-enc_p;
    enc_diff=diff;
    enc_sum+=diff;
    changed=1;
  }
  // ------여기 까지  -----------------------------

  if(changed){ // 차량 상태 변화가 있었다면
    if(!log_fp)log_fp=fopen("/tmp/can_log","w"); // 다음과 같은 위치에 txt파일을 만들거다!!
    fprintf(log_fp,"%f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
	    msg->header.stamp.toSec(),
		msg->time,steer,
		shift,
		speed,
		speed2,
		speed3,
		enc_sum,
		enc_diff,
		brake,
	    wheel1,
		wheel2,
		wheel3,
		wheel4,
	    accx,
		accy,
		accz,
		gyro);					// 그 txt파일 내부에 한줄 한줄 내용을 기입한다. 

    g_steer=(double)steer/100.;
    g_speed=(double)speed3/360.;
  }  

}


int main (int argc, char *argv[]){
  autoware_can_msgs::CANPacket candat;			// /can_raw와 같은 구조체 형식의 객체를 하나 만들어 놓는다
												// 나중에 이것을 가지고 subscribe할 계획이다.

  ros::init(argc, argv, "can_draw"); // 아하 이것의 topic Name은 can_draw겠구나!!
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("can_raw", 1000, chatterCallback);  // 이것을 사용해서 /can_raw토픽을 읽어들인다.
  // 꼭 callback함수는 이처럼 사용하더라!!!
  //  ros::Subscriber sub객체이름 = NodeHandle객체이름.subscribe맴버함수("subscribe할 topic의 이름", 주기, 함수 포인터 사용하여 callback함수 이름);

  pub = n.advertise<visualization_msgs::Marker>("can_status", 10);  
  // ros::Publisher를 정의할때도 꼭 이처럼 사용하더라!!
  // ros::Publisher pub객체이름 = NodeHandle객체이름.advertise< 앞으로 topic으로 publish할 데이터 구조체의 형식type >("topic Name", 주기(?))
  // 그리고 나중에 pub을 pulish라는 맴버함수를 사용하기위에 선언해 놓는다. -> pub.publish(아래의 line_list처럼 pulish할 내용을 담은 구조체 객체);


  //visualization_msgs::MarkerArray markers;


  while(ros::ok()){  
    visualization_msgs::Marker line_list;  // 나중에 pubblish할 내용을 이 객체에 모두 저장할 것이다. 
    geometry_msgs::Point p;
    
	// 아래의 과정들은 line_list라는 구조체내부 맴버 변수에 값을 대입하는 작업이다. 
	// 내가 하는 can과는 관련이 없으므로 깊게 공부하지는 않는다. rviz를 위한 작업들이다. 
    line_list.header.frame_id = "velodyne";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "steer_speed";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    //  line_list.lifetime=ros::Duration(0.2);
    
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.2;
    line_list.color.r = 1.0;
    line_list.color.g = 0.0;
    line_list.color.b = 0.0;
    line_list.color.a = 1;

    p.x=-1;
    p.y=0;
    p.z=-1.5;

    double angle=0;
    printf("%f %f\n",g_speed,g_steer);
    for(double l=0; l<g_speed*g_speed/(2.0 * 1.0)+4; l+=0.2){
      line_list.points.push_back(p);      
      angle+=g_steer/100;
      p.x += 0.2*cos(angle);
      p.y += 0.2*sin(angle);
      line_list.points.push_back(p);      
    }
    pub.publish(line_list); // 이렇게 /can_status라는 토픽을 Pulish한다.

    printf("p\n");
    usleep(50000); // 50000마이크로초 동안 대기하기 = 0.05초 대기하기
    ros::spinOnce(); // 이 문장은 꼭 pub.publish( ) 근처아래에 적어 놓아야 한다.
  }
}
