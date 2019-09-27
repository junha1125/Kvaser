#include "autoware_msgs/Mobileye_New.h"

#include <ros/ros.h>
#include "autoware_can_msgs/CANPacket.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

autoware_msgs::Mobileye_New		Mobil_Publ_Data;


void chatterCallback(const autoware_can_msgs::CANPacket::ConstPtr& msg)
{
  unsigned short w;		// 사용 X
  static short sign_id;	// 사용 X
  static short oo_id;	// 사용 X

  unsigned char c;
  float f;
  unsigned int CAN_DATA_INT_temp;
  float float_Temp;
  int Int_Temp;
  static int enc_sum;
  short diff;
  char CharBuff[100];

  // header는 일단 동일하게!
  Mobil_Publ_Data.header = msg->header;

  /*Mobileye Base Data Set*/
  //0x669
  static short Con_Left, Con_Right;
  static float Dis_Left, Dis_Right;

  //0x700
  static short Head_Valid;
  static float Head_Mea;
  static short LDW_Left_On, LDW_Right_On;
  static short FCW_On, Head_W_Lv;

  //0x460
  static short Speed;

  //0x738
  static short Num_O;

  //0x739
  static short O_ID[10];
  static unsigned int O_X[10];
  static signed int O_Y[10];
  static short O_Valid[10];

  static short O_Lane[10], CIPV_Flag[10];

  int changed_m=0;				// 모빌아이 데이터 값이 바뀌었는가?
  int changed_m_o=0;			// 모빌아이_오브젝트 데이터 값이 바뀌었는가?

  static int Num_Valid,Sign_Valid,OO_Valid;	// 사용 X
  static bool sign_same_frame;				// 사용 X
  static bool O_same_frame;					// 사용 X





  /*Mobileye Data Decoder */
  // 0x669 4개의 데이터 정의
  if(msg->id==0x669){
    w=(msg->dat[0] & 0x03);
    Con_Left = w;

    sprintf(CharBuff, "%02X%02X", msg->dat[2],msg->dat[1] & 0xF0);
    float_Temp = strtol(CharBuff,NULL,16) / 16;
    if (((unsigned int)float_Temp & 0x0800) == 0x0800)
        Dis_Left = -((0x0FFF - float_Temp + 0X01) * 0.02);
    else
        Dis_Left = float_Temp * 0.02;

    w=(msg->dat[5] & 0x03);
    Con_Right = w;

    sprintf(CharBuff, "%02X%02X", msg->dat[7],msg->dat[6] & 0xF0);
    float_Temp = strtol(CharBuff,NULL,16) / 16;
    if (((unsigned int)float_Temp & 0x0800) == 0x0800)
        Dis_Right = -((0x0FFF - float_Temp + 0X01) * 0.02);
    else
        Dis_Right = float_Temp * 0.02;

    changed_m=1;
  }

  // 0x700 6개의 데이터 정의
  if(msg->id==0x700){
    w=(msg->dat[2] & 0x01);
    Head_Valid = w;

    sprintf(CharBuff, "%02X", msg->dat[2] & 0xFE);
    Head_Mea = strtol(CharBuff, NULL, 16) / 10;

    w=(msg->dat[4] & 0x02);
    LDW_Left_On = w / 2;
    w=(msg->dat[4] & 0x04);
    LDW_Right_On = w / 4;

    w=(msg->dat[7] & 0x03);
    Head_W_Lv = w;

    changed_m=1;
  }
  // 0x760 1개의 데이터 정의
  if(msg->id==0x760){
    w=(msg->dat[2]);
    Speed = w;
    changed_m=1;
  }

  // 0x738 1개의 데이터 정의
  if(msg->id==0x738){
    w=(msg->dat[0]);
    Num_O = w;

    if(Num_O == 0){
        memset(O_ID, 0, sizeof(O_ID));
        memset(O_X, 0, sizeof(O_X));
        memset(O_Y, 0, sizeof(O_Y));
        memset(O_Valid,0,sizeof(O_Valid));
        memset(O_Lane, 0, sizeof(O_Lane));
        memset(CIPV_Flag, 0, sizeof(CIPV_Flag));
    }
    changed_m=1;
  }

  // 0x739 6개의 데이터 정의
  unsigned int temp_unsigned;

  if((msg->id>=0x739)&&(msg->id<=(0x738 + 10 * 3))){
    if((msg->id - 0x739) % 3 == 0){
        /* 이게 굳이 필요한 작업인지 모르겠다. 만약 필요하다고 생각이 들면 주석 풀기 .
		if(msg->id == 0x739){
            memset(O_ID, 0, sizeof(O_ID));
            memset(O_X, 0, sizeof(O_X));
            memset(O_Y, 0, sizeof(O_Y));
            memset(O_Valid,0,sizeof(O_Valid));
            memset(O_Lane, 0, sizeof(O_Lane));
            memset(CIPV_Flag, 0, sizeof(CIPV_Flag));
		}
		*/
		// ID
		O_ID[(msg->id - 0x739) / 3] = (msg->dat[0] & 0x7F);

        // X좌표
		sprintf(CharBuff, "%02X%02X", msg->dat[2] & 0x07, msg->dat[1]& 0xFF); //0x7F일 수도 있다.
                                                                              //( 나누기 16 즉 수소점을 4비트 왼쪽으로.)
		O_X[(msg->id - 0x739) / 3] = ((unsigned int)strtol(CharBuff, NULL, 16)) * 0.0625;

        // Y좌표
		sprintf(CharBuff, "%02X%02X", msg->dat[4] & 0x03, msg->dat[3]& 0xFF); //0x7F일 수도 있다.
		temp_unsigned = ((unsigned int)strtol(CharBuff, NULL, 16));
		if ((temp_unsigned & 0x200) == 0x200 ) // 음수라면
		    O_Y[(msg->id - 0x739) / 3] = -((0x03FF - temp_unsigned + 0x0001) * 0.0625);
		else
		    O_Y[(msg->id - 0x739) / 3] = temp_unsigned * 0.0625;

		w = (msg->dat[7] & 0xC0);
		O_Valid[(msg->id - 0x739) / 3] = w / 64;

		changed_m = 1;
    }
    else if((msg->id - 0x739) % 3 == 1){
        w = (msg->dat[3] & 0x03);
        O_Lane[(msg->id - 0x739) / 3] = w;

        w = (msg->dat[3] & 0x04);
        CIPV_Flag[(msg->id - 0x739) / 3] = w / 4;

        changed_m = 1;
    }
  }



  if(changed_m){
      Mobil_Publ_Data.Con_Left = Con_Left;
      Mobil_Publ_Data.Dis_Left = Dis_Left;
      Mobil_Publ_Data.Con_Right = Con_Right;
      Mobil_Publ_Data.Dis_Right = Dis_Right;

      Mobil_Publ_Data.Head_Valid = Head_Valid;
      Mobil_Publ_Data.Head_Mea = Head_Mea;
      Mobil_Publ_Data.LDW_Left_On = LDW_Left_On;
      Mobil_Publ_Data.LDW_Right_On = LDW_Right_On;
      Mobil_Publ_Data.FCW_On = FCW_On;
      Mobil_Publ_Data.Head_W_Lv = Head_W_Lv;

      Mobil_Publ_Data.Speed = Speed;

      Mobil_Publ_Data.Num_O = Num_O;

      for (int i = 0; i < 10; i++){
          Mobil_Publ_Data.O_ID[i] = O_ID[i];
          Mobil_Publ_Data.O_X[i] = O_X[i];
          Mobil_Publ_Data.O_Y[i] = O_Y[i];
          Mobil_Publ_Data.O_Valid[i] = O_Valid[i];
          Mobil_Publ_Data.O_Lane[i] = O_Lane[i];
          Mobil_Publ_Data.CIPV_Flag[i] = CIPV_Flag[i];
      }
  }
}


int main (int argc, char *argv[]){
  autoware_can_msgs::CANPacket candat;						// 사실 이건 필요없음. 선배들이 잘 모르고 넣어놓은 것 같음.

  ros::init(argc, argv, "can_Mobileye_Decoder");					// Node이름이 can_converter이다.
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("can_raw", 1, chatterCallback);
  ros::Publisher Mobileye_pub = n.advertise<autoware_msgs::Mobileye_New>("Mobileye_New", 1);	// 토픽 이름이 /Mobileye 이다.

  while(ros::ok()){
    Mobileye_pub.publish(Mobil_Publ_Data);
    ros::spinOnce();
  }
}
