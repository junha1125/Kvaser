//autoware_msgs라는 패키지에 있는 k7이라는 해더파일을 가져온다. 이 해더파일에는 
//typedef struct k7{header ~ int ~ float ~ float ~ } k7;과 같은 구조체 정의가 하나 있을 뿐이다. 
#include "autoware_msgs/K7.h"
#include "autoware_msgs/Mobileye.h"
#include "autoware_msgs/Niro.h"

#include <ros/ros.h>
#include "autoware_can_msgs/CANPacket.h"
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
autoware_msgs::Mobileye		Mobileye_dat;
autoware_msgs::Niro			Niro_dat;



//	[/can_raw]라는 토픽을 받아들이고 그렇게 받아 들인 토픽을 잠시동안 msg변수에 저장한다.
//  당연히 msg변수도 [/can_raw]의 구조체 자료형
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

  // Niro Data Set
  static short Left_Turn_Indicator, Left_Turn_Indicator_out_sign, Right_Turn_Indicator, Right_Turn_Indicator_out_sign;
  static short Fog_Light, Tail_Light, Head_Light, Up_Light, Auto_Light, Light_Status_out_sign, Emergency_Light, Emergency_Light_out_sign;
  static short Wiper_Operation, Wiper_Speed, Rear_Wiper_Operation, Rear_Wiper_Speed;
  static short LDWS_Switch, LDWS_Left_Warning, LDWS_Right_Warning;
  static short Brake_Switch, Gear_Position, Parking_Brake, Startup_key_state;
  static short Driver_Door_switch, Passenger_Door_switch, Rear_left_door_switch, Rear_right_door_switch, Eco_switch, Driver_Seat_Belt, Assistant_Seat_Belt;
  static short Rear_Side_Warning, Rear_Camera, L_SPAS_Warning, R_SPAS_Warning;
  static short Brake_Pedal_Pressure;
  static float Throttle_Position, N_Steering_Angle, RPM;
  static short N_Steering_Angle_velocity, N_Speed1, N_Speed2, Lateral_Acceleration, Door_Lock, Air_Conditioner_Operation_Driver, Air_Conditioner_Operation_Assistant;
  static short Parking_Assistance_System;

  // K7 Data Set
  static short Tail_Lamp,Turn_Lamp,Wiper_Mode,Wiper_Sensivity;
  static short APM_Mode,APM_Operated_State,APM_ERROR_STATUS;
  static float Steering_Angle,Desired_Steering_Angle;
  static short TM_G_SEL_DISP;
  static short ASM_Mode,ASM_Operated_State,ASM_ERROR_STATUS,Brake_Pedal_Signal;
  static float APS_Analog_Value;

  // Mobileye Base Data Set
  static short Con_Left, Con_Right, LDW_Left, LDW_Right, Type_Left, Type_Right;
  static float Dis_Left, Dis_Right;
  static short Head_Valid, LDW_OFF, LDW_Left_On, LDW_Right_On;
  static float Head_Mea;
  static short FCW_On,P_FCW,P_DZ,TSR_En,TSR_W_Lv,Head_W_Lv,HW_R_En;
  static short Sign1,Sign2,Sign3,Sign4;
  static short S_Sign1,S_Sign2,S_Sign3,S_Sign4;
  static short HLB_dec,L_Beam;
  static float L_Cur,L_Head,Yaw,Pitch;
  static short Con_Area,R_LDW,L_LDW;
  static short Num_O,Timestamp,L_Close,R_Close,GO,Close_Car;
  static short L_Signal,R_Signal,Wiper,Lo_Beam,Hi_Beam,Speed;

  // Mobileye Sign & Obstacle Data Set
  static short Sign_Type,S_Sign_Type;
  static float Sign_X,Sign_Y,Sign_Z;
  static short Filter_Type;
  static short O_ID[10];
  static float O_X[10], O_Y[10];
  static short Blinker[10], Cut_IO[10];
  static float O_VX[10];
  static short O_Type[10], O_Status[10], O_Brake_Light[10], O_Valid[10];
  static float O_Len[10], O_Wid[10];
  static short O_Lane[10], CIPV_Flag[10];
  static float O_Angle_Rate[10], O_Scale_Change[10], O_AX[10];
  static short O_Replaced[10];
  static float O_Angle[10];

  // Original Data Set Call (Ignore)
  static short steer,shift,speed2,speed3,brake;	// 사용 X
  static char  speed;							// 사용 X
  static short enc,enc_p,enc_diff;				// 사용 X
  static short wheel1,wheel2,wheel3,wheel4;		// 사용 X
  static short gyro,accy,accz;					// 사용 X
  static char accx;								// 사용 X
  static FILE* log_fp;							// 사용 X

  int changed_k=0;				// k7 데이터 값이 바뀌었는가?
  int changed_m=0;				// 모빌아이 데이터 값이 바뀌었는가?
  int changed_m_o=0;			// 모빌아이_오브젝트 데이터 값이 바뀌었는가? 
  int changed_n=0;				// Niro 데이터 값이 바뀌었는가?
  static int Num_Valid,Sign_Valid,OO_Valid;	// 사용 X
  static bool sign_same_frame;				// 사용 X
  static bool O_same_frame;					// 사용 X

  /*
  if(msg->id == "메세지 아이디 넘버로 16진수로 작성하는 것이 편하다.(메뉴얼 참고)"){
    w=msg->dat["0~7 중에서 읽고자 하는 데이터의 위치"];
    if(...)
    else if(...)

    ...
    ...

    changed_k = 1; [또는 changed_m = 1; changed_n = 1;]
    가공하는 메세지의 아이디를 전부 확인하고 확인한 메세지의 아이디가 어떤 센서의 CAN인지에 따라 changed_ + 'k[m,n]' = 1;로 마무리한다.
    k = K7, m = Mobileye, n = Niro
  }
  */
  // Niro
  if(msg->id==0x541){
    w=msg->dat[5];
    if(w == 0x04) {
        Left_Turn_Indicator = 1;
    }
    else if(w == 0x00) {
        Left_Turn_Indicator = 0;
    }

    if(w == 0x02) {
        Right_Turn_Indicator = 1;
    }
    else if(w == 0x00) {
        Right_Turn_Indicator = 0;
    }

    if(w == 0x01) Fog_Light = 1;
    else if(w == 0x00) Fog_Light = 0;

    w=msg->dat[2];
    if(w == 0x29) {
        Left_Turn_Indicator_out_sign = 1;
    }
    else if(w == 0x21) {
        Left_Turn_Indicator_out_sign = 0;
    }

    if(w == 0x40) {
        Right_Turn_Indicator_out_sign = 1;
    }
    else if(w == 0x00) {
        Right_Turn_Indicator_out_sign = 0;
    }

    if(w == 0x69) {
        Emergency_Light_out_sign = 1;
    }
    else {
        Emergency_Light_out_sign = 0;
    }

    w=msg->dat[4];
    if(w == 0x02) Emergency_Light = 1;
    else if(w == 0x00) Emergency_Light = 0;

    if(w == 0x04) Tail_Light = 1;
    else if(w == 0x00) Tail_Light = 0;

    if(w == 0x80) Head_Light = 1;
    else if(w == 0x00) Head_Light = 0;

    if((msg->dat[4] == 0x01) && (msg->dat[6] == 0x08)) Up_Light = 1;
    else if((msg->dat[4] == 0x00) && (msg->dat[6] == 0x00)) Up_Light = 0;

    if(w == 0xc0) Auto_Light = 1;
    else if(w == 0x00) Auto_Light = 0;

    w=msg->dat[3];
    if(w == 0x80) Light_Status_out_sign = 1;
    else if(w == 0x00) Light_Status_out_sign = 0;

    changed_n=1;
  }
  if(msg->id==0x553){
    w=msg->dat[4];
    if(w == 0x00) Wiper_Operation = 1;
    else if(w == 0x01) Wiper_Operation = 0;
    changed_n=1;
  }
  if(msg->id==0x559){
    w=msg->dat[2];
    Wiper_Speed = w / 4;

    w=msg->dat[4];
    Rear_Wiper_Operation = w / 16 / 4;

    changed_n=1;
  }
  if(msg->id==0x541){
    w=msg->dat[2];
    Rear_Wiper_Speed = w / 16 / 2;
    changed_n=1;
  }
  if(msg->id==0x340){
    w=(msg->dat[6] & 0x0F);
    if(w == 0x0B) LDWS_Switch = 1;
    else if(w == 0x07) LDWS_Switch = 0;

    w=(msg->dat[1] & 0x0F);
    if(w == 0x08) LDWS_Left_Warning = 1;
    else if(w == 0x00) LDWS_Left_Warning = 0;

    w=(msg->dat[1] & 0xF0);
    if(w == 0x20) LDWS_Right_Warning = 1;
    else if(w == 0x00) LDWS_Right_Warning = 0;
    changed_n=1;
  }
  if(msg->id==0x394){
    if((msg->dat[7] == 0x93) && ((msg->dat[5] & 0xF0) == 0xc0)) Brake_Switch = 1;
    else if((msg->dat[7] == 0x83) && (msg->dat[5] == 0x84)) Brake_Switch = 0;

    w=(msg->dat[7] & 0xF0);
    if(w == 0x10) Parking_Brake = 1;
    else Parking_Brake = 0;
    changed_n=1;
  }
  if(msg->id==0x372){
    w=msg->dat[2];
    Gear_Position = w;
    changed_n=1;
  }
  if(msg->id==0x371){
    w=msg->dat[2];
    Startup_key_state = w;
    changed_n=1;
  }
  if(msg->id==0x541){
    w=msg->dat[2];
    if(w == 0x40) Driver_Door_switch = 0;
    else if(w == 0x41) Driver_Door_switch = 1;

    w=msg->dat[4];
    if(w == 0x00) Passenger_Door_switch = 0;
    else if(w == 0x08) Passenger_Door_switch = 1;

    w=(msg->dat[1] & 0x0F);
    if(w == 0x04) Driver_Seat_Belt = 1;
    else if(w == 0x00) Driver_Seat_Belt = 0;

    w=(msg->dat[1] & 0xF0);
    if(w == 0x40) Assistant_Seat_Belt = 1;
    else if(w == 0x00) Assistant_Seat_Belt = 0;
    changed_n=1;
  }
  if(msg->id==0x553){
    w=msg->dat[3];
    if(w == 0x03) Rear_left_door_switch = 0;
    else if(w == 0x02) Rear_left_door_switch = 1;

    w=msg->dat[2];
    if(w == 0x11) Rear_right_door_switch = 0;
    else if(w == 0x91) Rear_right_door_switch = 1;
    changed_n=1;
  }
  if(msg->id==0x381){
    w=msg->dat[7];
    Eco_switch = w;
    changed_n=1;
  }
  if(msg->id==0x58B){
    w=(msg->dat[0] & 0x0F);
    Rear_Side_Warning = w;
    w=(msg->dat[0] & 0x20);
    Rear_Camera = w / 32;
    w=msg->dat[1];
    if(w == 0x00) L_SPAS_Warning = 0;
    else if(w == 0x01) L_SPAS_Warning = 1;
    else if(w == 0x02) L_SPAS_Warning = 2;
    w=msg->dat[2];
    if(w == 0x04) R_SPAS_Warning = 0;
    else if(w == 0x05) R_SPAS_Warning = 1;
    else if(w == 0x16) R_SPAS_Warning = 2;
    changed_n=1;
  }
  if(msg->id==0x371){
    w=msg->dat[0];
    Brake_Pedal_Pressure = w;
    w=msg->dat[7];
    Throttle_Position = w * 0.392157;
    w=msg->dat[4];
    N_Speed1 = w;
    changed_n=1;
  }
  if(msg->id==0x2B0){
    sprintf(CharBuff, "%02X%02X", msg->dat[1],msg->dat[0]);
    N_Steering_Angle = strtol(CharBuff,NULL,16);
    CAN_DATA_INT_temp = ((unsigned int)N_Steering_Angle & 0x8000);
    if(CAN_DATA_INT_temp == 0x8000)
      N_Steering_Angle = (0xFFFF - N_Steering_Angle + 1) / 10;
    else
      N_Steering_Angle = -(N_Steering_Angle / 10);
    w=msg->dat[2];
    N_Steering_Angle_velocity = 4 * w;
    changed_n=1;
  }
  if(msg->id==0x52A){
    w=msg->dat[0];
    N_Speed2 = w;
    changed_n=1;
  }
  if(msg->id==0x371){
    RPM = (float)(msg->dat[3] * 256 + msg->dat[2]) / 4;
    changed_n=1;
  }
  if(msg->id==0x130){
    w=msg->dat[5];
    Lateral_Acceleration = w;
    changed_n=1;
  }
  if(msg->id==0x559){
    w=msg->dat[1];
    if(w == 0x40) Door_Lock = 1;
    else if(w == 0x00) Door_Lock = 0;
    changed_n=1;
  }
  if(msg->id==0x5C4){
    w=msg->dat[0];
    Air_Conditioner_Operation_Driver = w / 2 + 14;
    if(w == 0x00) Air_Conditioner_Operation_Driver = 0;
    w=msg->dat[2];
    Air_Conditioner_Operation_Assistant = w / 2 + 14;
    if(w == 0x00) Air_Conditioner_Operation_Assistant = 0;
    changed_n=1;
  }
  if(msg->id==0x436){
    if((msg->dat[0] == 0x40) && (msg->dat[1] == 0x08) && (msg->dat[2] == 0x00) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 0;
    else if((msg->dat[0] == 0x80) && (msg->dat[1] == 0x10) && (msg->dat[2] == 0x00) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 1;
    else if((msg->dat[0] == 0xC0) && (msg->dat[1] == 0x18) && (msg->dat[2] == 0x10) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 2;

    else if((msg->dat[0] == 0x80) && (msg->dat[1] == 0x00) && (msg->dat[2] == 0x02) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 11;
    else if((msg->dat[0] == 0xC0) && (msg->dat[1] == 0x00) && (msg->dat[2] == 0x03) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 12;

    else if((msg->dat[0] == 0x80) && (msg->dat[1] == 0x00) && (msg->dat[2] == 0x10) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 21;
    else if((msg->dat[0] == 0xC0) && (msg->dat[1] == 0x00) && (msg->dat[2] == 0x18) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 22;

    else if((msg->dat[0] == 0x00) && (msg->dat[1] == 0x41) && (msg->dat[2] == 0x00) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 100;
    else if((msg->dat[0] == 0x00) && (msg->dat[1] == 0x82) && (msg->dat[2] == 0x00) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 101;
    else if((msg->dat[0] == 0x00) && (msg->dat[1] == 0xC3) && (msg->dat[2] == 0x00) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 102;

    else if((msg->dat[0] == 0x02) && (msg->dat[1] == 0x80) && (msg->dat[2] == 0x00) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 111;
    else if((msg->dat[0] == 0x03) && (msg->dat[1] == 0xC0) && (msg->dat[2] == 0x00) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 112;

    else if((msg->dat[0] == 0x10) && (msg->dat[1] == 0x80) && (msg->dat[2] == 0x00) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 121;
    else if((msg->dat[0] == 0x18) && (msg->dat[1] == 0xC0) && (msg->dat[2] == 0x00) && (msg->dat[3] == 0x09)) Parking_Assistance_System = 122;
    changed_n=1;
  }

  // K7
  if(msg->id==0x50){
    w=(msg->dat[1] & 0x0F);
    Tail_Lamp = w;
    w=(msg->dat[2] & 0xF0);
    Turn_Lamp = w/16;
    w=(msg->dat[2] & 0x0F);
    Wiper_Mode = w;
    w=(msg->dat[1] & 0xF0);
    if(Wiper_Mode == 8 || Wiper_Mode == 9)
    {
        if (w/16 == 2)
            Wiper_Sensivity = 0;
        else if (w/16 == 4)
            Wiper_Sensivity = 2;
        else if (w/16 == 6)
            Wiper_Sensivity = 4;
        else if (w/16 == 8)
            Wiper_Sensivity = 6;
        else
            Wiper_Sensivity = 8;
    }
    else
	Wiper_Sensivity = 0;
    changed_k=1;
  }
  if(msg->id==0x710){
    w=(msg->dat[0] & 0x01);
    APM_Mode = w;
    w=(msg->dat[0] & 0x0E);
    APM_Operated_State = w;
    w=(msg->dat[0] & 0x70);
    APM_ERROR_STATUS = w/16;

    sprintf(CharBuff, "%02X%02X", msg->dat[2],msg->dat[1]);
    Steering_Angle = strtol(CharBuff,NULL,16);
    CAN_DATA_INT_temp = ((unsigned int)Steering_Angle & 0x8000);
    if(CAN_DATA_INT_temp == 0x8000)
        Steering_Angle = (0xFFFF - Steering_Angle + 1) / 10;
    else
        Steering_Angle = -(Steering_Angle / 10);

    sprintf(CharBuff, "%02X%02X", msg->dat[4],msg->dat[3]);
    Desired_Steering_Angle = strtol(CharBuff,NULL,16);
    CAN_DATA_INT_temp = ((unsigned int)Desired_Steering_Angle & 0x8000);
    if(CAN_DATA_INT_temp == 0x8000)
        Desired_Steering_Angle = (0xFFFF - Desired_Steering_Angle + 1) / 10;
    else
        Desired_Steering_Angle = -(Desired_Steering_Angle / 10);
    changed_k=1;
  }
  if(msg->id==0x631){
    w=(msg->dat[1] & 0x0F);
    TM_G_SEL_DISP = w;
    changed_k=1;
  }
  if(msg->id==0x711){
    w=(msg->dat[0] & 0x01);
    ASM_Mode = w;

    w=(msg->dat[0] & 0x0E);
    ASM_Operated_State = w;

    w=(msg->dat[0] & 0xF0);
    ASM_ERROR_STATUS = w/16;

    if(msg->dat[4] == 0x00)
        Brake_Pedal_Signal = 0;
    else
        Brake_Pedal_Signal = 1;

    sprintf(CharBuff, "%02X%02X", msg->dat[6],msg->dat[5]);
    APS_Analog_Value = strtol(CharBuff,NULL,16);
    APS_Analog_Value = APS_Analog_Value/1000;
    changed_k=1;
  }
  
  // Mobileye
  if(msg->id==0x669){
    w=(msg->dat[0] & 0x03);
    Con_Left = w;
    w=(msg->dat[0] & 0x04);
    LDW_Left = w/4;
    w=(msg->dat[0] & 0xF0);
    Type_Left = w / 16;
    sprintf(CharBuff, "%02X%02X", msg->dat[2],msg->dat[1] & 0xF0);
    float_Temp = strtol(CharBuff,NULL,16) / 16;
    if (((unsigned int)float_Temp & 0x0800) == 0x0800)
        Dis_Left = -((0x0FFF - float_Temp + 0X01) * 0.02);
    else
        Dis_Left = float_Temp * 0.02;
    w=(msg->dat[5] & 0x03);
    Con_Right = w;
    w=(msg->dat[5] & 0x04);
    LDW_Right = w/4;
    w=(msg->dat[5] & 0xF0);
    Type_Left = w / 16;
    sprintf(CharBuff, "%02X%02X", msg->dat[7],msg->dat[6] & 0xF0);
    float_Temp = strtol(CharBuff,NULL,16) / 16;
    if (((unsigned int)float_Temp & 0x0800) == 0x0800)
        Dis_Right = -((0x0FFF - float_Temp + 0X01) * 0.02);
    else
        Dis_Right = float_Temp * 0.02;
    changed_m=1;
  }

  if(msg->id==0x700){
    w=(msg->dat[2] & 0x01);
    Head_Valid = w;
    sprintf(CharBuff, "%02X", msg->dat[2] & 0xFE);
    Head_Mea = strtol(CharBuff, NULL, 16) / 10;
    w=(msg->dat[4] & 0x01);
    LDW_OFF = w;
    w=(msg->dat[4] & 0x02);
    LDW_Left_On = w / 2;
    w=(msg->dat[4] & 0x04);
    LDW_Right_On = w / 4;
    w=(msg->dat[4] & 0x08);
    FCW_On = w / 8;
    w=(msg->dat[5] & 0x02);
    P_FCW = w / 2;
    w=(msg->dat[5] & 0x04);
    P_DZ = w / 4;
    w=(msg->dat[5] & 0x80);
    TSR_En = w / 128;
    w=(msg->dat[6] & 0x07);
    TSR_W_Lv = w;
    w=(msg->dat[7] & 0x03);
    Head_W_Lv = w;
    w=(msg->dat[7] & 0x04);
    HW_R_En = w / 4;
    changed_m=1;
  }
  if(msg->id==0x727){
    Sign1 = msg->dat[0];
    Sign2 = msg->dat[2];
    Sign3 = msg->dat[4];
    Sign4 = msg->dat[6];
    S_Sign1 = msg->dat[1];
    S_Sign2 = msg->dat[3];
    S_Sign3 = msg->dat[5];
    S_Sign4 = msg->dat[7];
    changed_m=1;
  }
  if(msg->id==0x728){
    w=(msg->dat[0] & 0x03);
    HLB_dec = w;
    sprintf(CharBuff, "%02X%02X", msg->dat[2] & 0x01, msg->dat[1]);
    Int_Temp = strtol(CharBuff, NULL, 16);
    L_Beam = Int_Temp;
    changed_m=1;
  }
  if(msg->id==0x737){
    sprintf(CharBuff, "%02X%02X", msg->dat[1], msg->dat[0]);
    float_Temp = strtol(CharBuff, NULL, 16);
    if (((unsigned int)float_Temp & 0x8000) == 0x8000)
        L_Cur = -((0xFFFF - float_Temp + 0X0001) * 3.81 * pow(10,-6));
    else
        L_Cur = float_Temp * 3.81 * pow(10,-6);
    sprintf(CharBuff, "%02X%02X", msg->dat[3] & 0x0F, msg->dat[2]);
    float_Temp = strtol(CharBuff, NULL, 16);
    if (((unsigned int)float_Temp & 0x0800) == 0x0800)
        L_Head = -((0x0FFF - float_Temp + 0X0001) * 0.0005);
    else
        L_Head = float_Temp * 0.0005;
    w=(msg->dat[3] & 0x10);
    Con_Area = w / 16;
    w=(msg->dat[3] & 0x20);
    R_LDW = w / 32;
    w=(msg->dat[3] & 0x40);
    L_LDW = w / 64;
    sprintf(CharBuff, "%02X%02X", msg->dat[5], msg->dat[4]);
    float_Temp = strtol(CharBuff, NULL, 16);
    if ((unsigned int)float_Temp < 0X7FFF)
        Yaw = -(0X7FFF - float_Temp) / 1024 / 512;
    else
        Yaw = (float_Temp - 0X7FFF) / 1024 / 512;
    sprintf(CharBuff, "%02X%02X", msg->dat[7], msg->dat[6]);
    float_Temp = strtol(CharBuff, NULL, 16);
    if ((unsigned int)float_Temp < 0X7FFF)
        Pitch = -(0X7FFF - float_Temp) / 1024 / 512;
    else
        Pitch = (float_Temp - 0X7FFF) / 1024 / 512;
    changed_m=1;
  }
  if(msg->id==0x738){
    w=(msg->dat[0]);
    Num_O = w;
    w=(msg->dat[1]);
    Timestamp = w;
    w=(msg->dat[3] & 0x04);
    L_Close = w / 4;
    w=(msg->dat[3] & 0x08);
    R_Close = w / 8;
    w=(msg->dat[3] & 0xF0);
    GO = w / 16;
    w=(msg->dat[5] & 0x01);
    Close_Car = w;
    Num_Valid = 1;
    if(Num_O == 0){
        memset(O_ID, 0, sizeof(O_ID));
        memset(O_X, 0, sizeof(O_X));
        memset(O_Y, 0, sizeof(O_Y));
        memset(Blinker, 0, sizeof(Blinker));
        memset(Cut_IO, 0, sizeof(Cut_IO));
        memset(O_VX, 0, sizeof(O_VX));
        memset(O_Type, 0, sizeof(O_Type));
        memset(O_Status, 0, sizeof(O_Status));
        memset(O_Brake_Light, 0, sizeof(O_Brake_Light));
        memset(O_Valid, 0, sizeof(O_Valid));
        memset(O_Len, 0, sizeof(O_Len));
        memset(O_Wid, 0, sizeof(O_Wid));
        memset(O_Lane, 0, sizeof(O_Lane));
        memset(CIPV_Flag, 0, sizeof(CIPV_Flag));
        memset(O_Angle_Rate, 0, sizeof(O_Angle_Rate));
        memset(O_Scale_Change, 0, sizeof(O_Scale_Change));
        memset(O_AX, 0, sizeof(O_AX));
        memset(O_Replaced, 0, sizeof(O_Replaced));
        memset(O_Angle, 0, sizeof(O_Angle));
    }
    changed_m=1;
  }
  if(msg->id==0x760){
    w=(msg->dat[0] & 0x02);
    L_Signal = w / 2;
    w=(msg->dat[0] & 0x04);
    R_Signal = w / 4;
    w=(msg->dat[0] & 0x08);
    Wiper = w / 8;
    w=(msg->dat[0] & 0x10);
    Lo_Beam = w / 16;
    w=(msg->dat[0] & 0x20);
    Hi_Beam = w / 32;
    w=(msg->dat[2]);
    Speed = w;
    changed_m=1;
  }
  if((msg->id>=0x720)&&(msg->id<=0x726)){
    Sign_Type = msg->dat[0];
    S_Sign_Type = msg->dat[1];
    sprintf(CharBuff, "%02X", msg->dat[2]);
    Sign_X = strtol(CharBuff, NULL, 16);
    sprintf(CharBuff, "%02X", msg->dat[3] & 0x7F);
    float_Temp = strtol(CharBuff, NULL, 16);
    if (((unsigned int)float_Temp & 0x40) == 0x40 )
        Sign_Y = -(0x7F - float_Temp + 0x01);
    else
        Sign_Y = float_Temp;
    sprintf(CharBuff, "%02X", msg->dat[4] & 0x3F);
    float_Temp = strtol(CharBuff, NULL, 16);
    if (((unsigned int)float_Temp & 0x20) == 0x20 )
        Sign_Z = -(0x3F - float_Temp + 0x01);
    else
        Sign_Z = float_Temp;
    Filter_Type = msg->dat[5];
    changed_m=1;
  }



  /* 
  총 10개의 obstacle에 대한 Data를 저장할 수 있으며
  각 Obstacle 하나하나에 대해서 많은 정보들을 얻을 수 있다. 
  0x739로 빼고, 3으로 나눈 나머지가 0인 메세지에서는 물체ID, 위치, Type 등을 알 수 있고
  0x739로 빼고, 3으로 나눈 나머지가 1인 메세지에서는 물체의 길이, 높이, 래이더 정보 등을 알 수 있고
  0x739로 빼고, 3으로 나눈 나머지가 2인 메세지에서는 물체 가속도, 물체 Angle, Scale등에 대한 정보를 얻을 수 있다.
  */
  if((msg->id>=0x739)&&(msg->id<=(0x738 + 10 * 3))){
    if((msg->id - 0x739) % 3 == 0){

		//첫번쨰 물체에 대한 정보가 0x739 0x73A 0x73B에 담겨있을것 이라는 게 나의 생각인데
		//첫번째 물체에 대한 Older Vehicle 정보가 변수에 저장되어 있을지도 있으니, 0으로 초기화 한다.  
		// 근데데 첫번쨰 obstacle데이터{obstacle[0]}만 초기화를 하지...? obstacle[1]는 안하나?
		if(msg->id == 0x739){ 
			memset(O_ID, 0, sizeof(O_ID));
			memset(O_X, 0, sizeof(O_X));
			memset(O_Y, 0, sizeof(O_Y));
			memset(Blinker, 0, sizeof(Blinker));
			memset(Cut_IO, 0, sizeof(Cut_IO));
			memset(O_VX, 0, sizeof(O_VX));
			memset(O_Type, 0, sizeof(O_Type));
			memset(O_Status, 0, sizeof(O_Status));
			memset(O_Brake_Light, 0, sizeof(O_Brake_Light));
			memset(O_Valid, 0, sizeof(O_Valid));
			memset(O_Len, 0, sizeof(O_Len));
			memset(O_Wid, 0, sizeof(O_Wid));
			memset(O_Lane, 0, sizeof(O_Lane));
			memset(CIPV_Flag, 0, sizeof(CIPV_Flag));
			memset(O_Angle_Rate, 0, sizeof(O_Angle_Rate));
			memset(O_Scale_Change, 0, sizeof(O_Scale_Change));
			memset(O_AX, 0, sizeof(O_AX));
			memset(O_Replaced, 0, sizeof(O_Replaced));
			memset(O_Angle, 0, sizeof(O_Angle));
		}

		O_ID[(msg->id - 0x739) / 3] = (msg->dat[0] & 0x7F);
		sprintf(CharBuff, "%02X%02X", msg->dat[2] & 0x0F, msg->dat[1]);
		O_X[(msg->id - 0x739) / 3] = strtol(CharBuff, NULL, 16);
		sprintf(CharBuff, "%02X%02X", msg->dat[4] & 0x03, msg->dat[3]);
		float_Temp = strtol(CharBuff, NULL, 16);
		if (((unsigned int)float_Temp & 0x0200) == 0x0200 )
		    O_Y[(msg->id - 0x739) / 3] = -((0x03FF - float_Temp + 0x0001) * 0.0625);
		else
		    O_Y[(msg->id - 0x739) / 3] = float_Temp * 0.0625;
		w = (msg->dat[4] & 0x1C);
		Blinker[(msg->id - 0x739) / 3] = w / 4;
		w = (msg->dat[4] & 0xE0);
		Cut_IO[(msg->id - 0x739) / 3] = w / 32;
		sprintf(CharBuff, "%02X%02X", msg->dat[6] & 0x0F, msg->dat[5]);
		float_Temp = strtol(CharBuff, NULL, 16);
		if (((unsigned int)float_Temp & 0x0800) == 0x0800 )
		    O_VX[(msg->id - 0x739) / 3] = -((0x0FFF - float_Temp + 0x0001) * 0.0625);
		else
		    O_VX[(msg->id - 0x739) / 3] = float_Temp * 0.0625;
		w = (msg->dat[6] & 0x70);
		O_Type[(msg->id - 0x739) / 3] = w / 16;
		w = (msg->dat[7] & 0x07);
		O_Status[(msg->id - 0x739) / 3] = w;
		w = (msg->dat[7] & 0x08);
		O_Brake_Light[(msg->id - 0x739) / 3] = w / 8;
		w = (msg->dat[7] & 0xC0);
		O_Valid[(msg->id - 0x739) / 3] = w / 64;
    }
    else if((msg->id - 0x739) % 3 == 1){
        sprintf(CharBuff, "%02X", msg->dat[0]);
        O_Len[(msg->id - 0x739) / 3] = strtol(CharBuff, NULL, 16) * 0.5;
        sprintf(CharBuff, "%02X", msg->dat[1]);
        O_Wid[(msg->id - 0x739) / 3] = strtol(CharBuff, NULL, 16) * 0.05;
        w = (msg->dat[3] & 0x03);
        O_Lane[(msg->id - 0x739) / 3] = w;
        w = (msg->dat[3] & 0x04);
        CIPV_Flag[(msg->id - 0x739) / 3] = w / 4;
    }
    else if((msg->id - 0x739) % 3 == 2){
        sprintf(CharBuff, "%02X%02X", msg->dat[1], msg->dat[0]);
        float_Temp = strtol(CharBuff, NULL, 16);
        if (((unsigned int)float_Temp & 0x8000) == 0x8000 )
            O_Angle_Rate[(msg->id - 0x739) / 3] = - ((0xFFFF - float_Temp + 0x0001) * 0.01);
        else
            O_Angle_Rate[(msg->id - 0x739) / 3] = float_Temp * 0.01;
        sprintf(CharBuff, "%02X%02X", msg->dat[3], msg->dat[2]);
        float_Temp = strtol(CharBuff, NULL, 16);
        if (((unsigned int)float_Temp & 0x8000) == 0x8000 )
            O_Scale_Change[(msg->id - 0x739) / 3] = - ((0xFFFF - float_Temp + 0x0001) * 0.0002);
        else
            O_Scale_Change[(msg->id - 0x739) / 3] = float_Temp * 0.0002;
        sprintf(CharBuff, "%02X%02X", msg->dat[5] & 0x03, msg->dat[4]);
        float_Temp = strtol(CharBuff, NULL, 16);
        if (((unsigned int)float_Temp & 0x0200) == 0x0200 )
            O_AX[(msg->id - 0x739) / 3] = - ((0x03FF - float_Temp + 0x0001) * 0.03);
        else
            O_AX[(msg->id - 0x739) / 3] = float_Temp * 0.03;
        w = (msg->dat[5] & 0x10);
        O_Replaced[(msg->id - 0x739) / 3] = w / 16;
        sprintf(CharBuff, "%02X%02X", msg->dat[7] & 0x03, msg->dat[6]);
        float_Temp = strtol(CharBuff, NULL, 16);
        if (((unsigned int)float_Temp & 0x0200) == 0x0200 )
            O_Angle[(msg->id - 0x739) / 3] = - ((0x03FF - float_Temp + 0x0001) * 0.01);
        else
            O_Angle[(msg->id - 0x739) / 3] = float_Temp * 0.01;
	
		changed_m = 1;
    }
  }







  /*
  데이터를 전부 확인하고 가공했으면 topic에 보낼 데이터를 수정하기 위한 과정을 거친다.
  각각의 메세지 구조에 맞추어 필요한 데이터를 저장하고 topic으로 보낼 준비를 한다.
  */
  if(changed_n){
    Niro_dat.tosec = msg->header.stamp.toSec();
    Niro_dat.time = msg->time;
    Niro_dat.Left_Turn_Indicator = Left_Turn_Indicator;
    Niro_dat.Left_Turn_Indicator_out_sign = Left_Turn_Indicator_out_sign;
    Niro_dat.Right_Turn_Indicator = Right_Turn_Indicator;
    Niro_dat.Right_Turn_Indicator_out_sign = Right_Turn_Indicator_out_sign;
    Niro_dat.Fog_Light = Fog_Light;
    Niro_dat.Tail_Light = Tail_Light;
    Niro_dat.Head_Light = Head_Light;
    Niro_dat.Up_Light = Up_Light;
    Niro_dat.Auto_Light = Auto_Light;
    Niro_dat.Light_Status_out_sign = Light_Status_out_sign;
    Niro_dat.Emergency_Light = Emergency_Light;
    Niro_dat.Emergency_Light_out_sign = Emergency_Light_out_sign;
    Niro_dat.Wiper_Operation = Wiper_Operation;
    Niro_dat.Wiper_Speed = Wiper_Speed;
    Niro_dat.Rear_Wiper_Operation = Rear_Wiper_Operation;
    Niro_dat.Rear_Wiper_Speed = Rear_Wiper_Speed;
    Niro_dat.LDWS_Switch = LDWS_Switch;
    Niro_dat.LDWS_Left_Warning = LDWS_Left_Warning;
    Niro_dat.LDWS_Right_Warning = LDWS_Right_Warning;
    Niro_dat.Brake_Switch = Brake_Switch;
    Niro_dat.Gear_Position = Gear_Position;
    Niro_dat.Parking_Brake = Parking_Brake;
    Niro_dat.Startup_key_state = Startup_key_state;
    Niro_dat.Driver_Door_switch = Driver_Door_switch;
    Niro_dat.Passenger_Door_switch = Passenger_Door_switch;
    Niro_dat.Rear_left_door_switch = Rear_left_door_switch;
    Niro_dat.Rear_right_door_switch = Rear_right_door_switch;
    Niro_dat.Eco_switch = Eco_switch;
    Niro_dat.Driver_Seat_Belt = Driver_Seat_Belt;
    Niro_dat.Assistant_Seat_Belt = Assistant_Seat_Belt;
    Niro_dat.Rear_Side_Warning = Rear_Side_Warning;
    Niro_dat.Rear_Camera = Rear_Camera;
    Niro_dat.L_SPAS_Warning = L_SPAS_Warning;
    Niro_dat.R_SPAS_Warning = R_SPAS_Warning;
    Niro_dat.Brake_Pedal_Pressure = Brake_Pedal_Pressure;
    Niro_dat.Throttle_Position = Throttle_Position;
    Niro_dat.N_Steering_Angle = N_Steering_Angle;
    Niro_dat.N_Steering_Angle_velocity = N_Steering_Angle_velocity;
    Niro_dat.N_Speed1 = N_Speed1;
    Niro_dat.N_Speed2 = N_Speed2;
    Niro_dat.RPM = RPM;
    Niro_dat.Lateral_Acceleration = Lateral_Acceleration;
    Niro_dat.Door_Lock = Door_Lock;
    Niro_dat.Air_Conditioner_Operation_Driver = Air_Conditioner_Operation_Driver;
    Niro_dat.Air_Conditioner_Operation_Assistant = Air_Conditioner_Operation_Assistant;
    Niro_dat.Parking_Assistance_System = Parking_Assistance_System;
  }
  if(changed_k){
      /*
      주석처리된 코드는 가공된 CAN의 데이터를 txt 파일에 저장하는 것으로 여기서는 필요하지 않다.
      */
//    if(!log_fp)log_fp=fopen("/tmp/can_log","w");
    //fprintf(log_fp,"%f %d %d %d %d %d\n", msg->header.stamp.toSec(),msg->time,Tail_Lamp,Turn_Lamp,Wiper_Mode,Wiper_Sensivity);
    /*printf("%f %d %d %d %d %d %d %d %d %f %f %d %d %d %d %d %f\n",msg->header.stamp.toSec(), msg->time,Tail_Lamp,Turn_Lamp,Wiper_Mode,Wiper_Sensivity,APM_Mode,
APM_Operated_State,APM_ERROR_STATUS,Steering_Angle,Desired_Steering_Angle,TM_G_SEL_DISP,ASM_Mode,ASM_Operated_State,ASM_ERROR_STATUS,Brake_Pedal_Signal,APS_Analog_Value);*/

    k7_dat.tosec = msg->header.stamp.toSec();
    k7_dat.time = msg->time;
    k7_dat.Tail_Lamp = Tail_Lamp;
    k7_dat.Turn_Lamp = Turn_Lamp;
    k7_dat.Wiper_Mode = Wiper_Mode;
    k7_dat.Wiper_Sensivity = Wiper_Sensivity;
    k7_dat.APM_Mode = APM_Mode;
    k7_dat.APM_Operated_State = APM_Operated_State;
    k7_dat.APM_ERROR_STATUS = APM_ERROR_STATUS;
    k7_dat.Steering_Angle = Steering_Angle;
    k7_dat.Desired_Steering_Angle = Desired_Steering_Angle;
    k7_dat.TM_G_SEL_DISP = TM_G_SEL_DISP;
    k7_dat.ASM_Mode = ASM_Mode;
    k7_dat.ASM_Operated_State = ASM_Operated_State;
    k7_dat.ASM_ERROR_STATUS = ASM_ERROR_STATUS;
    k7_dat.Brake_Pedal_Signal = Brake_Pedal_Signal;
    k7_dat.APS_Analog_Value = APS_Analog_Value;
  }
  if(changed_m){
      Mobileye_dat.tosec = msg->header.stamp.toSec();
      Mobileye_dat.time = msg->time;
      Mobileye_dat.Con_Left = Con_Left;
      Mobileye_dat.LDW_Left = LDW_Left;
      Mobileye_dat.Type_Left = Type_Left;
      Mobileye_dat.Dis_Left = Dis_Left;
      Mobileye_dat.Con_Right = Con_Right;
      Mobileye_dat.LDW_Right = LDW_Right;
      Mobileye_dat.Type_Right = Type_Right;
      Mobileye_dat.Dis_Right = Dis_Right;
      Mobileye_dat.Head_Valid = Head_Valid;
      Mobileye_dat.Head_Mea = Head_Mea;
      Mobileye_dat.LDW_OFF = LDW_OFF;
      Mobileye_dat.LDW_Left_On = LDW_Left_On;
      Mobileye_dat.LDW_Right_On = LDW_Right_On;
      Mobileye_dat.FCW_On = FCW_On;
      Mobileye_dat.P_FCW = P_FCW;
      Mobileye_dat.P_DZ = P_DZ;
      Mobileye_dat.TSR_En = TSR_En;
      Mobileye_dat.TSR_W_Lv = TSR_W_Lv;
      Mobileye_dat.Head_W_Lv = Head_W_Lv;
      Mobileye_dat.HW_R_En = HW_R_En;
      Mobileye_dat.Sign1 = Sign1;
      Mobileye_dat.Sign2 = Sign2;
      Mobileye_dat.Sign3 = Sign3;
      Mobileye_dat.Sign4 = Sign4;
      Mobileye_dat.S_Sign1 = S_Sign1;
      Mobileye_dat.S_Sign2 = S_Sign2;
      Mobileye_dat.S_Sign3 = S_Sign3;
      Mobileye_dat.S_Sign4 = S_Sign4;
      Mobileye_dat.HLB_dec = HLB_dec;
      Mobileye_dat.L_Beam = L_Beam;
      Mobileye_dat.L_Cur = L_Cur;
      Mobileye_dat.L_Head = L_Head;
      Mobileye_dat.Con_Area = Con_Area;
      Mobileye_dat.R_LDW = R_LDW;
      Mobileye_dat.L_LDW = L_LDW;
      Mobileye_dat.Yaw = Yaw;
      Mobileye_dat.Pitch = Pitch;
      Mobileye_dat.Num_O = Num_O;
      Mobileye_dat.Timestamp = Timestamp;
      Mobileye_dat.L_Close = L_Close;
      Mobileye_dat.R_Close = R_Close;
      Mobileye_dat.GO = GO;
      Mobileye_dat.Close_Car = Close_Car;
      Mobileye_dat.L_Signal = L_Signal;
      Mobileye_dat.R_Signal = R_Signal;
      Mobileye_dat.Wiper = Wiper;
      Mobileye_dat.Lo_Beam = Lo_Beam;
      Mobileye_dat.Hi_Beam = Hi_Beam;
      Mobileye_dat.Speed = Speed;
      Mobileye_dat.Sign_Type = Sign_Type;
      Mobileye_dat.S_Sign_Type = S_Sign_Type;
      Mobileye_dat.Sign_X = Sign_X;
      Mobileye_dat.Sign_Y = Sign_Y;
      Mobileye_dat.Sign_Z = Sign_Z;
      Mobileye_dat.Filter_Type = Filter_Type;
      for (int i = 0; i < 10; i++){
          Mobileye_dat.O_ID[i] = O_ID[i];
                  Mobileye_dat.O_X[i] = O_X[i];
                  Mobileye_dat.O_Y[i] = O_Y[i];
                  Mobileye_dat.Blinker[i] = Blinker[i];
                  Mobileye_dat.Cut_IO[i] = Cut_IO[i];
                  Mobileye_dat.O_VX[i] = O_VX[i];
                  Mobileye_dat.O_Type[i] = O_Type[i];
                  Mobileye_dat.O_Status[i] = O_Status[i];
                  Mobileye_dat.O_Brake_Light[i] = O_Brake_Light[i];
                  Mobileye_dat.O_Valid[i] = O_Valid[i];
                  Mobileye_dat.O_Len[i] = O_Len[i];
                  Mobileye_dat.O_Wid[i] = O_Wid[i];
                  Mobileye_dat.O_Lane[i] = O_Lane[i];
                  Mobileye_dat.CIPV_Flag[i] = CIPV_Flag[i];
                  Mobileye_dat.O_Angle_Rate[i] = O_Angle_Rate[i];
                  Mobileye_dat.O_Scale_Change[i] = O_Scale_Change[i];
                  Mobileye_dat.O_AX[i] = O_AX[i];
                  Mobileye_dat.O_Replaced[i] = O_Replaced[i];
                  Mobileye_dat.O_Angle[i] = O_Angle[i];
      }
      /*memcpy(Mobileye_dat.O_ID,O_ID,sizeof(O_ID));
      memcpy(Mobileye_dat.O_X,O_X,sizeof(O_X));
      memcpy(Mobileye_dat.O_Y,O_Y,sizeof(O_Y));
      memcpy(Mobileye_dat.Blinker,Blinker,sizeof(Blinker));
      memcpy(Mobileye_dat.Cut_IO,Cut_IO,sizeof(Cut_IO));
      memcpy(Mobileye_dat.O_VX,O_VX,sizeof(O_VX));
      memcpy(Mobileye_dat.O_Type,O_Type,sizeof(O_Type));
      memcpy(Mobileye_dat.O_Status,O_Status,sizeof(O_Status));
      memcpy(Mobileye_dat.O_Brake_Light,O_Brake_Light,sizeof(O_Brake_Light));
      memcpy(Mobileye_dat.O_Valid,O_Valid,sizeof(O_Valid));
      memcpy(Mobileye_dat.O_Len,O_Len,sizeof(O_Len));
      memcpy(Mobileye_dat.O_Wid,O_Wid,sizeof(O_Wid));
      memcpy(Mobileye_dat.O_Lane,O_Lane,sizeof(O_Lane));
      memcpy(Mobileye_dat.CIPV_Flag,CIPV_Flag,sizeof(CIPV_Flag));
      memcpy(Mobileye_dat.O_Angle_Rate,O_Angle_Rate,sizeof(O_Angle_Rate));
      memcpy(Mobileye_dat.O_Scale_Change,O_Scale_Change,sizeof(O_Scale_Change));
      memcpy(Mobileye_dat.O_AX,O_AX,sizeof(O_AX));
      memcpy(Mobileye_dat.O_Replaced,O_Replaced,sizeof(O_Replaced));
      memcpy(Mobileye_dat.O_Angle,O_Angle,sizeof(O_Angle));*/
  }  
}


int main (int argc, char *argv[]){
  autoware_can_msgs::CANPacket candat;						// 사실 이건 필요없음. 선배들이 잘 모르고 넣어놓은 것 같음.

  ros::init(argc, argv, "can_converter");					// Node이름이 can_converter이다. 
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("can_raw", 1000, chatterCallback);
  ros::Publisher K7_pub = n.advertise<autoware_msgs::K7>("K7", 10);						// 토픽 이름이 /K7		이다. 
  ros::Publisher Mobileye_pub = n.advertise<autoware_msgs::Mobileye>("Mobileye", 10);	// 토픽 이름이 /Mobileye 이다. 
  ros::Publisher Niro_pub = n.advertise<autoware_msgs::Niro>("Niro", 10);				// 토픽 이름이 /Niro		이다.

  while(ros::ok()){  

    //cap >> img;
    //imshow("image", img);

    //resize(img, img, Size(320,240), 0, 0, INTER_CUBIC);

    //sprintf(savefile, "img_%d.jpg", count++);
    //imwrite(savefile, img);

    K7_pub.publish(k7_dat);									// [/k7] 이라는 토픽은 k7_dat구조체 내부의 내용들을 publish한다.
    Mobileye_pub.publish(Mobileye_dat);
    Niro_pub.publish(Niro_dat);
    ros::spinOnce();
  }
}
