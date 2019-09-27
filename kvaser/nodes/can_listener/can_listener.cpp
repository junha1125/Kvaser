/*
**                Copyright 2012 by Kvaser AB, M�lndal, Sweden
**                        http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ===============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ===============================================================================
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** ---------------------------------------------------------------------------
**/

/*
 * Kvaser Linux Canlib
 * Read CAN messages and print out their contents
 */

#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <ros/ros.h>
#include "autoware_can_msgs/CANPacket.h"

int i = 0;
unsigned char willExit = 0;
int last;
time_t last_time = 0;			 
// [time_t]   ->   [__time64_t]   ->   [__int64]   ->   [long (Integer 8 Byte )]


// sig을 매개변수로 받고, 그 매개변수가 2라면, willExit라는 전역변수를 1로 바꾼다 즉 곧 프로그램 종료!
void sighand (int sig)
{
  switch (sig) {
  case SIGINT: //   #define SIGINT   2   // interrupt 발생시. 
    willExit = 1;
    alarm(0);
    break;
  }
}

 
int main (int argc, char *argv[])

{
  canHandle h;								// NodeHandle과 비슷한 CanHandle이다.
  int ret = -1;								// 함수의 return값을 저장하여 -1과같은 값이 저장되어있다면 EXIT한다

  // 아래의 변수들은 [/can_raw 구조체] 내부의 변수들에 들어갈 내용들을, 이전에 잠시 저장해 놓을 변수들이다. 
  long id; 
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long t;
  int channel = 0;
  int bitrate = BAUD_500K;					// #define  BAUD_500K  500,000이 되어 있을 듯 하다. 
  int j;
  autoware_can_msgs::CANPacket candat;		// /can_raw와 동일한 구조체를 가지는 객체를 생성해 놓음 

  ros::init(argc, argv, "can_listener");	// can_listenener라는 Node의 시작!!
  ros::NodeHandle n;						// 앞으로 이 Node의 Handle은 n이다. 이것으로 publish할 예정
 
  ros::Publisher can_pub = n.advertise<autoware_can_msgs::CANPacket>("can_raw", 10); // NodeHandle이용
  //topic이름이 /can_raw가 되겠군. 토픽 한줄의 구조체 모양은 원노트의 그림을 참조한다. 

  errno = 0;
  if (argc != 2 || (channel = atoi(argv[1]), errno) != 0) { // 예외(에러) 검출
    printf("usage %s channel\n", argv[0]);
    exit(1);
  } else {
    printf("Reading messages on channel %d\n", channel);
  }

  /* Use sighand as our signal handler */					//  예외(에러) 검출
  signal(SIGALRM, sighand); // 첫번쨰 매개변수는 상수 두번째 매개변수는 함수 포인터.
  signal(SIGINT, sighand);
  alarm(1);

  /* Allow signals to interrupt syscalls(in canReadBlock) */ // 예외(에러) 검출
  siginterrupt(SIGINT, 1);
  
  /* Open channels, parameters and go on bus */		// kvaser와 연결하고 can Open!!
  h = canOpenChannel(channel, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED); 
  if (h < 0) {												 // 예외(에러) 검출
    printf("canOpenChannel %d failed\n", channel);
    return -1;
  }

  // canHandle h내부 매개변수들을(객체의 맴버변수)의 내용을 바꿔주고, canBusOn!!
  canSetBusParams(h, bitrate, 4, 3, 1, 1, 0);	
  canSetBusOutputControl(h, canDRIVER_NORMAL);
  canBusOn(h);

  i = 0;

  //  while (!willExit) { 
  while(ros::ok()){									// 어떤 에러와 예외가 없다면 무한반복!
    do { 
      ret = canReadWait(h, &id, &msg, &dlc, &flag, &t, -1); 
	  // canReadWait라는 함수를 사용해서 위에서 선언한 지역변수들의 값을 모두 바꿔준다. 
	  // 차에서 들어온 can raw data들이 위의 지역변수에 저장되는 것이고, 
	  // 아래에 지역변수들의 내용을 candat 객체 [/can_raw구조체] 내부 변수들에 모두 대입한 후
	  // candata를 topic으로 publish할 예정이다.
      switch (ret) {
      case 0: // canReadWait를 정상적으로 수행했다면 return값으로 ret에 0이 저장된다. 
        printf("(%d) id:%ld dlc:%d data: ", i, id, dlc); 
        if (dlc > 8) {
          dlc = 8;			// dlc는 data Length를 의미하는 것인가?
        }
        for (j = 0; j < dlc; j++){
          printf("%2.2x ", msg[j]);
	      candat.dat[j]=msg[j];
		}
        printf(" flags:0x%x time:%ld\n", flag, t);
		candat.count =i;
		candat.time=t;
		candat.id = id;
		candat.len=dlc;
		candat.header.stamp=ros::Time::now();
		can_pub.publish(candat);
		ros::spinOnce();
		i++; // i로 몇개의 /can_raw구조체 data를 받았는지 셀 수 있다

		// last와 last_time모두 Global 변수 이다. 
		if (last_time == 0) {
		  last_time = time(0);
		} 
		else if (time(0) > last_time) {
			last_time = time(0);
			if (i != last) {
				printf("rx : %d total: %d\n", i - last, i);
			}
			last = i;
		}
        break; // switch문을 빠져나와서, do_while문을 반복한다. 
		

		// 두가지 case모두 error가 감지됐을때
		case canERR_NOMSG:
        break;
		
		default:  // 위의 2가지 case가 모두 아니라면 == (Matlab) otherwise
        perror("canReadBlock error");
        break;
      }
    } while (ret == canOK); // 현재 상태가 정상이 아니라면 아래의 문장을 처리하고 EXIT !!
    willExit = 1;
  }
   
  canClose(h);
   
  sighand(SIGALRM);
  printf("END Ready\n");

  return 0;
}





