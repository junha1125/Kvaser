/*
this node subscrive GPS data(latitude / longitude), CAN data(Mobileye / NiroCan), 
Lidar data(Point cloud data).Calculate using these data 
so that we get my vehicle&obstacle imformation 
and XY - coordinate which we can utilze on Uc - win Load.
*/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>			
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <autoware_msgs/gps_XY.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <autoware_msgs/car_infoArray.h>
#include <autoware_msgs/car_info.h>
#include <autoware_msgs/Mobileye_New.h> 
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define Ydistance 4
#define DELAYTIME 15

#define PORT_a 9991 
#define PORT_b 9992
#define PORT_c 9993
#define PORT_d 9994
#define PORT_e 9995
#define PORT_f 9996
#define PORT_g 9997

// Global variable For Topic Save 
autoware_msgs::gps_XY			gpsXYdata;
autoware_msgs::car_infoArray           carXYdatas;
autoware_msgs::Mobileye_New            MobileyeSub_topic;

// Global Variable For GPS -> UC-win my car XY
double Lat1=0;
double Lat2;
double Long1=0;
double Long2;
const double PI = 3.141592653589793;
const double Radius = 6378137;
double count = 0;
double sum_x=0;
double sum_y=0;
double temp_uc_X1 = 0;
double temp_uc_Y1 = 0;
double temp_uc_X2 = 0;
double temp_uc_Y2 = 0;
double temp_uc_X3 = 0;
double temp_uc_Y3 = 0;
double temp_uc_X4 = 0;
double temp_uc_Y4 = 0;
double temp_uc_X5 = 0;
double temp_uc_Y5 = 0;
double change = 0;


// Global variable For  Mobileye -> UC-win Obstacle XY
int Obstacle_ID[3];
unsigned int Obstacle_X[3];
int Obstacle_Y[3];
int O_Lane[3];
int CIPV_Flag[3];
double X1, Y1;
double X2, Y2;
double Obstacle_uc_X[3];
double Obstacle_uc_Y[3];  
bool Is_present[3] = {true, true, true};

/* Mobileye -> Finde Closest Obstacle */
void distance_order(int dist[5], int min_ind[3]){
	for(int i =0; i <5 ;i++) if(dist[i] == 0) dist[i] = 1000000000;

	
    int minindex = 0;

    for(int i =0; i <5 ;i++){
        if(dist[minindex] > dist[i]){
            minindex = i;
        }
    }
    if(dist[minindex] == 1000000000)  min_ind[0] = 10;
    else{
    dist[minindex] = 1000000000;
    min_ind[0] = minindex;
    }
  
    minindex = 0;
    for(int i =0; i <5 ;i++){
        if(dist[minindex] > dist[i]){
            minindex = i;
        }
    }
    if(dist[minindex] == 1000000000)  min_ind[1] = 10;
    else{
    dist[minindex] = 1000000000;
    min_ind[1] = minindex;
    }
    
    minindex = 0;
    for(int i =0; i <5 ;i++){
        if(dist[minindex] > dist[i] && i != min_ind[0] && i != min_ind[1]){
            minindex = i;
        }
    }
    if(dist[minindex] == 1000000000)  min_ind[2] = 10;
    else{
    dist[minindex] = 1000000000;
    min_ind[2] = minindex;
    }
}

// Mobile -> Uc win XY
void calcul_Obstacle(void){

    for(int i =0; i<3;i++){
    	Obstacle_uc_X[i] = 0.0;
    	Obstacle_uc_Y[i] = 0.0;
	Is_present[i] = true;
    }

    double theta = atan2(Y2-Y1,X2-X1); //라디안 단위
/*
    for(int i=0; i<3; i++){
	if(Obstacle_ID[i] != 0){
    	    if(O_Lane[i] ==1 && CIPV_Flag[i] == 1){
     	       Obstacle_uc_X[i] = X2 + Obstacle_X[i]*cos(theta);
     	       Obstacle_uc_Y[i] = Y2 + Obstacle_X[i]*sin(theta);
     	   }
     	   else if(Obstacle_Y > 0){
     	       Obstacle_uc_X[i] = X2 + Obstacle_X[i]*cos(theta) + Ydistance * sin(theta) ;
      	       Obstacle_uc_Y[i] = Y2 + Obstacle_X[i]*sin(theta) - Ydistance * cos(theta) ;
     	   }
     	   else if(Obstacle_Y < 0){
     	       Obstacle_uc_X[i] = X2 + Obstacle_X[i]*cos(theta) - Ydistance * sin(theta) ;
     	       Obstacle_uc_Y[i] = Y2 + Obstacle_X[i]*sin(theta) + Ydistance * cos(theta) ;
     	   }
	}
    }
*/


	
   	for(int i=0; i<3; i++){
    		  	if(O_Lane[i] ==1 && CIPV_Flag[i] == 1){
     	            		Obstacle_uc_X[0] = X2 + Obstacle_X[i]*cos(theta);
     	            		Obstacle_uc_Y[0] = Y2 + Obstacle_X[i]*sin(theta);
     	   		}
     	   		else if(Obstacle_Y[i] > 0){
     	       			Obstacle_uc_X[1] = X2 + Obstacle_X[i]*cos(theta) + Ydistance * sin(theta) ;
      	       			Obstacle_uc_Y[1] = Y2 + Obstacle_X[i]*sin(theta) - Ydistance * cos(theta) ;
     	   		}
     	   		else if(Obstacle_Y[i] < 0){
     	       			Obstacle_uc_X[2] = X2 + Obstacle_X[i]*cos(theta) - Ydistance * sin(theta) ;
     	       			Obstacle_uc_Y[2] = Y2 + Obstacle_X[i]*sin(theta) + Ydistance * cos(theta) ;
     	   		}
    	}

	if(Obstacle_uc_X[0] == 0){
		Obstacle_uc_X[0] = X2 + 10*cos(theta);
     		Obstacle_uc_Y[0] = Y2 + 10*sin(theta);
		Is_present[0] = false;
		
	}
	if(Obstacle_uc_X[1] == 0){
		Obstacle_uc_X[1] = X2 + 10*cos(theta) + Ydistance * sin(theta) ;
      	        Obstacle_uc_Y[1] = Y2 + 10*sin(theta) - Ydistance * cos(theta) ;
		Is_present[1] = false;
	}
	if(Obstacle_uc_X[2] == 0){
		Obstacle_uc_X[2] = X2 + 10*cos(theta) - Ydistance * sin(theta) ;
     	       	Obstacle_uc_Y[2] = Y2 + 10*sin(theta) + Ydistance * cos(theta) ;
		Is_present[2] = false;
	}
}


/* GPS Calculator */
double Cal_x(double Lat3, double Lon3, double Lat4, double Lon4)
{

	double Lat3_Rad = Lat3 * (PI / 180.0);
	double Lon3_Rad = Lon3 * (PI / 180.0);
	double Lat4_Rad = Lat4 * (PI / 180.0);
	double Lon4_Rad = Lon4 * (PI / 180.0);

	double dLat = 0;
	double dLon = Lon4_Rad - Lon3_Rad;

	double a = cos(Lat3_Rad) * cos(Lat4_Rad) * pow(sin(dLon / 2.0), 2.0);
	double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
	double d = Radius * c;

	//std::cout << d;

	return d;
}
double Cal_y(double Lat3, double Lon3, double Lat4, double Lon4)
{

	double Lat3_Rad = Lat3 * (PI / 180.0);
	double Lon3_Rad = Lon3 * (PI / 180.0);
	double Lat4_Rad = Lat4 * (PI / 180.0);
	double Lon4_Rad = Lon4 * (PI / 180.0);

	double dLat = Lat4_Rad - Lat3_Rad;
	double dLon = 0;

	double a = pow(sin(dLat / 2.0), 2.0);
	double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
	double d = Radius * c;
	
	//std::cout << "    " << d << std::endl;

	return d;
}



/*
Below is the code relating to the socket.
Each barrier has one socket function.
Each socket function sends an XY coordinate of the obstacle.
Transfer the location of 3 cars in front, 3 cars in back; 6 cars in total.
*/
/* Ros -> Window Data Send */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int call_socket(double currX, double currZ){
    

    /*variables for Socket a*/
    int sock_a;
    struct sockaddr_in addr_a;
    int recv_len_a;
    char value_a[50];
    char value_temp[20];
    const char comma[] = "\t";
    const char semiColon[2] = "\n";
    char Y[5] = "0";    
    char msg_a[1024];
    sprintf(value_a, "%.6f", currX);
    sprintf(value_temp, "%.6f", currZ);
    strcat(value_a, comma);
    strcat(value_a, value_temp);
    strcat(value_a, comma);
    strcat(value_a, Y); 
    strcat(value_a, semiColon); 

    /*set up Socket a */
    if((sock_a = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        perror("socket a ");
        return 1;
    }
	if(sock_a > 0){
        std::cout << " Socket create!! "<< std::endl;   
        }
        
	memset(&addr_a, 0x00, sizeof(addr_a));
	addr_a.sin_family = AF_INET;
	addr_a.sin_addr.s_addr = inet_addr("210.123.37.161");
	addr_a.sin_port = htons(PORT_a);

	if (connect(sock_a, (struct sockaddr*)&addr_a, sizeof(addr_a)) != -1) {
		std::cout << " Socket connecttttt!! "<< std::endl;
	}

        /*send/receive to/from Socket a*/
	write(sock_a, value_a, strlen(value_a));
	//recv_len_a = read(sock_a, msg_a, 1023);
	//msg_a[recv_len_a] = '\0';
	//printf("received data from socket a : %s\n", msg_a);
/*
	if ((recv_len_a = recv(sock_a, msg_a, 1024, 0)) < 0) {
		perror("recv a ");
		return 4;
	}

	msg_a[recv_len_a] = '\0';

	printf("received data from socket a : %s\n", msg_a);
*/
    /**/
    close(sock_a);

}

int call_socket1(double currX, double currZ){
    /*variables for Socket a*/
    int sock_a;
    struct sockaddr_in addr_a;
    int recv_len_a;
    char value_a[50];
    char value_temp[20];
    const char comma[] = "\t";
    const char semiColon[2] = "\n";
    char Y[5] = "0"; 
    char Y1[5] = "-10"; 
    char msg_a[1024];
    sprintf(value_a, "%.6f", currX);
    sprintf(value_temp, "%.6f", currZ);
    strcat(value_a, comma);
    strcat(value_a, value_temp);
    strcat(value_a, comma);
    if(Is_present[0]) strcat(value_a, Y); 
    else strcat(value_a, Y1);
    strcat(value_a, semiColon); 

    /*set up Socket a */
    if((sock_a = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        perror("socket a ");
        return 1;
    }
	if(sock_a > 0){
        std::cout << " Socket create!! "<< std::endl;   
        }
        
	memset(&addr_a, 0x00, sizeof(addr_a));
	addr_a.sin_family = AF_INET;
	addr_a.sin_addr.s_addr = inet_addr("210.123.37.161");
	addr_a.sin_port = htons(PORT_b);

	if (connect(sock_a, (struct sockaddr*)&addr_a, sizeof(addr_a)) != -1) {
		std::cout << " Socket connecttttt!! "<< std::endl;
	}
    /*send/receive to/from Socket a*/
	write(sock_a, value_a, strlen(value_a));
	//recv_len_a = read(sock_a, msg_a, 1023);
	//msg_a[recv_len_a] = '\0';
	//printf("received data from socket a : %s\n", msg_a);

    close(sock_a);
}

int call_socket2(double currX, double currZ){
/*variables for Socket a*/
    int sock_a;
    struct sockaddr_in addr_a;
    int recv_len_a;
    char value_a[50];
    char value_temp[20];
    const char comma[] = "\t";
    const char semiColon[2] = "\n";
    char Y[5] = "0"; 
    char Y1[5] = "-10";
    char msg_a[1024];
    sprintf(value_a, "%.6f", currX);
    sprintf(value_temp, "%.6f", currZ);
    strcat(value_a, comma);
    strcat(value_a, value_temp);
    strcat(value_a, comma);
    if(Is_present[1]) strcat(value_a, Y); 
    else strcat(value_a, Y1);
    strcat(value_a, semiColon); 

    /*set up Socket a */
    if((sock_a = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        perror("socket a ");
        return 1;
    }
	if(sock_a > 0){
        std::cout << " Socket create!! "<< std::endl;   
        }
        
	memset(&addr_a, 0x00, sizeof(addr_a));
	addr_a.sin_family = AF_INET;
	addr_a.sin_addr.s_addr = inet_addr("210.123.37.161");
	addr_a.sin_port = htons(PORT_c);

	if (connect(sock_a, (struct sockaddr*)&addr_a, sizeof(addr_a)) != -1) {
		std::cout << " Socket connecttttt!! "<< std::endl;
	}
    /*send/receive to/from Socket a*/
	write(sock_a, value_a, strlen(value_a));
	//recv_len_a = read(sock_a, msg_a, 1023);
	//msg_a[recv_len_a] = '\0';
	//printf("received data from socket a : %s\n", msg_a);

    close(sock_a);

}

int call_socket3(double currX, double currZ){
/*variables for Socket a*/
    int sock_a;
    struct sockaddr_in addr_a;
    int recv_len_a;
    char value_a[50];
    char value_temp[20];
    const char comma[] = "\t";
    const char semiColon[2] = "\n";
    char Y[5] = "0"; 
    char Y1[5] = "-10";
    char msg_a[1024];
    sprintf(value_a, "%.6f", currX);
    sprintf(value_temp, "%.6f", currZ);
    strcat(value_a, comma);
    strcat(value_a, value_temp);
    strcat(value_a, comma);
    if(Is_present[2]) strcat(value_a, Y); 
    else strcat(value_a, Y1); 
    strcat(value_a, semiColon); 

    /*set up Socket a */
    if((sock_a = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        perror("socket a ");
        return 1;
    }
	if(sock_a > 0){
        std::cout << " Socket create!! "<< std::endl;   
        }
        
	memset(&addr_a, 0x00, sizeof(addr_a));
	addr_a.sin_family = AF_INET;
	addr_a.sin_addr.s_addr = inet_addr("210.123.37.161");
	addr_a.sin_port = htons(PORT_d);

	if (connect(sock_a, (struct sockaddr*)&addr_a, sizeof(addr_a)) != -1) {
		std::cout << " Socket connecttttt!! "<< std::endl;
	}
    /*send/receive to/from Socket a*/
	write(sock_a, value_a, strlen(value_a));
	//recv_len_a = read(sock_a, msg_a, 1023);
	//msg_a[recv_len_a] = '\0';
	//printf("received data from socket a : %s\n", msg_a);

    close(sock_a);
}
/*If there is a vehicle behind my car, make the Y coordinate to -5. */
int call_socket4(double currX, double currZ, double currY){
    

    /*variables for Socket a*/
    int sock_a;
    struct sockaddr_in addr_a;
    int recv_len_a;
    char value_a[50];
    char value_temp[20];
    const char comma[] = "\t";
    const char semiColon[2] = "\n";
    char Y[2] = "0";
    char Y1[5] = "-5";
    char msg_a[1024];
    sprintf(value_a, "%.6f", currX);
    sprintf(value_temp, "%.6f", currZ);
    strcat(value_a, comma);
    strcat(value_a, value_temp);
    strcat(value_a, comma);
    if(currY == 0)
    {	strcat(value_a, Y1);
    }
    else
    {	strcat(value_a, Y); 
    }
    strcat(value_a, semiColon); 

    /*set up Socket a */
    if((sock_a = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        perror("socket a ");
        return 1;
    }
	if(sock_a > 0){
        std::cout << " Socket create!! "<< std::endl;   
        }
        
	memset(&addr_a, 0x00, sizeof(addr_a));
	addr_a.sin_family = AF_INET;
	addr_a.sin_addr.s_addr = inet_addr("210.123.37.161");
	addr_a.sin_port = htons(PORT_e);

	if (connect(sock_a, (struct sockaddr*)&addr_a, sizeof(addr_a)) != -1) {
		std::cout << " Socket connecttttt!! "<< std::endl;
	}

        /*send/receive to/from Socket a*/
	write(sock_a, value_a, strlen(value_a));
	//recv_len_a = read(sock_a, msg_a, 1023);
	//msg_a[recv_len_a] = '\0';
	//printf("received data from socket a : %s\n", msg_a);
/*
	if ((recv_len_a = recv(sock_a, msg_a, 1024, 0)) < 0) {
		perror("recv a ");
		return 4;
	}

	msg_a[recv_len_a] = '\0';

	printf("received data from socket a : %s\n", msg_a);
*/
    /**/
    close(sock_a);

}


int call_socket5(double currX, double currZ, double currY){
    

    /*variables for Socket a*/
    int sock_a;
    struct sockaddr_in addr_a;
    int recv_len_a;
    char value_a[50];
    char value_temp[20];
    const char comma[] = "\t";
    char semiColon[2] = "\n";
    char Y[2] = "0";
    char Y1[5] = "-5";
    char msg_a[1024];
    sprintf(value_a, "%.6f", currX);
    sprintf(value_temp, "%.6f", currZ);
    strcat(value_a, comma);
    strcat(value_a, value_temp);
    strcat(value_a, comma);
    if(currY == 0)
    {	strcat(value_a, Y1);
    }
    else
    {	strcat(value_a, Y); 
    }
    strcat(value_a, semiColon); 
    /*set up Socket a */
    if((sock_a = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        perror("socket a ");
        return 1;
    }
	if(sock_a > 0){
        std::cout << " Socket create!! "<< std::endl;   
        }
        
	memset(&addr_a, 0x00, sizeof(addr_a));
	addr_a.sin_family = AF_INET;
	addr_a.sin_addr.s_addr = inet_addr("210.123.37.161");
	addr_a.sin_port = htons(PORT_f);

	if (connect(sock_a, (struct sockaddr*)&addr_a, sizeof(addr_a)) != -1) {
		std::cout << " Socket connecttttt!! "<< std::endl;
	}

        /*send/receive to/from Socket a*/
	write(sock_a, value_a, strlen(value_a));
	//recv_len_a = read(sock_a, msg_a, 1023);
	//msg_a[recv_len_a] = '\0';
	//printf("received data from socket a : %s\n", msg_a);
/*
	if ((recv_len_a = recv(sock_a, msg_a, 1024, 0)) < 0) {
		perror("recv a ");
		return 4;
	}

	msg_a[recv_len_a] = '\0';

	printf("received data from socket a : %s\n", msg_a);
*/
    /**/
    close(sock_a);

}


int call_socket6(double currX, double currZ, double currY){
    

    /*variables for Socket a*/
    int sock_a;
    struct sockaddr_in addr_a;
    int recv_len_a;
    char value_a[50];
    char value_temp[20];
    const char comma[] = "\t";
    const char semiColon[2] = "\n";
    char Y[2] = "0";
    char Y1[5] = "-5";
    char msg_a[1024];
    sprintf(value_a, "%.6f", currX);
    sprintf(value_temp, "%.6f", currZ);
    strcat(value_a, comma);
    strcat(value_a, value_temp);
    strcat(value_a, comma);
    if(currY == 0)
    {	strcat(value_a, Y1);
    }
    else
    {	strcat(value_a, Y); 
    }
    strcat(value_a, semiColon); 

    /*set up Socket a */
    if((sock_a = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        perror("socket a ");
        return 1;
    }
	if(sock_a > 0){
        std::cout << " Socket create!! "<< std::endl;   
        }
        
	memset(&addr_a, 0x00, sizeof(addr_a));
	addr_a.sin_family = AF_INET;
	addr_a.sin_addr.s_addr = inet_addr("210.123.37.161");
	addr_a.sin_port = htons(PORT_g);

	if (connect(sock_a, (struct sockaddr*)&addr_a, sizeof(addr_a)) != -1) {
		std::cout << " Socket connecttttt!! "<< std::endl;
	}

        /*send/receive to/from Socket a*/
	write(sock_a, value_a, strlen(value_a));
	//recv_len_a = read(sock_a, msg_a, 1023);
	//msg_a[recv_len_a] = '\0';
	//printf("received data from socket a : %s\n", msg_a);
/*
	if ((recv_len_a = recv(sock_a, msg_a, 1024, 0)) < 0) {
		perror("recv a ");
		return 4;
	}

	msg_a[recv_len_a] = '\0';

	printf("received data from socket a : %s\n", msg_a);
*/
    /**/
    close(sock_a);

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Lidar -> Cluster Information Calculator */
double Center_x(double x, double y)
{	
	
	double c_x=1;
 	if(x>0)
	 { 	
		c_x = x+y/2;
		
}
	else
	 {
		 c_x = x-y/2;
	}
	//std::cout << " c_x: "<< c_x << std::endl;
	return c_x;
}
double Center_y(double x, double y)
{	
	double c_y=1;
 	if(x>0)
	 { 	
		 c_y = x+y/2;
}
	else
	 {
		 c_y = x-y/2;
}
        //std::cout << " c_y: "<< c_y << std::endl;
	return c_y;
}







/* Lidar Cluster Topic Callback function */

void CarCallback(const jsk_recognition_msgs::BoundingBoxArray& input)
{	

	/* Get car_info */
	for(size_t i =0; i < input.boxes.size(); i++)
	{ 	
	
		double temp_x = input.boxes[i].pose.position.x;
		double temp_dix = input.boxes[i].dimensions.x;
		double temp_y = input.boxes[i].pose.position.y;
		double temp_diy = input.boxes[i].dimensions.y;
 		
		/* Limit car dimension */
		if(temp_dix > 6.0 || temp_diy > 2.10 || temp_diy < 1.5 ) 
		{
			change = 0; //Hide the car in UC_Win/Road by zeroing the change when there is no car in the specified conditions.
			continue;
		
		} 
		
		/* Car Center */
		double center_x = Center_x(temp_x,temp_dix); 
		double center_y = Center_y(temp_y,temp_dix);


		/* Adjust the Lidar data */
		if(center_y > 8.5 || center_y < -8.5 || center_x > 0 || center_x <-10  ) 
		{	
			change = 0;		
			continue;
			
		} 
		
		/* Remove Lidar area */
		if(center_x >-1.55 && center_x < 0) 
		{
			change = 0;
			continue;
		}

		/*Reverse Lidar coordinate to fit UC_coordinate */
		center_y = -center_y;  


		std::cout << "New car info \n" << std::endl;

		std::cout << "car_x: " << center_x << '\n';
		std::cout << "car_y: " << center_y << '\n';		


		std::cout << "After adding relative coordinate" << std::endl;	
		

		/*Adding relative coordinate */
		double theta = atan2(temp_uc_Y2-temp_uc_Y1, temp_uc_X2-temp_uc_X1); 
		/* When there is a car */
		change++;

		/* When the car is on the right */
		if(center_y >1.0)
		{	temp_uc_X5 = temp_uc_X2 + center_x*cos(theta) + Ydistance * sin(theta);
			temp_uc_Y5 = temp_uc_Y2 + center_x*sin(theta) - Ydistance * cos(theta);


			temp_uc_X4 = temp_uc_X2 + 10*cos(theta);	
			temp_uc_Y4 = temp_uc_Y2 + 10*sin(theta); 

			temp_uc_X3 = temp_uc_X2 + 10*cos(theta) - Ydistance * sin(theta);
			temp_uc_Y3 = temp_uc_Y2 + 10*sin(theta) + Ydistance * cos(theta);  
	
		

		
		}
		/* When the car is on the left */
		else if(center_y < -1.0)
		{	temp_uc_X3 = temp_uc_X2 + center_x*cos(theta) - Ydistance * sin(theta);
			temp_uc_Y3 = temp_uc_Y2 + center_x*sin(theta) + Ydistance * cos(theta); 
				
		        temp_uc_X4 = temp_uc_X2 + 10*cos(theta);	
			temp_uc_Y4 = temp_uc_Y2 + 10*sin(theta); 

			temp_uc_X5 = temp_uc_X2 + 10*cos(theta) + Ydistance * sin(theta);
			temp_uc_Y5 = temp_uc_Y2 + 10*sin(theta) - Ydistance * cos(theta);  
			


			
		}	
		/* When the car is in the middle */
		else
		{	temp_uc_X4 = temp_uc_X2 + center_x*cos(theta);	
			temp_uc_Y4 = temp_uc_Y2 + center_x*sin(theta);
		
			temp_uc_X3 = temp_uc_X2 + 10*cos(theta) + Ydistance * sin(theta);
			temp_uc_Y3 = temp_uc_Y2 + 10*sin(theta) - Ydistance * cos(theta);  
			
                     
			temp_uc_X5 = temp_uc_X2 + 10*cos(theta) - Ydistance * sin(theta);
			temp_uc_Y5 = temp_uc_Y2 + 10*sin(theta) + Ydistance * cos(theta);  
			

		} 
	
		/* Create car_info */ 
		autoware_msgs::car_info	 carXYdata;
		carXYdata.header = input.boxes[i].header;
		carXYdata.car_x = center_x;
		carXYdata.car_y = center_y;

	
		//std::cout << "Center of car_x: " << center_x << '\n';
		//std::cout << "Center of car_y: " << center_y << '\n';
		carXYdatas.cars.push_back(carXYdata);		
		
		change = 0;
		
		}
		
std::cout << "Frame end \n" << std::endl; ;
}







/* GPS Topic Callback function */
void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	gpsXYdata.header = msg->header; 
	double gps_lat = msg->latitude;
	double gps_long = msg->longitude;

	double Lat2 = gps_lat;
	double Long2 = gps_long;
	gpsXYdata.Is_change = 1;
	


	if (Lat1 == 0)
	{
		gpsXYdata.uc_x = Cal_x(0, 0, Lat2, Long2);
		gpsXYdata.uc_y = Cal_y(0, 0, Lat2, Long2);
	}
	else
	{
		gpsXYdata.uc_x = Cal_x(Lat1, Long1, Lat2, Long2);
		gpsXYdata.uc_y = Cal_y(Lat1, Long1, Lat2, Long2);
		

		if(gpsXYdata.uc_x == 0 && gpsXYdata.uc_y == 0) {
			gpsXYdata.Is_change = 0;
			//call_socket(uc_y);
		}

		// If all conditions are met, execute the following code. Then, the data is transferred using a socket.
		if(gpsXYdata.Is_change == 1){
			if(fmod(count,DELAYTIME)==0  && count > 49){
               			X2 = sum_x;
                		Y2 = sum_y;

				temp_uc_X2 = X2;
				temp_uc_Y2 = Y2;


                		calcul_Obstacle();
				std::cout <<  "-------------" << std::endl;
				std::cout << "my x : " << sum_x << "     my y : " << sum_y << std::endl; 
				std::cout << "uc obstacle X : " << "1. " <<  Obstacle_uc_X[0] << "          2. " <<  Obstacle_uc_X[1] << "          3. " << Obstacle_uc_X[2] << std::endl;
				std::cout << "uc obstacle Y : " << "1. " <<  Obstacle_uc_Y[0] << "          2. " <<  Obstacle_uc_Y[1] << "          3. " << Obstacle_uc_Y[2] << std::endl;  
				std::cout << (int)Is_present[0] << (int)Is_present[1] << (int)Is_present[2] << std::endl;
				
				std::cout << " Socket Start!! "<< std::endl;
				int tTemp = call_socket(sum_x, sum_y);
				int tTemp1 = call_socket1(Obstacle_uc_X[0], Obstacle_uc_Y[0]);
				int tTemp2 = call_socket2(Obstacle_uc_X[1] ,Obstacle_uc_Y[1]);
				int tTemp3 = call_socket3(Obstacle_uc_X[2], Obstacle_uc_Y[2]);
				/* Using Socket and Send Obstacle_uc_X[0] Obstacle_uc_Y[0]  // Obstacle_uc_X[1] Obstacle_uc_Y[1]  // Obstacle_uc_X[2] Obstacle_uc_Y[2]*/

				/*When car on the road */
				if(change != 0) 
				{
					double car_z = 1;
					int iTemp4 = call_socket4(temp_uc_X3,temp_uc_Y3, car_z);
					int iTemp5 = call_socket5(temp_uc_X4,temp_uc_Y4, car_z);
					int iTemp6 = call_socket6(temp_uc_X5,temp_uc_Y5, car_z);
				}
				
				/*When there is no car on the road  */
				else 
				{
				double theta = atan2(temp_uc_Y2-temp_uc_Y1, temp_uc_X2-temp_uc_X1);

				double C_X3 = X2 - 10*cos(theta);
				double C_Y3 = Y2 + 10*sin(theta);
				
				double C_X4 = C_X3 - 4*cos(90-theta);
				double C_Y4 = C_Y3 + 4*sin(90-theta);
		
				double C_X5 = C_X3 + 4*cos(90-theta);
				double C_Y5 = C_Y3 - 4*sin(90-theta);
				
				double car_z1 = 0;

					int iTemp4 = call_socket4(C_X3,C_Y3,car_z1);   //Center line
					int iTemp5 = call_socket5(C_X4,C_Y4,car_z1);	//Left line
					int iTemp6 = call_socket6(C_X5,C_Y5,car_z1);	//Right line
				}


				std::cout << " Socket End!! \n"<< std::endl;

				X1 = sum_x;
                		Y1 = sum_y;
				temp_uc_X1 = X1;
				temp_uc_Y1 = Y1;
			}
			sum_x += gpsXYdata.uc_x;
			sum_y += gpsXYdata.uc_y;

			if(count < 20 ){
				gpsXYdata.uc_x = 0.000001;
				gpsXYdata.uc_y = 0.000001;
			}	
			else{
				gpsXYdata.uc_x = sum_x;
				gpsXYdata.uc_y = sum_y;
			}	
		

			std::cout.precision(15);
			std::cout << "lat1 : " << Lat1 << "   lat2 : " << Lat2 << '\n'  ;
			std::cout << "long1 : " << Long1 << "   long2 : " << Long2 << "                count : " << count++ <<  "\n" ;
			std::cout << "x값 : " << gpsXYdata.uc_x << "   y값 :  " << gpsXYdata.uc_y << std::endl << std::endl;
	
		}
	}
	
	Lat1 = gps_lat;
	Long1 = gps_long;

	//sleep();

	
}


/* Acceleration Topic Callback function */
void chatterCallback_IMU(const sensor_msgs::Imu::ConstPtr& msg){
	gpsXYdata.accel_x = (msg->linear_acceleration).x;
	gpsXYdata.accel_y = (msg->linear_acceleration).y;
	gpsXYdata.accel_z = (msg->linear_acceleration).z;
}


/* Mobileye Topic Callback function */
void CarCallback_Mobileye(const autoware_msgs::Mobileye_New ::ConstPtr& msg){
     int distance[5];
     int minindex[3];

     for(int i = 0; i<5; i++) distance[i] = sqrt( pow((msg->O_X)[i],2) + pow((msg->O_Y)[i],2) );
     distance_order(distance, minindex);

     for(int i = 0; i<3; i++){
	if(minindex[i] != 10){
        	Obstacle_ID[i] = (msg->O_ID)[ minindex[i] ];
        	Obstacle_X[i] = (msg->O_X)[ minindex[i] ];
        	Obstacle_Y[i] = (msg->O_Y)[ minindex[i] ];
        	O_Lane[i] = (msg->O_Lane)[ minindex[i] ];
        	CIPV_Flag[i] = (msg->CIPV_Flag)[ minindex[i] ];

	}
	else{
		Obstacle_ID[i] = 0;
        	Obstacle_X[i] = 0;
        	Obstacle_Y[i] = 0;
        	O_Lane[i] = 0;
        	CIPV_Flag[i] = 0;
	}

	
     }
}





/* Main start */
int main (int argc, char *argv[]){
  

  ros::init(argc, argv, "GPS_tude_to_XY");					// Node name is GPS_tude_to_XY 
  ros::NodeHandle n;
  ros::Subscriber sub_mobileye = n.subscribe("/Mobileye_New", 1, CarCallback_Mobileye);
  ros::Subscriber sub = n.subscribe("/xsens/gps_data", 1, chatterCallback);	
  ros::Subscriber sub_imu = n.subscribe("/xsens/imu", 1, chatterCallback_IMU);
  ros::Subscriber sub_car = n.subscribe("/bounding_boxes", 1, CarCallback);	
  ros::Publisher raw_pub = n.advertise<autoware_msgs::gps_XY>("gps_XY", 1);
  ros::Publisher car_pub = n.advertise<autoware_msgs::car_infoArray>("Car_Info", 1);			
  while(ros::ok()){  
    raw_pub.publish(gpsXYdata);	
    car_pub.publish(carXYdatas);	
    ros::spinOnce();

  }
}
