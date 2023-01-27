#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>

//serial com
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
//multi thread
#include <pthread.h>
//string
#include <string>
//cos,sin
#include <math.h>

#define SERIAL_DEVICE_L  "/dev/ttyUSB0"  // ttyHS0, ttyHS1, ttyHS3 are available
#define SERIAL_DEVICE_R  "/dev/ttyUSB1"  // ttyHS0, ttyHS1, ttyHS3 are available


/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

#define _USE_MATH_DEFINES

#define POLYNORMIAL 0xA001
#define ENCODER_PULSE_REV 4096.
#define MAX_MOTOR_SPEED_RPM 80

#define NO_OF_MOTOR 1
#define L_MOTOR_ID  1
#define R_MOTOR_ID  2

#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 


#define BAUDRATE        B115200


struct ComDev
{
    int SerialCom;
    int send_update_flag =0;
    int nwrite  = 0;
    
    unsigned char readbuff[9]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char writebuff[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    int motor_command = 0;      // 1 : set motor rpm
                                // 2 : encoder read  
    
} myComDev,myComDevL,myComDevR;

struct BaseSensorData
{
    int delta_encode_left = 0; 
    int delta_encode_right = 0 ;
    int encode_left = 0; 
    int encode_right = 0; 
} myBaseSensorData;

struct OdomCaculateData
{
    //motor params
    float speed_ratio=(67.6*3.14159/1000)/2200 ;//0.000176; //unit: m/encode  67.5 
    float wheel_distance= 0.226; //0.1495; //unit: m
    float wheel_diameter = 170;  // 170 mm    
    float encode_sampling_time=0.04; //unit: s
    float cmd_vel_linear_max=0.8; //unit: m/s
    float cmd_vel_angular_max=0.0; //unit: rad/s
    //odom result
    float position_x=0.0; //unit: m
    float position_y=0.0; //unit: m
    float oriention=0.0; //unit: rad
    float velocity_linear=0.0; //unit: m/s
    float velocity_angular=0.0; //unit: rad/s
    
}myOdomCaculateData;

struct MyMotorRPM
{
 int  left_motor_rpm = 0; 
 int right_motor_rpm = 0;
}myMotorRPM;


typedef unsigned char BYTE;

typedef struct
{
  BYTE byLow;
	BYTE byHigh;
} lByte;


struct pollfd  poll_events;      // 체크할 event 정보를 갖는 struct
int    poll_state;
//unsigned char protocal_test[12] ={0,};



#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 


void Init_COM_L(void)
{
  char fd_serial[20];
  struct termios oldtio, newtio;
  
   
  int    poll_state;
  int    uart_fd1;
  int r;
  
  // UART configuration
  strcpy(fd_serial, SERIAL_DEVICE_L); //FFUART
  
  
  myComDev.SerialCom = open(fd_serial, O_RDWR | O_NOCTTY );
  if (myComDev.SerialCom <0) 
  {
    printf("Serial %s  Device Err\n", fd_serial );
    exit(1);
  }
  //printf("CarControlInit(void) Uart Device : %s\n", SERIAL_DEVICE);
  
  
   memset(&newtio, 0, sizeof(newtio) );
   newtio.c_cflag       = BAUDRATE| CS8 | CLOCAL | CREAD;
   newtio.c_oflag       = 0;
   newtio.c_lflag       = 0;
   newtio.c_cc[VTIME]   = 0;
   newtio.c_cc[VMIN]    = 1;
   
   tcflush(myComDev.SerialCom, TCIFLUSH);
   tcsetattr(myComDev.SerialCom, TCSANOW, &newtio);
   fcntl(myComDev.SerialCom, F_SETFL, FNDELAY);    

   // poll 사용을 위한 준비   
   poll_events.fd        = myComDev.SerialCom;
   poll_events.events    = POLLIN | POLLERR;          // 수신된 자료가 있는지, 에러가 있는지
   poll_events.revents   = 0;
  
}

void Init_COM_R(void)
{
  char fd_serial[20];
  struct termios oldtio, newtio;
  
   
  int    poll_state;
  int    uart_fd1;
  int r;
  
  // UART configuration
  strcpy(fd_serial, SERIAL_DEVICE_L); //FFUART
  
  
  myComDev.SerialCom = open(fd_serial, O_RDWR | O_NOCTTY );
  if (myComDev.SerialCom <0) 
  {
    printf("Serial %s  Device Err\n", fd_serial );
    exit(1);
  }
  //printf("CarControlInit(void) Uart Device : %s\n", SERIAL_DEVICE);
  
  
   memset(&newtio, 0, sizeof(newtio) );
   newtio.c_cflag       = BAUDRATE| CS8 | CLOCAL | CREAD;
   newtio.c_oflag       = 0;
   newtio.c_lflag       = 0;
   newtio.c_cc[VTIME]   = 0;
   newtio.c_cc[VMIN]    = 1;
   
   tcflush(myComDev.SerialCom, TCIFLUSH);
   tcsetattr(myComDev.SerialCom, TCSANOW, &newtio);
   fcntl(myComDev.SerialCom, F_SETFL, FNDELAY);    

   // poll 사용을 위한 준비   
   poll_events.fd        = myComDev.SerialCom;
   poll_events.events    = POLLIN | POLLERR;          // 수신된 자료가 있는지, 에러가 있는지
   poll_events.revents   = 0;
  
}

unsigned short CheckSum_CRC16(unsigned char *packet, short len)
{
	int i;
	unsigned short crc, flag;
	
	crc = 0xffff;
	
	while(len--)
	{
		crc ^= *(packet++);
		
		for(i=0;i<8;i++)
		{
			flag = crc & 0x0001;
			crc >>= 1;
			if (flag) crc ^= POLYNORMIAL;
		}
		
	}
	return crc;	
}


void write_serial(unsigned char *buf, int len)
{  
  while(myComDev.send_update_flag != 0);//wait for flag clear 
  myComDev.send_update_flag = 1;  
}    


void read_serial_data(short no_data)
{
	unsigned char buf[100]={0x00,};
	int i, cnt;
	int read_data = 0 ;
  while (1)
  {
      poll_state = poll(                                // poll()을 호출하여 event 발생 여부 확인     
                         (struct pollfd*)&poll_events,  // event 등록 변수
                                                    1,  // 체크할 pollfd 개수
                                                  400  // time out 시간
                       );

      if (poll_state > 0)                             // 발생한 event 가 있음
      {     
         if ( poll_events.revents & POLLIN)            // event 가 자료 수신?
         {
            cnt = read(myComDev.SerialCom, buf, 40);
           
            read_data += cnt;
            
             if(read_data>=no_data) break;
           /*for(i=0;i<cnt;i++)
            {
              printf( "data received - %d %d %x\n", read_data, i, buf[i]);
		     }
	          */   
         }
         if ( poll_events.revents & POLLERR)      // event 가 에러?
         {
            printf( "통신 라인에 에러가 발생, 프로그램 종료");
            break;
         }
      }
      else if(poll_state < 0)
      {
          printf("Poll Critial Error!\n");
          break;
      }
      else if(poll_state == 0)
      {
          printf("wait...\n");
         
      }
   }
   
   for(i=0;i<8;i++)  myComDev.readbuff[i] = buf[i];
	
}

lByte Int2Byte(short nln)
{
	lByte Ret;
	
	Ret.byLow = nln & 0xff;
	Ret.byHigh = nln >> 8 & 0xff;
	return Ret;
}

void set_profile_velocity_mode(BYTE moto_id)
{
	
	unsigned short protocal_crc16;
	
	myComDev.writebuff[0] = moto_id;
  myComDev.writebuff[1] = 0x06;
  myComDev.writebuff[2] = 0x20;
  myComDev.writebuff[3] = 0x32;
  myComDev.writebuff[4] = 0x00;
  myComDev.writebuff[5] = 0x03;
    
  protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
  myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
  myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
	//printf("protocal CRC16 %X \n", protocal_crc16);
	myComDev.motor_command = 1;
	write_serial(myComDev.writebuff,8);
	
}


void motor_set_s_curver_accleration(BYTE moto_id,short time_ms)
{
	lByte two_byte;
	unsigned short protocal_crc16;
	
	myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x06;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x37;
    
    two_byte = Int2Byte(time_ms);
    myComDev.writebuff[4] = (char)two_byte.byHigh;
    myComDev.writebuff[5] = (char)two_byte.byLow;
    
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
	//printf("protocal CRC16 %X \n", protocal_crc16);
	
	write_serial(myComDev.writebuff,8);	
}

void motor_set_s_curver_decleration(BYTE moto_id,short time_ms)
{
	lByte two_byte;
	unsigned short protocal_crc16;
	
	myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x06;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x38;
    
    two_byte = Int2Byte(time_ms);
    myComDev.writebuff[4] = (char)two_byte.byHigh;
    myComDev.writebuff[5] = (char)two_byte.byLow;
    
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
	//printf("protocal CRC16 %X \n", protocal_crc16);
	
    write_serial(myComDev.writebuff,8);	
}

void motor_enable(BYTE moto_id)
{
	
   unsigned short protocal_crc16;
	
    myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x06;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x31;
    myComDev.writebuff[4] = 0x00;
    myComDev.writebuff[5] = 0x08;
    
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
    write_serial(myComDev.writebuff,8);	
}

void init_motor_encoder(BYTE moto_id)
{
	
    unsigned short protocal_crc16;
    
    myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x06;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x05;
    myComDev.writebuff[4] = 0x00;
    myComDev.writebuff[5] = 0x01;
    
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
	//printf("protocal CRC16 %X \n", protocal_crc16);
	
	  write_serial(myComDev.writebuff,8);	
}

void set_max_motor_rpm(BYTE moto_id,short max_rpm)
{
    lByte two_byte;
    unsigned short protocal_crc16;
	
    myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x06;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x0A;
    
    two_byte = Int2Byte(max_rpm);
    myComDev.writebuff[4] = (char)two_byte.byHigh;
    myComDev.writebuff[5] = (char)two_byte.byLow;
        
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
    //printf("protocal CRC16 %X \n", protocal_crc16);	
    write_serial(myComDev.writebuff,8);	
}

void set_target_speed(BYTE moto_id,short speed)
{
	
    lByte two_byte;
    unsigned short protocal_crc16;
	
    myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x06;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x3A;
    
    two_byte = Int2Byte(speed);
    myComDev.writebuff[4] = (char)two_byte.byHigh;
    myComDev.writebuff[5] = (char)two_byte.byLow;
        
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
	//printf("protocal CRC16 %X \n", protocal_crc16);
	
	  write_serial(myComDev.writebuff,8);
}

void clear_alarm(BYTE moto_id)
{
    lByte two_byte;
    unsigned short protocal_crc16;
    unsigned char buf[8]={0x00,};
	
    myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x06;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x31;
    
    myComDev.writebuff[4] = 00;
    myComDev.writebuff[5] = 06;
        
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
    //printf("clear alarm protocal CRC16 %X \n", protocal_crc16);
	
    write_serial(myComDev.writebuff,8);
  //  read(uart_fd, buf,8);
}

void interrupt_motor_enable(BYTE moto_id)
{
    lByte two_byte;
    unsigned short protocal_crc16;
    unsigned char buf[8]={0x00,};
	
    myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x06;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x31;
    
    myComDev.writebuff[4] = 00;
    myComDev.writebuff[5] = 07;
        
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
	//printf("protocal CRC16 %X \n", protocal_crc16);
	
    write_serial(myComDev.writebuff,8);

}

void read_motor_position(BYTE moto_id)
{
    int motor_position = 0; 
    unsigned char buf[20]={0x00,};
    int i, cnt;
    int read_data = 0 ; 
	
    unsigned short protocal_crc16;

    myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x03;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x2a;
    myComDev.writebuff[4] = 0x00;
    myComDev.writebuff[5] = 0x02;
    
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	  
    myComDev.motor_command = 2;
    write_serial(myComDev.writebuff,8);		
	
}

int cal_motor_position(void)
{
   int motor_position = 0; 
   motor_position = (short)myComDev.readbuff[3]*16777216 +(short)myComDev.readbuff[4]*65536 + (short)myComDev.readbuff[5]*256 + (short)myComDev.readbuff[6];  
   
   //printf("motor pulse %d  %X %X %X %X\n", motor_position ,myComDev.readbuff[3],myComDev.readbuff[4],myComDev.readbuff[5],myComDev.readbuff[6]);
   
   myComDev.motor_command = 0;
   return motor_position;
}


short read_motor_speed(BYTE moto_id)
{
    short motor_rpm = 0; 
    unsigned char buf[20]={0x00,};
    int i, cnt;
    int read_data = 0 ;
	
    unsigned short protocal_crc16;
	
    myComDev.writebuff[0] = moto_id;
    myComDev.writebuff[1] = 0x03;
    myComDev.writebuff[2] = 0x20;
    myComDev.writebuff[3] = 0x2c;
    myComDev.writebuff[4] = 0x00;
    myComDev.writebuff[5] = 0x01;
    
    protocal_crc16 = CheckSum_CRC16(myComDev.writebuff, 6);
    myComDev.writebuff[6] = (unsigned char)((protocal_crc16 >>0) & 0x00ff);
    myComDev.writebuff[7] = (unsigned char)((protocal_crc16 >>8) & 0x00ff);
	
    //printf("protocal CRC16 %X \n", protocal_crc16);
	
        
    write(myComDev.SerialCom, &buf[0], 8);
    
	
	
    while (1)
    {
      poll_state = poll(                                // poll()을 호출하여 event 발생 여부 확인     
                         (struct pollfd*)&poll_events,  // event 등록 변수
                                                    1,  // 체크할 pollfd 개수
                                                  50  // time out 시간
                       );

      if (poll_state > 0)                             // 발생한 event 가 있음
      {     
         if ( poll_events.revents & POLLIN)            // event 가 자료 수신?
         {
            cnt = read(myComDev.SerialCom, buf, 20);
           
            read_data += cnt;
            
            if(read_data>=7) 			  break;			  
		    
		  
           /*
            for(i=0;i<cnt;i++)
            {
              printf( "data received - %d %d %x\n", read_data, i, buf[i]);
		     }
		   */
         }
         if ( poll_events.revents & POLLERR)      // event 가 에러?
         {
            printf( "통신 라인에 에러가 발생, 프로그램 종료");
            break;
         }
      }
      else if(poll_state < 0)
      {
          printf("Critial Error!\n");
          break;
      }
      else if(poll_state == 0)
      {
          printf("wait...\n");
          if(read_data>=7) 
          {
			  
			  break;			  
	  }
      }
   }
	motor_rpm = (short)buf[4] +(short)buf[3]*256;
	myComDev.send_update_flag = 0;
	//printf("motor rpm %d %X %X\n", motor_rpm, buf[3],buf[4]);
	return motor_rpm;
	
}



void init_motor_setup(void)
{
	int i;
	
	for(i=1;i<=NO_OF_MOTOR;i++)
  {
	   set_profile_velocity_mode(i);
     //read_serial_data(8);
     //printf("Set Profile Velocity Mode\n");		
      
     motor_set_s_curver_accleration(i,1000);
  // read_serial_data(8);
     //printf("Set Profile S-curve Accleration\n");
       
     motor_set_s_curver_decleration(i,1000);       
  // read_serial_data(8);
     //printf("Set Profile S-curve Decleration\n");	      
       
     set_max_motor_rpm(i,MAX_MOTOR_SPEED_RPM);
  //   read_serial_data(8);	
       
     init_motor_encoder(i);
  //   read_serial_data(8);       
          
     ROS_INFO("Motor[%d] Init",i);	     
    }   
     
}

void robot_move(struct MyMotorRPM rpm_motor)
{
	int i;
  
	for(i=1;i<=NO_OF_MOTOR;i++)
	{
    ///  motor_enable(i);
  
	}
	
  
  //ros::Duration(0.005).sleep(); //delay 10ms
  motor_enable(L_MOTOR_ID);
	set_target_speed(L_MOTOR_ID, rpm_motor.left_motor_rpm );
  
  motor_enable(R_MOTOR_ID);
  set_target_speed(R_MOTOR_ID, -rpm_motor.right_motor_rpm);
  
	
}
void callback(const geometry_msgs::Twist & cmd_input)
{
    float angular_temp;
    float linear_temp;
    linear_temp = cmd_input.linear.x ;//m/s
    angular_temp = cmd_input.angular.z ;//rad/s
    
    //motor max vel limit
    float linear_max_limit  = myOdomCaculateData.cmd_vel_linear_max;
    float angular_max_limit = myOdomCaculateData.cmd_vel_angular_max;
    
    if(linear_temp>linear_max_limit)
        linear_temp = linear_max_limit;
    if(linear_temp<(-1*linear_max_limit))
        linear_temp = -1*linear_max_limit;
    if(angular_temp>angular_max_limit)
        angular_temp = angular_max_limit;
    if(angular_temp<(-1*angular_max_limit))
        angular_temp = -1*angular_max_limit;
      
          
    double left_motor_rpm_temp = 0;  
    double right_motor_rpm_temp = 0;
   
   
    left_motor_rpm_temp  = ((linear_temp - 0.5*(myOdomCaculateData.wheel_distance)*angular_temp) / (myOdomCaculateData.wheel_diameter/1000.*3.141592))  ;
    right_motor_rpm_temp = ((linear_temp + 0.5*(myOdomCaculateData.wheel_distance)*angular_temp) / (myOdomCaculateData.wheel_diameter/1000.*3.141592))  ; //* 1.872411485) ;
      
    
    //delta_encode_left_temp = (linear_temp-0.5*(myOdomCaculateData.wheel_distance)*angular_temp)*(myOdomCaculateData.encode_sampling_time)/(myOdomCaculateData.speed_ratio);
    //delta_encode_right_temp = (linear_temp+0.5*(myOdomCaculateData.wheel_distance)*angular_temp)*(myOdomCaculateData.encode_sampling_time)/(myOdomCaculateData.speed_ratio);
    
    myMotorRPM.left_motor_rpm  = RPS2RPM(left_motor_rpm_temp) ;   
    myMotorRPM.right_motor_rpm = RPS2RPM(right_motor_rpm_temp) ;
   // printf(" ."); 
   // printf("[send to serial-com] left rpm = %d right rpm =%d\r\n",myMotorRPM.left_motor_rpm,myMotorRPM.right_motor_rpm);
   
    
    
}

void *mywriteframe_thread(void *pt)
{
   int i =0;
   while(1)
   {
     
     if(myComDev.send_update_flag == 1) //get flag
     {
            myComDev.nwrite=write(myComDev.SerialCom,myComDev.writebuff,8);
            //debug
            //printf("send:%x %x %x %x %x %x\r\n",myComDev.writebuff[0],myComDev.writebuff[1],myComDev.writebuff[2],myComDev.writebuff[3],myComDev.writebuff[4],myComDev.writebuff[5],
            //                                    myComDev.writebuff[6],myComDev.writebuff[7],myComDev.writebuff[8],myComDev.writebuff[9],myComDev.writebuff[10]);
            read_serial_data(8);
            //usleep(500); /*&& (myComDev.readbuff[1] == 0x03))*/
            if( (myComDev.motor_command == 2) && (myComDev.readbuff[1] == 0x03))
            {
               
               if(myComDev.readbuff[0] == L_MOTOR_ID)        myBaseSensorData.encode_left = cal_motor_position();
               if(myComDev.readbuff[0] == R_MOTOR_ID)        myBaseSensorData.encode_right = cal_motor_position();
               /*
               for(i=0;i<8;i++) 
               {
                   printf("%X ",myComDev.readbuff[i]);
                   myComDev.motor_command = 0;
               }
               printf("\n");
               */
            }
            myComDev.send_update_flag = 0; //clear flag          
     }        
     
     if(i==10) //if not input cmd_vel during 0.5s, stop motor
     {
           
           
                      
            // printf("tt\n");     
             
             i=0;
      }

      else
      {
            i++;
      }     
      ros::Duration(0.01).sleep(); //delay 10ms
  }
}

void *myRead_Encnoder_thread(void *pt)
{
    int i=0;
  
    //control freq: 100hz (10ms)
    while(1)
    { 
        
        if(i==5)
        {
          read_motor_position(L_MOTOR_ID);
        }
        
        if(i==10)
        {
          read_motor_position(R_MOTOR_ID);
        }
        
        if(i==20)
        {
           
           robot_move(myMotorRPM);
           i=0;
        }
        i++;
        ros::Duration(0.01).sleep(); //delay 10ms
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "zltech_motor_control_node");
  ros::NodeHandle n;

  std::string com_port = "/dev/ttyUSB0";
  std::string cmd_vel_topic = "cmd_vel";
  std::string odom_pub_topic = "odom";
  std::string wheel_left_speed_pub_topic = "wheel_left_speed";
  std::string wheel_right_speed_pub_topic = "wheel_right_speed";
  std::string wheel_left_encoder_pub_topic = "wheel_left_encoder";
  std::string wheel_right_encoder_pub_topic = "wheel_right_encoder";
  
  std::string odom_frame_id = "odom";
  std::string odom_child_frame_id = "base_footprint";
  
  //get param from user launch
  /*serial_com set*/
  ros::param::get("~com_port", com_port);
  /*motor param set*/
  ros::param::get("~speed_ratio", myOdomCaculateData.speed_ratio);
  ros::param::get("~wheel_distance", myOdomCaculateData.wheel_distance);
  ros::param::get("~wheel_diameter", myOdomCaculateData.wheel_diameter );
  ros::param::get("~encode_sampling_time", myOdomCaculateData.encode_sampling_time);
  /*velocity limit*/
  ros::param::get("~cmd_vel_linear_max", myOdomCaculateData.cmd_vel_linear_max);
  ros::param::get("~cmd_vel_angular_max", myOdomCaculateData.cmd_vel_angular_max);
  /*other*/
  ros::param::get("~cmd_vel_topic", cmd_vel_topic);
  ros::param::get("~odom_pub_topic", odom_pub_topic);
  ros::param::get("~wheel_left_speed_pub_topic", wheel_left_speed_pub_topic);
  ros::param::get("~wheel_right_speed_pub_topic", wheel_right_speed_pub_topic);
  ros::param::get("~odom_frame_id", odom_frame_id);
  ros::param::get("~odom_child_frame_id", odom_child_frame_id);

  ros::Subscriber sub = n.subscribe(cmd_vel_topic, 5, callback);
  
  ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>(odom_pub_topic, 20);
  ros::Publisher wheel_left_speed_pub= n.advertise<std_msgs::Float32>(wheel_left_speed_pub_topic, 20);
  ros::Publisher wheel_right_speed_pub= n.advertise<std_msgs::Float32>(wheel_right_speed_pub_topic, 20);
  ros::Publisher wheel_left_encoder_pub= n.advertise<std_msgs::Int32>(wheel_left_encoder_pub_topic, 20);
  ros::Publisher wheel_right_encoder_pub= n.advertise<std_msgs::Int32>(wheel_right_encoder_pub_topic, 20);  
    
    
    
  double scan_duration; 
  unsigned short protocal_crc16;
  double elapsed_time = 0.0;
  clock_t start_time, end_time;
  int motor_position_new[5]            = {0,};
  int motor_position_old[5]            = {0,};
  double motor_speed_rpm[5]            = {0,};
  double motor_speed_rps[5]            = {0,};
  double motor_elapsed_angle_degree[5] = {0.};
  double motor_elapsed_angle_radian[5] = {0.};
  
  Init_COM_L();
  if ( myComDev.SerialCom < 0 )
  {
	   
     ROS_ERROR("Unable to open port");
     
    //printf("Unable to open port \n");
	     return -1;
  }
   else
  {
     ROS_INFO("Serial Port initialized");
     //printf("Serial Port initialized\n");
        
  }
  myComDev.send_update_flag = 0;
  pthread_t thread_id_1;
  int ret1=pthread_create(&thread_id_1,NULL,*mywriteframe_thread,NULL);
  
  pthread_t thread_id_2;
  int ret2=pthread_create(&thread_id_2,NULL,*myRead_Encnoder_thread,NULL);
     

  init_motor_setup();
    
  sleep(1);
    
  ROS_INFO("Set Motor Enable"); 
  
  
  //interrupt_motor_enable(R_MOTOR_ID);
  //interrupt_motor_enable(L_MOTOR_ID);
    
  myMotorRPM.left_motor_rpm = 0 ;   myMotorRPM.right_motor_rpm = 0 ;
  //robot_move(myMotorRPM);
 // printf("Motor Run\n");
  

  start_time = clock();
  ros::Rate loop_rate(10.0); //10.0HZ
    
  while(ros::ok())
  {
    
    //  robot_move(myMotorRPM);
      
      //myBaseSensorData.encode_right =  read_motor_position(R_MOTOR_ID);
     
      end_time = clock();
     
      motor_elapsed_angle_degree[1] = (motor_position_new[1] - motor_position_old[1])/ENCODER_PULSE_REV * 360.;
      motor_elapsed_angle_radian[1] = DEG2RAD(motor_elapsed_angle_degree[1]);
        
      elapsed_time = (double)(end_time - start_time)/CLOCKS_PER_SEC*100;
        
      motor_speed_rps[1] =  motor_elapsed_angle_degree[1]/elapsed_time/360.;
      motor_speed_rpm[1] =  RPS2RPM(motor_speed_rps[1]);
      motor_position_old[1] = motor_position_new[1];
        
	//printf("Encoder : %d  %6.3lf\n",motor_position_new[1], motor_position_new[1]/ENCODER_PULSE_REV * 360.);
       // printf("Elapsed Time : %f[sec]  %f[deg]  %f[rps] %f[rpm] %5.2lf[rpm] \n", elapsed_time, motor_elapsed_angle_degree[1],motor_speed_rps[1],motor_speed_rpm[1],(float)read_motor_speed(1)/10);    
	//start_time = clock();
  
     printf("%d  %d\n", myBaseSensorData.encode_left, myBaseSensorData.encode_right);
	   start_time = end_time;
	
      
     ros::spinOnce();
     loop_rate.sleep();
  }

  
    interrupt_motor_enable(1);
   // interrupt_motor_enable(2);
   // interrupt_motor_enable(3);
   // interrupt_motor_enable(4);
    
    
    
    close(myComDev.SerialCom);
    return 0;
}

