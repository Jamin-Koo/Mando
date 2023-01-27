/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
/*******************************************************************************
 *  INCLUDE #define POLYNORMIAL 0xA001FILES
 *******************************************************************************
 */
 
#include "ros/ros.h" 

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>                   // B115200, CS8 등 상수 정의
#include <fcntl.h>                     // O_RDWR , O_NOCTTY 등의 상수 정의
#include <time.h>
#include <math.h>

//multi thread
#include <pthread.h>

#include "SimpleKalmanFilter.h"

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 
#define SONAR_MAX_DISTANCE 3200

 
typedef unsigned char BYTE;

union
{
    float data ;
    char  bytedata[4];
    
} m_robot_speed , m_current_robot_speed;

union
{
    short data ;
    char  bytedata[2];
    
} m_robot_angle;

union 
{ 	
	unsigned short a; 
	unsigned char b[2]; 
} crc_16_val;


BYTE sonar_read_tx_data[8] = {0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85, 0xF6};

int sonar_range[5]={0,0,0,0,0}; //센서는 4개이지만 ID 가 1번 부터 시작이여서 배열은 5번까지

int kalman_sonar_range[5] ; //={0,0,0,0,0};

/*
 //////////////////// sonar 배치 ////////////////////

         1                                 3


 
         2                                 4
*/

SimpleKalmanFilter KF_1(30, 30, 0.01);
SimpleKalmanFilter KF_2(30, 30, 0.01);
SimpleKalmanFilter KF_3(30, 30, 0.01);
SimpleKalmanFilter KF_4(30, 30, 0.01);

#define BAUDRATE        B9600
#define SERIAL_DEVICE   "/dev/Sonar"  
//#define SERIAL_DEVICE   "/dev/ttyS0"  

static int uart_fd;
unsigned char protocal_test[12] ={0,};
unsigned char read_buf [20];

int CRC16_MODBUS (const uint8_t *nData, uint16_t wLength)
{

  static const uint16_t wCRCTable[] = { 0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780, 
	                                    0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 
	                                    0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801, 
	                                    0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40, 
	                                    0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 
	                                    0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0, 
	                                    0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 
	                                    0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41, 
	                                    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981, 
	                                    0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 
	                                    0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200, 
	                                    0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141, 
	                                    0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 
	                                    0XA441, 0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0, 
	                                    0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 
	                                    0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40, 
	                                    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381, 
	                                    0X7340, 0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 
	                                    0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01, 
	                                    0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81, 0X5B40, 
	                                    0X9901, 0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 
	                                    0X4A40, 0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1, 
	                                    0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 
	                                    0X81C1, 0X8081, 0X4040 };
	                                    
	uint8_t nTemp; 
    uint16_t wCRCWord = 0xFFFF; 

    while (wLength--)
    { 
      nTemp = *nData++ ^ wCRCWord; 
      wCRCWord >>= 8; 
      wCRCWord ^= wCRCTable[nTemp]; 
    } 

    return wCRCWord;                                 
}




void write_serial(unsigned char *buf, int len)
{
	write(uart_fd, &buf[0], len);
} 


int init_serial_port(void)
{
  int serial_port = open(SERIAL_DEVICE, O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 100;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, BAUDRATE);
  cfsetospeed(&tty, BAUDRATE);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return -1;
  }
    
  else
  {
      return serial_port;
  } 
}


void *readserial_thread(void *pt)
{
    
    int num_bytes = -1;
    unsigned char insert_buf; 
   

    while(1)
    { 
		printf("thread starts\n");
     	while( (num_bytes = read(uart_fd, &insert_buf, 1)   ) > 0 )	
        {
	       //printf("thread\n");
           //printf("No read %d\n",num_bytes);
	   
	       for(int i=0;i<6;i++)
           {
                read_buf[i]=read_buf[i+1];
           }
           read_buf[6]=insert_buf;
	     /*  
	       for(int i=0;i<=6;i++)
	       {
			   printf("0x%02x ", read_buf[i]);	
           }
            
           printf("\n\n\n\n\n");   
          */ 
           // printf("\n\n\n\n\n");   
           if( (read_buf[1]==0x03) && (read_buf[2]==0x02) )
		   {
				   
				  crc_16_val.a = CRC16_MODBUS(read_buf,5); 
				  if( ( crc_16_val.b[0]==read_buf[5]) && ( crc_16_val.b[1]==read_buf[6]) )
				  {
					  sonar_range[read_buf[0]] = read_buf[3]*256 + read_buf[4];  //sonar id는 1번 부터
					  /*
					 
					  */
					  //printf("CRC16 is O.K. %d %d\n", read_buf[0], sonar_range[read_buf[0]]);
				  }   
		   }       
         
	   }
	}	
	
} 


int ComSetup(void)
{
    int serial_port= open(SERIAL_DEVICE, O_RDWR); //set name of serial-com
    if(serial_port == -1)
    {
        printf("Can't open serial port!\n");
    }
    
     struct termios options;

  // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &options) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
    }

    tcflush(serial_port, TCIFLUSH);
    cfsetispeed(&options, B9600);   //set recieve bps of serial-com
    cfsetospeed(&options, B9600);   //set send bps of serial-com
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~(ONLCR | OCRNL);

    if(tcsetattr(serial_port, TCSANOW, &options) != 0)
    {
        printf("Can't set serial port options!\n");
    }
}

void read_sonar_data(int sonar_id)
{
	unsigned short protocal_crc16;
	int i =0;
	protocal_test[0] = sonar_id;
	for(i=1;i <= 5; i++)   protocal_test[i] = sonar_read_tx_data[i];
    
    crc_16_val.a = CRC16_MODBUS(protocal_test,6);
    
    
    protocal_test[6] = crc_16_val.b[0];
    protocal_test[7] = crc_16_val.b[1];    
   
    printf("Send Protocal : ");
    for(i=0;i <= 7; i++)
    {
	  printf("0x%02x " , protocal_test[i]);	
    }
    //printf("0x%02x " , protocal_test[0]);
    printf("\n\n");
    write_serial(protocal_test,8);  
}

int main(int argc, char **argv)
{
  int sonar_id = 1;	
  int sonar_obstacle_detect = 0;
  int i=0;
  ros::init(argc, argv, "Clean_Robot_Sonar_node");
  ros::NodeHandle n;
  
  std::string com_port = "/dev/Sonar"; 
  
  //ros::Subscriber sub = n.subscribe(cmd_vel_topic, 20, callback);
  //ros::Subscriber sub_rpy_angle = n.subscribe("/rpy_degree", 20, callback2);
  
  
  uart_fd = init_serial_port(); 
  
  pthread_t id_1;
  int ret1=pthread_create(&id_1,NULL,*readserial_thread,NULL);
  
  ros::Publisher pub_sonar1 = n.advertise<sensor_msgs::Range>("/RangeSonar1",10);
  ros::Publisher pub_sonar2 = n.advertise<sensor_msgs::Range>("/RangeSonar2",10);
  ros::Publisher pub_sonar3 = n.advertise<sensor_msgs::Range>("/RangeSonar3",10);
  ros::Publisher pub_sonar4 = n.advertise<sensor_msgs::Range>("/RangeSonar4",10);
  ros::Publisher pub_sonar_obstacle_stop = n.advertise<std_msgs::Bool>("/obstacle_flag",10);
  
  
  ros::Rate loop_rate(20); //5.0HZ
  
  std_msgs::Bool m_obstacle_flag;   // 1: stop
  m_obstacle_flag.data = 0;
  
  sensor_msgs::Range sonar_range1;
  sonar_range1.header.frame_id =  "sonar_range1";
  sonar_range1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_range1.field_of_view = (float)(30.0/180.0) * 3.14159;
  sonar_range1.min_range = 0.0;
  sonar_range1.max_range = SONAR_MAX_DISTANCE;
  
  sensor_msgs::Range sonar_range2;
  sonar_range2.header.frame_id =  "sonar_range2";
  sonar_range2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_range2.field_of_view = (float)(30.0/180.0) * 3.14159;
  sonar_range2.min_range = 0.0;
  sonar_range2.max_range = SONAR_MAX_DISTANCE;
  
  sensor_msgs::Range sonar_range3;
  sonar_range3.header.frame_id =  "sonar_range3";
  sonar_range3.field_of_view = (float)(30.0/180.0) * 3.14159;
  sonar_range3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_range3.min_range = 0.0;
  sonar_range3.max_range = SONAR_MAX_DISTANCE;
  
  sensor_msgs::Range sonar_range4;
  sonar_range4.header.frame_id =  "sonar_range4";
  sonar_range4.field_of_view = (float)(30.0/180.0) * 3.14159;
  sonar_range4.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_range4.min_range = 0.0;
  sonar_range4.max_range = SONAR_MAX_DISTANCE;
  
  
  while(ros::ok())
  {
     //printf("S:%4.2lf | A:%3d\n", m_robot_speed.data, m_robot_angle.data);
     //read_sonar_data(1);
     read_sonar_data(sonar_id);    
     
     switch(sonar_id)
	 {
	    case 1:   
		         //kalman_sonar_range[1]= KF_1.updateEstimate(sonar_range[1]);
		         sonar_range1.header.stamp = ros::Time::now();
		         sonar_range1.range = sonar_range[sonar_id];
		         pub_sonar1.publish(sonar_range1);
		         break;
		case 2:   
		         //kalman_sonar_range[2]= KF_2.updateEstimate(sonar_range[2]);
		         sonar_range2.header.stamp = ros::Time::now();
		         sonar_range2.range = sonar_range[sonar_id];
		         pub_sonar2.publish(sonar_range2);
		         break;        
		case 3:   
		         //kalman_sonar_range[3]= KF_3.updateEstimate(sonar_range[3]);
		         sonar_range3.header.stamp = ros::Time::now();
		         sonar_range3.range = sonar_range[sonar_id];
		         pub_sonar3.publish(sonar_range3);
		         break;
		case 4:   
		         //kalman_sonar_range[4]= KF_4.updateEstimate(sonar_range[4]);
		         sonar_range4.header.stamp = ros::Time::now();
		         sonar_range4.range = sonar_range[sonar_id];
		         pub_sonar4.publish(sonar_range4);
		         break; 
	 }
     //printf("CRC16 is O.K. ID: %d Sonar : %4d  kalman : %4d\n", sonar_id, sonar_range[sonar_id],kalman_sonar_range[sonar_id]);
     
     
     //printf("CRC16 is O.K. ID: %d Sonar : %4d \n",sonar_id,sonar_range[sonar_id]);
     
     
     /*여기에 초음파 센서에 장애물이 감지되면 차량을 서게 하는  topic을 publish  해야 함 */
     
     sonar_id ++;
     if(sonar_id==5) sonar_id=1; //초음파 센서가 4개이므로 나중에 5로 바꾸어 야 함
     
     for(i=1,sonar_obstacle_detect=0; i<=4;i++)
     {
        if( (sonar_id,sonar_range[i] < 400) && (sonar_id,sonar_range[i] >0))
        {
			m_obstacle_flag.data = 1;	
			pub_sonar_obstacle_stop.publish(m_obstacle_flag);	
			sonar_obstacle_detect++;	
			break;
		}
	 }
	 
	 if( (sonar_obstacle_detect==0) /*&& (m_obstacle_stop_flag.data == 1)*/)
	 {
		 m_obstacle_flag.data  = 0;
		 pub_sonar_obstacle_stop.publish(m_obstacle_flag);	
		 
	 }
     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;
}
