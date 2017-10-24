/*
 * 每次都需要把chmod 666 ttyUSB0
 * 每1s只能获取到约97Hz的数据
 * 将MPU6050封装成发布器节点sensor_imu_node
*/


#define PORT "/dev/ttyUSB0"
#define NCC 32

#include <stdio.h>          //标准输入输出定义
#include <stdlib.h>         //标准函数库定义
#include <unistd.h>        //Unix 标准函数定义
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>          //文件控制定义
#include <termios.h>    //PPSIX 终端控制定义
#include <errno.h>        //错误号定义
#include <string.h>
#include <iostream>
#include <sys/time.h>
#include <math.h>

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sstream>

using namespace std;

float accl[3], angv[3], ang[3], Temp;

sensor_msgs::Imu imu;

/**
*@brief                     initial the serial
*@param  fd             which serial
*@param  speed      speed of serial
*@return  void
*/
int speed_arr[] = { B300, B600, B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200};
int name_arr[] = { 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};

void Set_serial(int fd, int speed)
{
        int i;
        int status;
        struct termios opt;
        tcgetattr(fd, &opt);
        for ( i=0; i < sizeof(speed_arr) / sizeof(int); i++)
        {
                tcflush(fd, TCIOFLUSH);
                cfsetispeed(&opt, speed_arr[i]);    //speed_arr[i]Bps
                cfsetospeed(&opt, speed_arr[i]);
                status = tcsetattr(fd, TCSANOW, &opt);
                if (status != 0)
                {
                        perror("tcsetattr fd !");
                        return;
                }
                tcflush(fd, TCIOFLUSH);
        }
}

void DecodeIMUData(unsigned char *reTemp)
{
        if(reTemp[0]==0x55)      //检查帧头
        {
            switch(reTemp[1])
            {
                   case 0x51:
                       accl[0] = (short(reTemp [3]<<8| reTemp [2]))/32768.0*16;
                       accl[1] = (short(reTemp [5]<<8| reTemp [4]))/32768.0*16;
                       accl[2] = (short(reTemp [7]<<8| reTemp [6]))/32768.0*16;
                       Temp = (short(reTemp [9]<<8| reTemp [8]))/340.0+36.25;
                       ROS_INFO("Acceleration x %f y %f z %f", accl[0], accl[1], accl[2]);
                       imu.header.stamp = ros::Time::now();
                       imu.linear_acceleration.x = accl[0];
                       imu.linear_acceleration.y = accl[1];
                       imu.linear_acceleration.z = accl[2];
                       imu.linear_acceleration_covariance[0] = accl[0];
                       imu.linear_acceleration_covariance[4] = accl[1];
                       imu.linear_acceleration_covariance[8] = accl[2];
                       break;
                   case 0x52:
                       angv[0] = (short(reTemp [3]<<8| reTemp [2]))/32768.0*2000;
                       angv[1] = (short(reTemp [5]<<8| reTemp [4]))/32768.0*2000;
                       angv[2] = (short(reTemp [7]<<8| reTemp [6]))/32768.0*2000;
                       Temp = (short(reTemp [9]<<8| reTemp [8]))/340.0+36.25;
                       ROS_INFO("Angular velocity x %f y %f z %f", angv[0], angv[1], angv[2]);
                       imu.angular_velocity.x = angv[0];
                       imu.angular_velocity.y = angv[1];
                       imu.angular_velocity.z = angv[2];
                       imu.angular_velocity_covariance[0] = angv[0];
                       imu.angular_velocity_covariance[4] = angv[1];
                       imu.angular_velocity_covariance[8] = angv[2];
                       break;
                   case 0x53:
                       ang[0] = (short(reTemp [3]<<8| reTemp [2]))/32768.0*180;
                       ang[1] = (short(reTemp [5]<<8| reTemp [4]))/32768.0*180;
                       ang[2] = (short(reTemp [7]<<8| reTemp [6]))/32768.0*180;
                       Temp = (short(reTemp [9]<<8| reTemp [8]))/340.0+36.25;
                       ROS_INFO("Angle x %f y %f z %f", ang[0], ang[1], ang[2]);
                       float fCosHRoll = cos(ang[0] * .5f);
                       float fSinHRoll = sin(ang[0] * .5f);
                       float fCosHPitch = cos(ang[1] * .5f);
                       float fSinHPitch = sin(ang[1] * .5f);
                       float fCosHYaw = cos(ang[2] * .5f);
                       float fSinHYaw = sin(ang[2] * .5f);
                       imu.orientation.w = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
                       imu.orientation.x = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
                       imu.orientation.y = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
                       imu.orientation.z = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
                       imu.orientation_covariance[0] = ang[0];
                       imu.orientation_covariance[4] = ang[1];
                       imu.orientation_covariance[8] = ang[2];
                       break;
            }
        }
}

int main(int argc, char **argv)
{
	// 初始化,重映射命令行的参数(可使用),指定节点名称
	ros::init(argc, argv, "sensor_imu_node");
	ROS_INFO("sensor_imu begin ..."); // 替代prinf/cout
	ros::NodeHandle n; // 建立为节点初始化或销毁的句柄
        // 向imu_data话题发布sensor_imu::mpu6050类型的消息
        
        imu.header.frame_id = "map";
	ros::Publisher imu_data_pub = n.advertise<sensor_msgs::Imu>("/imu0", 10);
	
        int fd;
        fd = open(PORT, O_RDWR);
        if (fd ==-1)
        {
                perror("Can not Open Serial Port !");
                return 0;
        }
        Set_serial(fd, 115200); // 串口波特率115200

        char reBuf[50];
        unsigned char reTemp[11];
        int head = 0, end = 0; //head/end of the data
        while (ros::ok())
        {
                head = 0;
		end = read(fd, reBuf, 1); // 不断读数据直到读到帧头0x55
                if (end == 0) continue;
		if (reBuf[0]==0x55) 
		{		
			end = read(fd, &reBuf[1], 32); // 读取完整的数据帧, 长度=33-1(帧头)
			while ((end - head + 1) >= 11)
			{
				for (int i = 0; i <= 10; i++) reTemp[i] = reBuf[i+head];
				if (!((reTemp[0] == 0x55) & ((reTemp[1] == 0x51) | (reTemp[1] == 0x52) | (reTemp[1] == 0x53))))
				{
					head ++;
					continue;
				}
			        DecodeIMUData(reTemp);
				head += 11;
			}
		}
		imu_data_pub.publish(imu);
                usleep(10000); // 每隔10ms读取一次串口
        }
        close(fd);
}






















