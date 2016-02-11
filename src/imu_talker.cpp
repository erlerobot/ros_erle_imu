#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include "MPU9250.h"
#include "AHRS.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

// Objects

MPU9250 imu;    // MPU9250
AHRS    ahrs;   // Mahony AHRS

// Sensor data

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

// Orientation data

float roll, pitch, yaw;

// Timing data

float offset[3];
struct timeval tv;
float dt;
unsigned long previoustime, currenttime;

//============================= Initial setup =================================

void imuSetup()
{
    //----------------------- MPU initialization ------------------------------

    imu.initialize();

    //-------------------------------------------------------------------------

	printf("Beginning Gyro calibration...\n");
	for(int i = 0; i<100; i++)
	{
		imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		offset[0] += (-gx*0.0175);
		offset[1] += (-gy*0.0175);
		offset[2] += (-gz*0.0175);
		usleep(10000);
	}
	offset[0]/=100.0;
	offset[1]/=100.0;
	offset[2]/=100.0;

	printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
	ahrs.setGyroOffset(offset[0], offset[1], offset[2]);
}

//=============================================================================

int main(int argc, char *argv[])
{
    //-------------------- IMU setup and main loop ----------------------------
    imuSetup();

    ros::init(argc, argv, "imu_talker");
    
    ros::NodeHandle n;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu9250", 1000);
    ros::Publisher temperature_pub = n.advertise<sensor_msgs::Temperature>("temperature9250", 1000);

    ros::Rate loop_rate(50);

    int count = 0;
    while (ros::ok()){

        //----------------------- Calculate delta time ----------------------------

        gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        if(dt < 1/1300.0) 
            usleep((1/1300.0-dt)*1000000);
        gettimeofday(&tv,NULL);
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;

        //-------- Read raw measurements from the MPU and update AHRS --------------
        // Accel + gyro.
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ahrs.updateIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);

        //------------------------ Read Euler angles ------------------------------

        ahrs.getEuler(&roll, &pitch, &yaw);

        sensor_msgs::Imu msg;
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;

        msg.linear_acceleration_covariance[0] = 0.04;
        msg.linear_acceleration_covariance[1] = 0;
        msg.linear_acceleration_covariance[2] = 0;

        msg.linear_acceleration_covariance[3] = 0;
        msg.linear_acceleration_covariance[4] = 0.04;
        msg.linear_acceleration_covariance[5] = 0;

        msg.linear_acceleration_covariance[6] = 0;
        msg.linear_acceleration_covariance[7] = 0;
        msg.linear_acceleration_covariance[8] = 0.04;


        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;


        msg.angular_velocity_covariance[0] = 0.02;
        msg.angular_velocity_covariance[1] = 0;
        msg.angular_velocity_covariance[2] = 0;

        msg.angular_velocity_covariance[3] = 0;
        msg.angular_velocity_covariance[4] = 0.02;
        msg.angular_velocity_covariance[5] = 0;

        msg.angular_velocity_covariance[6] = 0;
        msg.angular_velocity_covariance[7] = 0;
        msg.angular_velocity_covariance[8] = 0.02;

        msg.orientation.w = ahrs.getW();
        msg.orientation.x = ahrs.getX();
        msg.orientation.y = ahrs.getY();
        msg.orientation.z = ahrs.getZ();

        msg.orientation_covariance[0] = 0.0025;
        msg.orientation_covariance[1] = 0;
        msg.orientation_covariance[2] = 0;

        msg.orientation_covariance[3] = 0;
        msg.orientation_covariance[4] = 0.0025;
        msg.orientation_covariance[5] = 0;

        msg.orientation_covariance[6] = 0;
        msg.orientation_covariance[7] = 0;
        msg.orientation_covariance[8] = 0.0025;

        imu_pub.publish(msg);

        sensor_msgs::Temperature temperature_msg;
        temperature_msg.temperature = imu.temperature;
        temperature_msg.variance = 0;
        temperature_pub.publish(temperature_msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;

}
