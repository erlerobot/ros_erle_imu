#!/usr/bin/env python

import spidev
import time
import rospy
import math
import numpy as np

from MPU9250 import MPU9250
from sensor_msgs.msg import Imu

def talker():
	pub = rospy.Publisher('imu', Imu, queue_size=10)
	rospy.init_node('ros_erle_imu', anonymous=True)
	rate = rospy.Rate(10)

	imu = MPU9250()
	imu.initialize()
	
	msg = Imu()

	while not rospy.is_shutdown():

		imu.read_acc()
		
		m9a, m9g, m9m = imu.getMotion9()

		msg.header.stamp = rospy.get_rostime()

		#ORIENTATION calculation
		'''You can just integrate your angular velocity to get angular 
		position (as Euler angles), convert the Euler angles to Quaternion, 
		then multiply the Quaternion to accumulate the orientation.

		Suppose your input is given by a 3D vector of angular velocity: 
		omega = (alpha, beta, gamma), given by degrees per second. 
		To get the Euler angles, E, in degrees, multiply omega by the change 
		in time, which we can call dt '''

		dt = 0.1 #10Hz in seconds
		
		AngVelX = m9g[0]*57.29577951308*dt
		AngVelY = m9g[1]*57.29577951308*dt
		AngVelZ = m9g[2]*57.29577951308*dt

		w_or = math.cos(AngVelX/2) * math.cos(AngVelY/2) * math.cos(AngVelZ/2) + math.sin(AngVelX/2) * math.sin(AngVelY/2) * math.sin(AngVelZ/2)
		x_or = math.sin(AngVelX/2) * math.cos(AngVelY/2) * math.cos(AngVelZ/2) - math.cos(AngVelX/2) * math.sin(AngVelY/2) * math.sin(AngVelZ/2)
		y_or = math.cos(AngVelX/2) * math.sin(AngVelY/2) * math.cos(AngVelZ/2) + math.sin(AngVelX/2) * math.cos(AngVelY/2) * math.sin(AngVelZ/2)
		z_or = math.cos(AngVelX/2) * math.cos(AngVelY/2) * math.sin(AngVelZ/2) - math.sin(AngVelX/2) * math.sin(AngVelY/2) * math.cos(AngVelZ/2)

		msg.orientation.x = x_or
		msg.orientation.y = y_or
		msg.orientation.z = z_or
		msg.orientation.w = w_or
		msg.orientation_covariance[0] = x_or * x_or
		msg.orientation_covariance[0] = y_or * y_or
		msg.orientation_covariance[0] = z_or * z_or		

		msg.angular_velocity.x = m9g[0]
		msg.angular_velocity.y = m9g[1]
		msg.angular_velocity.z = m9g[2]
		msg.angular_velocity_covariance[0] = m9g[0] * m9g[0]
                msg.angular_velocity_covariance[4] = m9g[1] * m9g[1]
                msg.angular_velocity_covariance[8] = m9g[2] * m9g[2]
		
		
		msg.linear_acceleration.x = m9a[0]
		msg.linear_acceleration.y = m9a[1]
		msg.linear_acceleration.z = m9a[2]
		msg.linear_acceleration_covariance[0] = m9a[0] * m9a[0]
		msg.linear_acceleration_covariance[4] = m9a[1] * m9a[1]
		msg.linear_acceleration_covariance[8] = m9a[2] * m9a[2]
		
		pub.publish(msg)

		rate.sleep()

if __name__ == '__main__':
	try:
        	talker()
  	except rospy.ROSInterruptException:
        	pass
