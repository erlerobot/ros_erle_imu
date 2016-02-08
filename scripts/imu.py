#!/usr/bin/env python

import spidev
import time
import rospy
import math
import numpy as np
from time import time
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
		
		msg.header.stamp = rospy.get_rostime()

		m9a, m9g, m9m = imu.getMotion9()


		#----------update IMU
		ax = m9a[0]
		ay = m9a[1]
		az = m9a[2]
		gx = m9g[0]
		gy = m9g[1]
		gz = m9g[2]
		q0 = 0.0 #W
		q1 = 0.0 #X
		q2 = 0.0 #Y
		q3 = 0.0 #Z

		#----------Calculate delta time
		t = time()
		currenttime = 0
		previoustime = currenttime
		currenttime = 1000000 * t + t / 1000000
		dt = (currenttime - previoustime) / 1000000.0
		if (dt < (1/1300.0)) : 
			time.sleep((1/1300.0 - dt) * 1000000)
		t = time()
		currenttime = 1000000 * t + t / 1000000
		dt = (currenttime - previoustime) / 1000000.0
		print "Delta time: d = %f" % dt

		#Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)) :
			
			#Normalise accelerometer measurement
			recipNorm = (ax * ax + ay * ay + az * az)**-.5
			ax *= recipNorm
			ay *= recipNorm
			az *= recipNorm

			#Estimated direction of gravity and vector perpendicular to magnetic flux
			halfvx = q1 * q3 - q0 * q2
			halfvy = q0 * q1 + q2 * q3
			halfvz = q0 * q0 - 0.5 + q3 * q3

			#Error is sum of cross product between estimated and measured direction of gravity
			halfex = (ay * halfvz - az * halfvy)
			halfey = (az * halfvx - ax * halfvz)
			halfez = (ax * halfvy - ay * halfvx)

			#Compute and apply integral feedback (if enabled)
			integralFBx += twoKi * halfex * dt;
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx
			gy += integralFBy
			gz += integralFBz

			#Apply proportional feedback
			gx += twoKp * halfex;
			gy += twoKp * halfey;
			gz += twoKp * halfez;

		#Integrate rate of change of quaternion
		gx *= (0.5 * dt)
		gy *= (0.5 * dt)
		gz *= (0.5 * dt)
		qa = q0
		qb = q1
		qc = q2
		q0 += (-qb * gx - qc * gy - q3 * gz)
		q1 += (qa * gx + qc * gz - q3 * gy)
		q2 += (qa * gy - qb * gz + q3 * gx)
		q3 += (qa * gz + qb * gy - qc * gx)

		#Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
		q0 *= recipNorm
		q1 *= recipNorm
		q2 *= recipNorm
		q3 *= recipNorm

		#Fill message
		msg.orientation.x = q1
		msg.orientation.y = q2
		msg.orientation.z = q3
		msg.orientation.w = q0
		msg.orientation_covariance[0] = q1 * q1
		msg.orientation_covariance[0] = q2 * q2
		msg.orientation_covariance[0] = q3 * q3		

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
