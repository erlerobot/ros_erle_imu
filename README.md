# ros-erle-imu
A ROS package to read data from the IMU using the Imu.msg message. 
Quaternion calculation is unfinished so don't take it as a reference.

The package consists of a talker in python and a listener in c++.
The talker reads data while the listener prints it. The listener also converts Quaternions to Euler angles.

The IMU message is taken form the 'sensor_msgs' package, so you should import the package to your workspace.
You can find that <a href="https://github.com/ros/common_msgs">here</a>.

Quick setup:
-----
```bash
# Within a catkin directory
cd src
git clone https://github.com/erlerobot/ros_erle_imu      
cd ..
catkin_make_isolated --pkg ros_erle_imu
sudo -s # you'll need privileges since we're accessing spi
source devel/setup.bash
rosrun ros_erle_imu imu_talker
```
If you have problems to execute the talker (imu.py), try executing it as root.

Test it by typing the following in a different terminal:

OPTION 1
```bash
rostopic echo imu  
```
OPTION 2
```bash
rosrun ros_erle_imu imu_listener
```

Erle-Brain 2 Supported

Testing
-----
```bash
rosrun ros_erle_imu visualization.py
```

![](http://docs.erlerobotics.com/erle_robots/erle_copter/examples/visualizing_imu/IMU_brain2.png)

Links
-----

  - [Erle Robotics](www.erlerobotics.com)
  - [Erle-Brain](https://erlerobotics.com/blog/product/erle-brain-v2/)


