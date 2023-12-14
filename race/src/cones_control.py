#!/usr/bin/env python3
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped


global kp 
kp = 1.0 
global kd
kd = 1.0
global ki
ki = 0.0
prev_error = 0.0

vel_input = 1
command_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 1)

def control(data: pid_input):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle
	angle=0.0

	angle = kp*data.pid_error + kd*((prev_error - data.pid_error ))
	command = AckermannDrive()
	an=command.steering_angle-angle
	command.steering_angle = an
	command.speed = vel_input
	command.steering_angle_velocity = 10.0
	command.acceleration = 0.0
    

	new= AckermannDriveStamped()
	new.drive=command
	command_pub.publish(new)
	prev_error = data.pid_error
	

if __name__ == '__main__':

	rospy.init_node('pid_controller', anonymous=True)
	print("PID Control Node is Listening to error")
	rospy.Subscriber("/err", pid_input, control)
	rospy.spin()