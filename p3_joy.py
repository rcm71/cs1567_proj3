#!/usr/bin/env python

#changes the command array. also applies speed limit to controller input
#speed limit handled here. smoothing handled in merge_smoother.py


import os
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
command_pub = rospy.Publisher("robot_command", Int32MultiArray, queue_size=1)
twist_pub = rospy.Publisher("robot_twist", Twist, queue_size=1)
command = Int32MultiArray()
command.data = [1,1,1,1,1,0,1,0,0,0]
twist = Twist()


def callback(data):
        global command_pub, twist_pub
	if(data.buttons[0] > 0):#bumper - A
		command.data[0] = command.data[0] ^ 1
	if(data.buttons[1] > 0):#cliff - B
		command.data[1] = command.data[1] ^ 1
	if(data.buttons[2] > 0):#drop - x
                command.data[2] = command.data[2] ^ 1
	if(data.buttons[3] > 0):#backwards only - Y
                command.data[3] =command.data[3] ^ 1
	if(data.buttons[6] > 0):#LED - Back (select)
                command.data[4] =command.data[4] ^ 1
	if(data.buttons[5] > 0):#Emergency Brake - RB
                command.data[5] =command.data[5] ^ 1
	if(data.buttons[7] > 0):#Mode switch - Start
                command.data[6] = (command.data[6] + 1) % 3# 0,1,2 cycle
	if(data.axes[7] > 0):#Linear Speed Limit
                if (command.data[7]<8):#max speed 8
			command.data[7]+=1
	if(data.buttons[4] > 0):#Toggle odom recording
	     	command.data[9] = command.data[9] ^ 1
	elif(data.axes[7] < 0):#more linear limit
		if(command.data[7] > 0):#min speed 0
			command.data[7]-=1
	if(data.axes[6] > 0):# Angular speed limit
		if(command.data[8] < 10):
			command.data[8]+=1
	elif(data.axes[6] < 0):
		if(command.data[8] > 0):
			command.data[8]-=1

	rt = (data.axes[5] - 1) * -.5# changes our scale to 0->1
	lt = (data.axes[2] - 1) * -.5
	if(lt > 0 and rt > 0):
		twist.linear.x = 0
	elif(rt > 0):
		twist.linear.x = command.data[7] * rt / 10.0# forwards
	elif(lt > 0):
		twist.linear.x = (command.data[7] * lt / 10.0) * -1# backwards	
	else:
		twist.linear.x = 0
	
	twist.angular.z = (command.data[8] / 10.0) *  data.axes[0]


        twist_pub.publish(twist)
	command_pub.publish(command)
	os.system('clear')
	
	if (command.data[0] == 1):
		print('Bumper: on')
	else:
		print('Bumper : off')
	if (command.data[1] == 1):
		print('Cliff: on')
	else:
		print('Cliff: off')
	if (command.data[2] == 1):
		print('Wheel Drop : on')
	else:
		print('Wheel Drop: off')
	if (command.data[3] == 1):
		print('Backward Only: on')
	else:
		print('Backward Only: off')
	if (command.data[4]  == 1):
		print('LED and Sound: on')
	else: 
		print('LED and Sound: off')
	if (command.data[5] == 1):
		print('Emergency Brake: Engaged')
	else: 
		print('Emergency Brake: Disengaged')
	if(command.data[6] == 0):
		print('Smoother: Off')
	elif(command.data[6] == 1):
		print('Smoother: Eco')
	else:
		print('Smoother: Sport')
	print('Linear  Limit:' , command.data[7]/10.0)
	print('Angular Limit:' , command.data[8]/10.0)
 
def main():
        rospy.init_node("joystick", anonymous=True)
        rospy.Subscriber("joy", Joy, callback)
        rospy.spin()

if __name__ == '__main__':
        try:
		main()
	except rospy.ROSInterruptException:
		pass
