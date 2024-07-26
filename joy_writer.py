#!/usr/bin/env python

import rospy
import math 

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led, Sound, BumperEvent,CliffEvent,WheelDropEvent
from std_msgs.msg import Empty,Int32MultiArray
from nav_msgs.msg import Odometry
import threading


class movement:#used for both linear and angular
	def __init__(self, curr, limit, goal):
		self.curr = curr
		self.limit = limit
		self.goal = goal

angular = movement(0,0,0)
linear = movement(0,0,0)
commands=[1,1,1,1,1,0,1,8,10,0]
#^ bumper,cliff,wheeldrop,backwardsonly,led,brake,mode,lin.speed,ang.speed
adjusted_twist = Twist()
velocity_pub = rospy.Publisher("/mobile_base/commands/velocity",							   Twist,queue_size = 10)#10???? 
led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size = 10)
led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size = 10)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size = 10)
resetOdomPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size = 10)

odom = Odometry()
active_flash = False
sensor_triggered = False
A_SMOOTHER = 0.04
L_SMOOTHER = 0.008

##############################################

def odom_callback(data):
    global odom
    odom = data

##############################################

def command_callback(command):
    global commands, linear, angular
    linear.limit = command.data[7]/10
    angular.limit = command.data[8]/10
    commands = command.data
	
##############################################

def twist_callback(twist):
	global linear, angular
	linear.goal = twist.linear.x
	angular.goal = twist.angular.z
    
##############################################

def cleanUp():
	global adjusted_twist, pub1, pub2
	adjusted_twist.angular.z = 0
	adjusted_twist.linear.x = 0
	velocity_pub.publish(adjusted_twist)
	led = Led()
	led.value = 0
        led1.publish(led)
	led2.publish(led)
	rospy.sleep(1)


##############################################
#sensor callback functions

def wheelDropCallback(data):
    '''
    data.wheel(0 for left, 1 for right)
    data.state(0 for raised, 1 for dropped)
    '''
    global sensor_triggered

    if(commands[2] == 1): #if command[wheeldrop] is on
        if(data.state != 0): #wheel raised
            sensor_triggered = True
            
            if(data.wheel == 0):
                rospy.loginfo("left wheel dropped")
            else:
                rospy.loginfo("right wheel dropped")
   	else:
	    sensor_triggered = False
    else:
        pass

##############################################  

def CliffCallback(data):
    
    global sensor_triggered

    if(commands[1] == 1): #if command[cliff] is on
        if(data.state != 0): #cliff detected
            sensor_triggered = True
            if(data.sensor == 0):
                rospy.loginfo("left cliff detected")
            elif(data.sensor == 1):
                rospy.loginfo("center cliff detected")
            else:
                rospy.loginfo("right cliff detected")
   	else:
	    sensor_triggered = False
    else:
        pass

##############################################  
def BumperCallback(data):
    global sensor_triggered

    if(commands[0] == 1): #if command[cliff] is on
        if(data.state != 0): #bumper pressed
            sensor_triggered = True
            
            if(data.bumper == 0):
                rospy.loginfo("left bumper pressed")
            elif(data.bumper == 1):
                rospy.loginfo("center bumper pressed")
            else:
                rospy.loginfo("right bumper pressed")
	else:
	    sensor_triggered = False
    else:
        pass

############################################## 

def play_sound():
        global sound_pub, commands
        while sound_pub.get_num_connections() == 0:
                pass

        sound = Sound()
        sound.value = Sound.ON
	if (commands[4] == 1):
        	for _ in range(3):
                	sound_pub.publish(sound)
                	rospy.sleep(1)

##############################################
def flash_leds():
        global led1, led2, active_flash, commands
        while led1.get_num_connections() == 0 or led2.get_num_connections() == 0:
                pass

        led = Led()
	active_flash = True
	if(commands[4] == 1):
        	for value in range(3, -1, -1):
                	led.value = value
                	led1.publish(led)
                	led2.publish(led)
                	rospy.sleep(1)
	active_flash = False


##############################################

##############################################

def main():
    global active_flash, commands, resetOdomPub, angular, linear,velocity_pub
    linear.curr = 0.0#hold ya horses
    rospy.init_node("smoother", anonymous=True)
    rospy.Subscriber("robot_command", Int32MultiArray, command_callback)
    rospy.Subscriber("robot_twist", Twist, twist_callback)	
    rospy.on_shutdown(cleanUp)
    rate = rospy.Rate(100)
    #Hz = times per second. 10ms = 100 times per second
    #wheeldrop, cliff, bumper subscribe
    rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent,wheelDropCallback)
    rospy.Subscriber("/mobile_base/events/cliff", CliffEvent,CliffCallback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, BumperCallback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    x = 0.0
    y = 0.0
    path = None
    firstTime = True
    done = False # used to kill after writing
    fileName = input('Enter Filename:')
	#trusty steed. true workhorse. Pie o My
    
    while not rospy.is_shutdown() and not done:

        #angular smoother
        angular_diff = abs(angular.goal - angular.curr)
        if angular_diff <= 0.001:# float == bad
            angular.curr = angular.curr
            if angular.goal ==0:
                angular.curr = 0
        elif (angular.curr < angular.goal):
            angular.curr += A_SMOOTHER#MORRRRE
        elif angular.curr > angular.goal:
            angular.curr -= A_SMOOTHER#LESSSSS
        angular_diff = abs(angular.goal - angular.curr)
        if angular_diff <= 0.001:# float == bad again
            angular.curr = angular.curr
            if angular.goal ==0:
                angular.curr = 0


	#linear smoothing
	linear_diff = abs(linear.goal - linear.curr)
        #floats suck. if close, it's there
	if linear_diff <= 0.001:
             linear.curr = linear.curr
	     if linear.goal == 0:
                 linear.curr = 0
		# wont stop on 0 eek, was going -.0000000006
        elif (linear.curr < linear.goal):#not there yet
            linear.curr += commands[6]*L_SMOOTHER
	    #array is mode, eco=1, sport=2 (sport double fast!)
        elif(linear.curr > linear.goal):#slow down!
             linear.curr -= commands[6]*L_SMOOTHER
	#again? yes.        
	linear_diff = abs(linear.goal - linear.curr)
        if linear_diff <= 0.001:
             linear.curr = linear.curr
             if linear.goal == 0:
                 linear.curr = 0# wont stop on 0 eek
	

	#smoothing off, use raw data
        if(commands[6] ==0):
		linear.curr = linear.goal
		angular.curr = angular.goal #smoothing off

	#huuuge set for our publishing
	adjusted_twist.linear.x = linear.curr
	adjusted_twist.angular.z = angular.curr
	
	#emergency brake, hard stop	
	if(commands[5] == 1):
		adjusted_twist.linear.x = 0
		adjusted_twist.angular.z = 0
		linear.curr = 0
		angular.curr = 0 
		
        #backwards only check if triggered
        if(commands[3] == 1 and sensor_triggered and linear.curr > 0): 
	#if backwardsonly is on and we are touching something, no move
                    adjusted_twist.linear.x = 0
                    adjusted_twist.angular.z = 0
	            linear.curr = 0
		    angular.curr = 0

	#threaded led activation, only when going backwards
	if  active_flash == False and adjusted_twist.linear.x < 0:
                led_thread = threading.Thread(target=flash_leds)
                sound_thread = threading.Thread(target=play_sound)
                led_thread.start()
                sound_thread.start()
	#start recording
	if(commands[9] == 1):
	    if(firstTime):
		firstTime = False
  		resetOdomPub.publish(Empty())
		rospy.sleep(1)
		path = []
    	    newx = odom.pose.pose.position.x
    	    newy = odom.pose.pose.position.y
    	    newDistance = math.sqrt(math.pow((newx - x), 2) + math.pow((newy - y), 2))
    	    if newDistance > .1:
        	x = newx
        	y = newy
        	path.append(str(round(x,2))+","+str(round(y,2))+"\n")
	else:
	    if not firstTime:
		done = True
		fd = open(fileName, 'w')
		for s in path:
		    fd.write(s)
		fd.close()
	velocity_pub.publish(adjusted_twist)
        rate.sleep()#^publish and wait for next cycle (10ms)

if __name__ == '__main__':
        try:
	    main()
	except rospy.ROSInterruptException:
	    pass
