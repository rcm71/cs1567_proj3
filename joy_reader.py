#!/usr/bin/env python

# methodlogy: We are going to treat wherever the robot is as 0,0
# this is done by a simple filecoord - odomcoord
# this allows us to know relationally where the point is to the robot
# q1 = angle, q2 = angle - 90, q3 = -angle - 90, q4 = angle + 90

import rospy
import math
from tf.transformations import euler_from_quaternion 
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

isOdomReady = False
odom = Odometry()



def odomCallback(data):
    global isOdomReady, odom
    isOdomReady = True
    odom = data

def findPathAngle(point):
    global odom
    splat = point.split(',') # "x,y"
    odomX = odom.pose.pose.position.x
    odomY = odom.pose.pose.position.y
    goalY = float(splat.pop()) - odomY
    goalX = float(splat.pop()) - odomX
    distance = math.sqrt((goalX)**2 + (goalY)**2) # hypotenuse
    angle = math.asin((goalY) / distance) # asin opp/hyp = theta
    if goalX < 0:
	angle = angle + (math.pi / 2)
    if goalY < 0:
	angle = angle * -1
    return angle

def findOdomAngle(odom):
    # Convert quaternion to degree
    q = [odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    # roll, pitch, and yaw are in radian:
    return yaw

# ok is this copy paste above? yes. need diff return tho...
def distanceFromPoint(point):
    global odom
    splat = point.split(',') # "x,y"
    goalY = float(splat.pop())
    goalX = float(splat.pop())
    odomX = odom.pose.pose.position.x
    odomY = odom.pose.pose.position.y
    distance = math.sqrt((goalX - odomX)**2 + (goalY - odomY)**2) # hypotenuse
    return distance

def main():
    fileName = input("input filename: ")
    path = open(fileName, 'r')
    coord1 = path.readline()
    coord2 = path.readline()
    coord3 = path.readline()
    rospy.init_node('coord_follower', anonymous = True)
    rate = rospy.Rate(100)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    twist = Twist()
    firstFound = False # have we seen first point? (angular)
    twistPub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 10)#10???? 
    firstError = True
    while not rospy.is_shutdown():
        actual = findOdomAngle(odom)
	angle1 = findPathAngle(coord1)
        angle2 = findPathAngle(coord2)
        angle3 = findPathAngle(coord3)
        averageAngle = (angle1 + angle2 + angle3) / 3
	if firstError:
	    olderror = averageAngle - actual
	    firstError = False
        DIFF_VAR = 1.2
	ERR_VAR = 1.2
	error = averageAngle - actual
	differential = error - olderror
	olderror = error

	twist.angular.z = (DIFF_VAR * differential +
			   ERR_VAR * error)
	print(str(round(actual,2)) + "\t" + str(round(averageAngle,2))+ "\t" + str(round(twist.angular.z)))        
        if abs(actual - averageAngle) < .1:
            firstFound = True
            twist.angular.z = 0
        if firstFound:
            twist.linear.x = .1
        else:
            twist.linear.x = 0
        if distanceFromPoint(coord1) < .05:
            coord1 = coord2
            coord2 = coord3
            # get get "" as return is at EOF
            # only write to coord3 if we have no more
            # would like all 3 final coord to be the same value,
            # so we hone in
            potentialPoint = path.readline()
            print("switched points: "+potentialPoint)
            if potentialPoint != "":
                coord3 = potentialPoint
        twistPub.publish(twist)
	rate.sleep()
    path.close()

if __name__ == "__main__":
    main()
