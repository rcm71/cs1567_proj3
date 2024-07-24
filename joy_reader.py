#!/usr/bin/env python

import rospy
import math 
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

isOdomReady = False
odom = Odometry()


def cleanUp():
    something

def odomCallback(data):
    global isOdomReady, odom
    isOdomReady = True
    odom = data

def findPathAngle(point):
    global odom
    splat = point.split(',') # "x,y"
    print(splat)
    goalY = splat.pop()
    goalX = splat.pop()
    odomX = odom.pose.pose.position.x
    odomY = odom.pose.pose.position.y
    distance = math.sqrt((goalX - odomX)**2 + (goalY - odomY)**2) # hypotenuse
    angle = math.asin((goalY - odomY) / distance) # asin opp/hyp = theta
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
    print(splat)
    goalY = splat.pop()
    goalX = splat.pop()
    odomX = odom.pose.pose.position.x
    odomY = odom.pose.pose.position.y
    distance = math.sqrt((goalX - odomX)**2 + (goalY - odomY)**2) # hypotenuse
    return distance

def main():
    path = open("path.txt", 'r')
    coord1 = path.readline()
    coord2 = path.readline()
    coord3 = path.readline()
    rospy.init_node('coord_follower', anonymous = True)
    rospy.on_shutdown(cleanUp)
    rate = rospy.Rate(100)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    twist = Twist()
    angle1 = findPathAngle(coord1)
    angle2 = findPathAngle(coord2)
    angle3 = findPathAngle(coord3)
    firstFound = False # have we seen first point? (angular)
    averageAngle = angle1 + angle2 + angle3 / 3
    twistPub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 10)#10???? 

    while not rospy.is_shutdown():
        actual = findOdomAngle(odom)

        if actual < averageAngle:
            twist.angular.z = .2
        elif actual > averageAngle:
            twist.angular.z = -.2
        if abs(actual - averageAngle) < .1:
            firstFound = True
            twist.angular.z = 0
        if firstFound:
            twist.linear.x = .2
        else:
            twist.linear.x = 0

        if distanceFromPoint(coord1) < .05:
            coord1 = coord2
            coord2 = coord3
            # get get "" as return is at EOF
            # only write to coord3 if we have no more
            # would like all 3 final coord to be the same value, so we hone in
            potentialPoint = path.readLine()
            if potentialPoint != "":
                coord3 = potentialPoint
            angle1 = angle2
            angle2 = angle3
            angle3 = findPathAngle(coord3)
            averageAngle = angle1 + angle2 + angle3 / 3



        twistPub.publish(twist)
    path.close()

if __name__ == "__main__":
    main()