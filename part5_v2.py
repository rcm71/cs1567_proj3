#!/usr/bin/python

#import this and that
import rospy, math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

############################################################

#twist
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
twist = Twist()

#Odom
x=0
y = 0
degree = 0

class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
	return self.x +","+self.y


def idealPosition(startCoordinate,endCoordinate, obsCoordinate,intersection):
    if endCoordinate.x - startCoordinate.x == 0:
	return Coordinate(obsCoordinate.x + .5, obsCoordinate.y)
    ogSlope = (endCoordinate.y-startCoordinate.y)  / (endCoordinate.x - startCoordinate.x)
    
    desiredDistance = .5
    
    if ogSlope != 0:
        newSlope = -1/ogSlope
    else:
	return Coordinate(obsCoordinate.x, obsCoordinate.y - .5)
    ##I dont think will work..
    
    #m = math.tan(math.atan(ogSlope))

    newX = intersection.x + desiredDistance *10 * math.cos(math.atan(ogSlope))
    newY = intersection.y + desiredDistance *10 * math.sin(math.atan(ogSlope))
    
    coord = Coordinate(newX, newY) 
    #print(coord.x, coord.y)
    return coord

def intersectionPoint(start, end, obs):
    if end.x - start.x == 0:
	return Coordinate(start.x, obs.y)
    ogSlope = (end.y - start.y) / (end.x - start.x)
    ogIntercept = (-ogSlope * end.x) + end.y
    if ogSlope != 0:
        newSlope = -1/ogSlope
    else:
	return Coordinate(obs.x,start.y)
    newIntercept =( -newSlope * obs.x) + obs.y
    xIntercept = (newIntercept - ogIntercept) / (ogSlope - newSlope)
    yIntercept = newSlope * xIntercept + newIntercept
    return Coordinate(xIntercept, yIntercept)
############################################################

#return Odom degree
def odomCallback(data):
    global x,y,degree, yaw

    # Convert quaternion to degree
    q = [data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    
    degree = yaw * 180 / math.pi
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

############################################################

#return the turn angle in RADIAN
def calculate_angle_and_distnace(coord):
    global x,y
    x1 = coord.x
    y1 = coord.y
#    print(x1,y1)
    angle = math.atan2(y1-y, x1-x)
    
    distance = math.sqrt(math.pow((x1-x),2) + math.pow((y1-y),2))

    return angle, distance

############################################################

def turn_the_robot(turning_degree_in_rad):

    global twist, yaw
    if 	yaw < turning_degree_in_rad:
        twist.angular.z = 0.3
    else:
	    twist.angular.z = -.3
    if abs(yaw - turning_degree_in_rad) < .05:
	    twist.angular.z = 0
 

############################################################

def move_the_robot(distance):
    #print("moving...")
    #code
    global twist, x
    if distance > .05:
	    twist.linear.x = .2
    else:
	    twist.linear.x = 0
    
    #needs PID


############################################################
def distanceBetween(coord1, coord2):
    return math.sqrt((coord1.x - coord2.x)**2 + (coord1.y - coord2.y)**2)


def main():
    global twist,pub
    rospy.init_node('part5', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rate = rospy.Rate(10)    
    start = Coordinate(0,0)
    list_of_coordinates = [start]

    while not rospy.is_shutdown():
        user_input = raw_input("Enter a coordinate 'x,y' or 'n' to stop: \n")
        
        if user_input == 'n':
            break
        else:
            try:
                # Split the input by comma and strip any whitespace
                x_str, y_str = user_input.split(',')
                # Convert the split strings to integers
                x1 = float(x_str.strip())
                y1 = float(y_str.strip())
                # Append the tuple to the list
                coord = Coordinate(x1,y1)
		list_of_coordinates.append(coord)
            except ValueError:
                print("Invalid input. Please enter coordinates in the form 'x,y'.")
        adjusting = True
    #end of while loop for coord inputs

    while not rospy.is_shutdown():
        user_input = raw_input("Enter an obstacle coord 'x,y'\n")
        try:
            # Split the input by comma and strip any whitespace
            x_str, y_str = user_input.split(',')
            # Convert the split strings to integers
            x1 = float(x_str.strip())
            y1 = float(y_str.strip())
            # Append the tuple to the list
            obsCoord = Coordinate(x1,y1)
            break
        except ValueError:
            print("Invalid input. Please enter coordinates in the form 'x,y'.") 


    while adjusting and not rospy.is_shutdown():
        adjusting = False
        newList = []
        for i in range(len(list_of_coordinates) - 1):
            currCoord = list_of_coordinates[i]
            nextCoord = list_of_coordinates[i + 1]
            intersection = intersectionPoint(currCoord, nextCoord, obsCoord)
            newList.append(currCoord)
            if distanceBetween(obsCoord, intersection) <= 0.36:
                adjusting = True
                newCoord = idealPosition(currCoord, nextCoord, obsCoord, intersection)
                newList.append(newCoord)
        newList.append(list_of_coordinates[-1])
        list_of_coordinates = newList
    
    print(list_of_coordinates)






    for coord in list_of_coordinates:
    #Read each coordinate
        turning_degree_in_rad, distance_to_move = calculate_angle_and_distnace(coord)
        #print(turning_degree_in_rad,distance_to_move)
        while not rospy.is_shutdown() and abs(yaw - turning_degree_in_rad) > .05: 
        #calculate the angle
            turning_degree_in_rad, distance_to_move = calculate_angle_and_distnace(coord)
        #turn to angle of the next coordinate
            turn_the_robot(turning_degree_in_rad)
	    pub.publish(twist)
            rate.sleep()
	    #print(x, coord.x)
        while not rospy.is_shutdown() and distance_to_move > .03:
        #move the robot to the next coordinate
	    turning_degree_in_rad, distance_to_move = calculate_angle_and_distnace(coord)
            turn_the_robot(turning_degree_in_rad)
            move_the_robot(distance_to_move)
	    pub.publish(twist)
            rate.sleep()
        twist.linear.x = 0
    #end of the for loop
    
    #stop the robot
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)


############################################################
if __name__ == '__main__':
    main()
