#!/usr/bin/env python
import rospy
#import xmlrpclib
import numpy as np
from operator import itemgetter
from _ast import Or
import numpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

startPose = 0
openList = []
closeList = []
straightCost=1
diagnolCost=1.4
maze = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]

r= len(maze)
c= len(maze[0]) 
h = np.zeros((r,c))
g = np.zeros(r*c, dtype = int)
parent = np.zeros(r*c, dtype = int)
#Shifting the origin 
startx = 0
starty = 0
endx = 0
endy = 0
start=0
def main():
    global openList,closeList,g,f,h,startx,starty,endx,endy,start
    #print ('HIIIII')
    
    goalx = np.floor(rospy.get_param('astar/goalx'))
    goaly = np.floor(rospy.get_param('astar/goaly'))
    startx = np.floor(rospy.get_param('astar/startx'))
    starty = np.floor(rospy.get_param('astar/starty'))
    #print 'PARAMS:::', startx,starty,endx,endy
    startx = int(8-startx)
    starty = int(9+starty)
    endx = int(8-goalx)
    endy = int(9+goaly)
    #print 'PARAMS:::', startx,starty,endx,endy
    start=c*startx+starty
    openList=maze[startx][starty]
    computeHeuristicCost()
    g[c*startx+starty]=0.0
    openList = [[startx,starty,0,0]] #x,y,parent,fValue
    parent[c*startx+starty]=9999
    #print (len(openList))
    p=c*startx+starty
    #findMinF(openList)
    while(len(openList)>0):
        #print 'IN WHILE'
        #print openList[0][3]
        minimum=findMinF()
        #print ('Element work:', c*minimum[0]+minimum[1])
        #print ('######################')
        openList.remove(minimum)
       # print minimum[0],minimum[1]
        maze[minimum[0]][minimum[1]]=3
        
        #parent[4*minimum[0]+ minimum[1]]=p
        closeList.append(minimum)
        if numpy.abs(minimum[0]-endx)<=1 and numpy.abs(minimum[1]-endy)<=1:#stoppig when reached 1 block near goal
            break
        findAdjacentCellsValues(minimum)
        #print len(openList)
        #for i in range(len(openList)):
         #   for j in range(len(openList[i])):
                    #print(maze[i][j])
                    #print(openList[i][j]),
          #  print
    findPath(minimum)
    #print (parent)


def update_pose(data):
        global startPose
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        startPose = data
        startPose.x = round(startPose.x, 4)
        startPose.y = round(startPose.y, 4)

def euclidean_distance(goal_pose):
	global startPose
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - startPose.x), 2) +
                    pow((goal_pose.y - startPose.y), 2))

def linear_vel(goal_pose, constant=1.5):
	global startPose
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * euclidean_distance(goal_pose)

def steering_angle(goal_pose):
	global startPose
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - startPose.y, goal_pose.x - startPose.x)

def angular_vel(goal_pose, constant=6):
	global startPose
	"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (steering_angle(goal_pose) - startPose.theta)





def move2goal(x,y):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = x
        goal_pose.y = y

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.01

        vel_msg = Twist()

        while euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = angular_vel(goal_pose)

            # Publishing our vel_msg
            velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()



def findPath(n):
    global start,openList
    print ("PATH")
    i = c*n[0]+n[1]
    print(8-int(i/c),i%c-9)
    while not rospy.is_shutdown():
    	while(i!=start):
        	next = n[2]
        	row = int(i/c)
        	column = i%c
        	#i=maze[row][column]
		print(8-row,column-9)
        	i= parent[i]
        	#print (i)
    
    #Moving the robot
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, update_pose)
    pose = Pose()
    rate = rospy.Rate(10)
    
    print ("PATH")
    i = c*n[0]+n[1]
    move2goal(8-int(i/c),i%c-9)
    while not rospy.is_shutdown():
	twist = Twist()
	pub.publish(twist)
	rate.sleep()
    	print (i)
    	while(i!=start):
        	next = n[2]
        	row = int(i/c)
        	column = i%c
        	#i=maze[row][column]
		print(8-row,column-9)
		move2goal(8-row,column-9)
        	i= parent[i]
        	#print (i)
    

    

def findAdjacentCellsValues(n):
    global openList,g
    x=n[0]
    y=n[1]
    x1=x-1
    y1=y-1
    x2=x+1
    y2=y+1
    
    
    if x>0 and y>0 and maze[x1][y1]==0 and maze[x1][y1]!=3 :
        print (c*x1+y1)
        if maze[x1][y1]==2:
            newCost=g[c*x+y]+14
            if newCost<g[c*x1+y1]:
                element=[x1,y1,x+y,n[3]+14]
                maze[x1][y1]=2
                openList.append(element)
                g[c*x1+y1]=g[c*x+y]+14
                parent[c*x1+y1]=c*x+y
        else:    
            element=[x1,y1,x+y,n[3]+14]
            maze[x1][y1]=2
            openList.append(element)
            g[c*x1+y1]=g[c*x+y]+14
            parent[c*x1+y1]=c*x+y
    if x<r-1 and y>0 and maze[x2][y1]==0 and maze[x2][y1]!=3 :
        print (c*x2+y1)
        if maze[x2][y1]==2:
            newCost=g[c*x+y]+14
            if newCost<g[c*x2+y1]:
                element=[x2,y1,x+y,n[3]+14]
                maze[x2][y1]=2
                openList.append(element)
                g[c*x2+y1]=g[c*x+y]+14 
        else:    
            element=[x2,y1,x+y,n[3]+14]
            maze[x2][y1]=2
            openList.append(element)
            g[c*x2+y1]=g[c*x+y]+14
  	    parent[c*x2+y1]=c*x+y
    if x<r-1 and y<c-1 and maze[x2][y2]==0 and maze[x2][y2]!=3 :
        print (c*x2+y2)
        if maze[x2][y2]==2:
            newCost=g[c*x+y]+14
            if newCost<g[c*x2+y2]:
                element=[x2,y2,x+y,n[3]+14]
                maze[x2][y2]=2
                openList.append(element)
                g[c*x2+y2]=g[c*x+y]+14 
                parent[c*x2+y2]=4*x+y
        else:    
            element=[x2,y2,x+y,n[3]+14]
            maze[x2][y2]=2
            openList.append(element)
            g[c*x2+y2]=g[c*x+y]+14
            parent[c*x2+y2]=c*x+y
    if x>0 and y<c-1 and maze[x1][y2]==0 and maze[x1][y2]!=3 :
        print (c*x1+y2)
        if maze[x1][y2]==2:
            newCost=g[c*x+y]+14
            if newCost<g[c*x1+y2]:
                element=[x1,y2,x+y,n[3]+14]
                maze[x1][y2]=2
                openList.append(element)
                g[c*x1+y2]=g[c*x+y]+14 
                parent[c*x1+y2]=c*x+y
        else:    
            element=[x1,y2,x+y,n[3]+14]
            maze[x1][y2]=2
            openList.append(element)
            g[c*x1+y2]=g[c*x+y]+14
            parent[c*x1+y2]=c*x+y
    if x>0 and maze[x1][y]==0 and maze[x1][y]!=3 :
        print (c*x1+y)
        if maze[x1][y]==2:
            newCost=g[c*x+y]+14
            if newCost<g[c*x1+y]:
                element=[x1,y,x+y,n[3]+14]
                maze[x1][y]=2
                openList.append(element)
                g[c*x1+y]=g[c*x+y]+14 
                parent[c*x1+y]=c*x+y
        else:    
            element=[x1,y,x+y,n[3]+14]
            maze[x1][y]=2
            openList.append(element)
            g[c*x1+y]=g[c*x+y]+14
            parent[c*x1+y]=c*x+y
    if x<r-1 and maze[x2][y]==0 and maze[x2][y]!=3 :
        print (c*x2+y)        
        if maze[x2][y]==2:
            newCost=g[c*x+y]+14
            if newCost<g[c*x2+y]:
                element=[x2,y,x+y,n[3]+10]
                maze[x2][y]=2
                openList.append(element)
                g[c*x2+y]=g[c*x+y]+10
                parent[c*x2+y]=c*x+y
        else:    
            element=[x2,y,x+y,n[3]+10]
            maze[x2][y]=2
            openList.append(element)
            g[c*x2+y]=g[c*x+y]+10
            parent[c*x2+y]=c*x+y
    if y>0 and maze[x][y1]==0 and maze[x][y1]!=3 :
        print (c*x+y1)
        if maze[x][y1]==2:
            newCost=g[c*x+y]+14
            if newCost<g[c*x+y1]:
                element=[x,y1,x+y,n[3]+10]
                maze[x][y1]=2
                openList.append(element)
                g[c*x+y1]=g[c*x+y]+10
                parent[c*x+y1]=c*x+y
        else:    
            element=[x,y1,x+y,n[3]+10]
            maze[x][y1]=2
            openList.append(element)
            g[c*x+y1]=g[c*x+y]+10
            parent[c*x+y1]=c*x+y
    if y<c-1 and maze[x][y2]==0 and maze[x][y2]!=3 :
        print (c*x+y2)
        
        if maze[x][y2]==2:
            newCost=g[c*x+y]+14
            if newCost<g[c*x+y2]:
                element=[x,y2,x+y,n[3]+10]
                maze[x][y2]=2
                openList.append(element)
                g[c*x+y2]=g[c*x+y]+10
                parent[c*x+y2]=c*x+y
        else:    
            element=[x,y2,x+y,n[3]+10]
            maze[x][y2]=2
            openList.append(element)
            g[c*x+y2]=g[c*x+y]+10
            parent[c*x+y2]=c*x+y
    
        
    
def findMinF():
    openList.sort(key=itemgetter(3), reverse=False)
    return openList[0]    





def computeHeuristicCost():
    for i in range(len(maze)):
            for j in range(len(maze[i])):
                    #print(maze[i][j])
                    h[i][j] = np.sqrt((i-endx)**2+ (j-endy)**2)
                    print(h[i][j]),
            print

    

if __name__=="__main__":
    rospy.init_node('astar', anonymous=True)
    main()






### GO TO GOAL Reference: http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
