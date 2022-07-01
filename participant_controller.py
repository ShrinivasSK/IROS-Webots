import casadi as ca
# from numpy import *
import numpy as np
import time
import math 
from scipy.spatial import KDTree                          							                                                           
import cv2                                                              
import sys
from controller import Robot
robot = Robot()

MAXIMUM_TIME = 3*60*1000
SPENT_TIME = 0

timestep = int(robot.getBasicTimeStep())
ld = robot.getDevice('lidar_tilt')                                
ld.enable(timestep)
															
ld.enablePointCloud()                                                       
ld.getHorizontalResolution()  

wall_extra_width = 3
max_path_length = 25

bot_angular_resolution = 5
waypoint_change_threshold = 3.5#1.5#3.5#2.5

arm_wait_counter = 125


pi = math.pi
t_start = time.time()
inf = np.inf
#   from controller import Robot
# robot = Robot()
# timestep = int(robot.getBasicTimeStep())
motor_left = robot.getDevice('wheel_left_joint')
motor_right = robot.getDevice('wheel_right_joint')
motor_left.setPosition(inf)    
motor_right.setPosition(inf)   
motor_left.setVelocity(0)                              
motor_right.setVelocity(0)                               
gps = robot.getDevice('gps')
gps.enable(timestep)
gps_ee = robot.getDevice('gps_ee')
gps_ee.enable(timestep)
IU = robot.getDevice('inertial unit')
IU.enable(timestep)

arm_1 = robot.getDevice('arm_1_joint')
arm_2 = robot.getDevice('arm_2_joint')
arm_3 = robot.getDevice('arm_3_joint')
arm_4 = robot.getDevice('arm_4_joint')

# arm_1.setPosition(90.0*3.14159/180.0)
# arm_2.setPosition(135.0*3.14159/180.0)
# arm_4.setPosition(-89.0*3.14159/180.0) # -15.0 or 45.0

# arm_1.setPosition(1.375)
# arm_2.setPosition(1.02)
# arm_3.setPosition(-3.3)
# arm_4.setPosition(2.29)#-3.46)
arm_1.setPosition(1.57)
arm_2.setPosition(1.02)
arm_3.setPosition(-3.2  )#-189*3.14159/180)
arm_4.setPosition(2.29)#0.0*3.14159/180.0)#-3.46)

counter_1 = 0
while (robot.step(timestep) != -1):     
	counter_1 += 1
	theta_odom =IU.getRollPitchYaw()[0]  + pi/2  
	if theta_odom > pi:
		theta_odom = theta_odom - 2*pi
	motor_left.setVelocity(0.0)             
	motor_right.setVelocity(0.0)

	if counter_1 >= arm_wait_counter:
		break

robot.step(timestep)
motor_left.setVelocity(0)             
motor_right.setVelocity(0)
robot.step(timestep)


# time.sleep(2)

# time.sleep(2)
# counter_ = 0
# while(1):
# 	counter_ += 1

# 	if counter_ >= 5000:
# 		break



''' ###############################
Mapping grids
'''
grid = np.zeros(shape=(100, 100), dtype='uint8')				##########
gridlines = np.zeros(shape=(100, 100), dtype='uint8')				#########
dst_4 = np.zeros(shape=(100, 100))
grid_unprocessed = np.zeros(shape=(100, 100), dtype='uint8')

grid_2 = grid.copy()
dst_3 = grid.copy()


for i in range(0,100):
	for j in range(0,100):
		if(i%10==0 or j%10==0):
			gridlines[i][j]=1



def pid(target_theta):

	global SPENT_TIME

	robot_dia = 0.5
	wheel_rad = 0.15
	omega_wheel_max = 10.15#5.15#10.15  ####################################
	

	# print("Adjusting Yaw")
	kp,ki,kd = 7.5, 0, 400#7.5, 0, 200#2, 0, 200#7.5, 0, 200#10,0,225										
	
	


	e_prev,sum_e = 0,0
	while (robot.step(timestep) != -1):     
	
		theta_odom =IU.getRollPitchYaw()[0]  + pi/2  
		if theta_odom > pi:
			theta_odom = theta_odom - 2*pi
		break	

	# if abs((target_theta*180/pi)-(theta_odom*180/pi)) > 135:					
	# 	kd = 325 								




	e = target_theta - theta_odom

	while ( abs(e) > 5e-3 and robot.step(timestep) != -1):  ####################################
		
		SPENT_TIME += timestep

		theta_odom =IU.getRollPitchYaw()[0]  + pi/2  
		if theta_odom > pi:
			theta_odom = theta_odom - 2*pi
		
		e = target_theta - theta_odom
		sum_e = sum_e + e
		OMEGA = kp*e + ki*sum_e + kd*(e - e_prev)
		VELOCITY = 0
		e_prev= e


	
		OMEGA_LEFT_WHEEL = (VELOCITY -OMEGA*robot_dia)/wheel_rad                         
		OMEGA_RIGHT_WHEEL = (VELOCITY +OMEGA*robot_dia)/wheel_rad

		OMEGA_LEFT_WHEEL = min(OMEGA_LEFT_WHEEL, omega_wheel_max)               
		OMEGA_LEFT_WHEEL = max(OMEGA_LEFT_WHEEL, -1 * omega_wheel_max)          
		OMEGA_RIGHT_WHEEL = min(OMEGA_RIGHT_WHEEL, omega_wheel_max)             
		OMEGA_RIGHT_WHEEL = max(OMEGA_RIGHT_WHEEL, -1 * omega_wheel_max)        

		motor_left.setVelocity(OMEGA_LEFT_WHEEL)             
		motor_right.setVelocity(OMEGA_RIGHT_WHEEL)
		

		# begin{please do not change}
		gps_ee_vals = gps_ee.getValues()
		robot.setCustomData(waypoints_string + ' ' + str(gps_ee_vals[0]) + ' ' + str(gps_ee_vals[1]))
		# end{please do not change}
		# cv2.imshow('w1', (dst_4*255).astype(np.uint8))
		# cv2.imshow('w2', (grid_unprocessed*255).astype(np.uint8))
		# cv2.waitKey(1)

		# print (theta_odom*180/pi)
	motor_left.setVelocity(0)             
	motor_right.setVelocity(0)	
	# print (theta_odom*180/pi)			


# def check_for_close_waypoints(x1, y1):
	
	

# 	pass


'''
Waypoints
'''
# Obtain waypoints
all_waypoints = []
waypoints_string = robot.getCustomData()
waypoints_split = waypoints_string.split()
for i in range(10):
	waypoints_element = [float(waypoints_split[2*i]), float(waypoints_split[2*i+1])]
	all_waypoints.append(waypoints_element)
# print('Waypoints:', waypoints)

# begin{please do not change}
if (len(waypoints_split) != 20):
	waypoints_string = ' '.join(waypoints_split[:20])
# end{please do not change}


# all_waypoints = []
# waypoints_string = robot.getCustomData().split()
# for i in range(10):
#     waypoints_element = [float(waypoints_string[2*i]), float(waypoints_string[2*i+1])]
#     all_waypoints.append(np.array(waypoints_element, dtype='f'))
# print('Waypoints:', all_waypoints)




'''
Daughter Waypoints
'''
daughter_waypoints = []
for i in range(0,len(all_waypoints)):

	if(all_waypoints[i][0]+1 >=-5 and all_waypoints[i][0]+1 <5 and all_waypoints[i][1] >=-5 and all_waypoints[i][1] <5):
		daughter_waypoints.append([all_waypoints[i][0]+1,all_waypoints[i][1]])
	else:
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1]])
	
	if(all_waypoints[i][0]-1 >=-5 and all_waypoints[i][0]-1 <5 and all_waypoints[i][1] >=-5 and all_waypoints[i][1] <5):
		daughter_waypoints.append([all_waypoints[i][0]-1,all_waypoints[i][1]])
	else:
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1]])
	
	if(all_waypoints[i][0] >=-5 and all_waypoints[i][0] <5 and all_waypoints[i][1] + 1 >=-5 and all_waypoints[i][1] + 1 <5):
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1] + 1])
	else:
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1]])
	
	if(all_waypoints[i][0] >=-5 and all_waypoints[i][0] <5 and all_waypoints[i][1] - 1 >=-5 and all_waypoints[i][1] - 1 <5):
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1] - 1])
	else:
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1]])
	

	if(all_waypoints[i][0]+1 >=-5 and all_waypoints[i][0]+1 <5 and all_waypoints[i][1] + 1 >=-5 and all_waypoints[i][1] + 1<5):
		daughter_waypoints.append([all_waypoints[i][0]+1,all_waypoints[i][1]+1])
	else:
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1]])
	
	if(all_waypoints[i][0]-1 >=-5 and all_waypoints[i][0]-1 <5 and all_waypoints[i][1]-1 >=-5 and all_waypoints[i][1]-1 <5):
		daughter_waypoints.append([all_waypoints[i][0]-1,all_waypoints[i][1]-1])
	else:
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1]])
	
	if(all_waypoints[i][0]+1 >=-5 and all_waypoints[i][0]+1 <5 and all_waypoints[i][1] - 1 >=-5 and all_waypoints[i][1] - 1 <5):
		daughter_waypoints.append([all_waypoints[i][0]+1,all_waypoints[i][1] - 1])
	else:
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1]])
	
	if(all_waypoints[i][0]-1 >=-5 and all_waypoints[i][0]-1 <5 and all_waypoints[i][1] + 1 >=-5 and all_waypoints[i][1] + 1 <5):
		daughter_waypoints.append([all_waypoints[i][0]-1,all_waypoints[i][1] + 1])
	else:
		daughter_waypoints.append([all_waypoints[i][0],all_waypoints[i][1]])



	# daughter_waypoints.append([all_waypoints[i][0]+1,all_waypoints[i][1]+1])
	# daughter_waypoints.append([all_waypoints[i][0]-1,all_waypoints[i][1]-1])
	# daughter_waypoints.append([all_waypoints[i][0]+1,all_waypoints[i][1]-1])
	# daughter_waypoints.append([all_waypoints[i][0]-1,all_waypoints[i][1]+1])

current_waypoint_index = 0
waypoint = daughter_waypoints[current_waypoint_index]
print(all_waypoints)


def KDTree_manual(list,x,y):
	global dst_3														
	minimum_dist = 1000
	minimum_index = 0
	for l in range(0,list.shape[0]):
		# print("CURRENT POINT: ", list[l][0], list[l][1])

		x_ = int(list[l][0])
		y_ = int(list[l][1])

		if(x_ >= 0 and x_ < 100 and y_ >= 0 and y_ < 100):
			if dst_3[x_][y_]!=1:	
				if ((x-list[l][0])**2 + (y-list[l][1])**2) < minimum_dist:
						minimum_dist = ((x-list[l][0])**2 + (y-list[l][1])**2)
						minimum_index = l
	return minimum_index


'''
Function to pop and rotate arm
'''
def pop_and_arm(index, x, y):
	global daughter_waypoints
	global all_waypoints
	# a=(index+1)%4
	# d=54
	# if a==0:
	# 	d=int((index+1)/4)
	# 	d=d-1
	# 	daughter_waypoints.pop(4*d)
	# 	daughter_waypoints.pop(4*d+1)
	# 	daughter_waypoints.pop(4*d+2)
	# 	daughter_waypoints.pop(4*d+3)
	# else:
	# 	if index>3:
	# 		d=int(((index+1)-(index+1)%4)/4)
	# 	else:
	# 		d=0
	# 	daughter_waypoints.pop(4*d)
	# 	daughter_waypoints.pop(4*d+1)
	# 	daughter_waypoints.pop(4*d+2)
	# 	daughter_waypoints.pop(4*d+3)

	a = index % 8
	d = int(index / 8)

	print("PARENT WAYPOINT: ", all_waypoints[d])
	print("DAUGHTER: ", daughter_waypoints[d])

	global ld
	lidar_values = ld.getRangeImageArray()

	bot_theta =IU.getRollPitchYaw()[0] + pi/2                          
	if bot_theta > pi :
		bot_theta = bot_theta - 2*pi

	bot_x = gps.getValues()[0]
	bot_y = gps.getValues()[1] 
	update_map(lidar_values, bot_x, bot_y, bot_theta)
	
	if(a == 0):
		print("Calling PID for angle: ", 180)
		pid(pi)
	elif(a == 1):
		print("Calling PID for angle: ", 0)
		pid(0)
	elif(a == 2):
		print("Calling PID for angle: ", -90)
		pid(-1 * pi/2)
	elif(a == 3):
		print("Calling PID for angle: ", 90)
		pid(pi/2)
	elif(a == 4):
		print("Calling PID for angle: ", -135)
		pid((-135) * pi / 180)
	elif(a == 5):
		print("Calling PID for angle: ", 45)
		pid((45) * pi / 180)
	elif(a == 6):
		print("Calling PID for angle: ", 135)
		pid((135) * pi / 180)
	else:
		print("Calling PID for angle: ", -45)
		pid((-45) * pi / 180)
	

	print("LEN of daughter waypoints (in func) (before): ", len(daughter_waypoints))
	print("Daughter waypoints (in func) (before): \n", daughter_waypoints)

	# print("POPING: ", daughter_waypoints[4*d], daughter_waypoints[4*d+1], daughter_waypoints[4*d+2], daughter_waypoints[4*d+3])

	daughter_waypoints.pop(8*d)
	daughter_waypoints.pop(8*d)
	daughter_waypoints.pop(8*d)
	daughter_waypoints.pop(8*d)
	daughter_waypoints.pop(8*d)
	daughter_waypoints.pop(8*d)
	daughter_waypoints.pop(8*d)
	daughter_waypoints.pop(8*d)

	all_waypoints.pop(d)

	print("LEN of daughter waypoints (in func): ", len(daughter_waypoints))
	print("Daughter waypoints (in func): \n", daughter_waypoints)




	# Printing all values
	# print(all_waypoints)	
	# print("WAYPOINT INDEX IN ORIGINAL LIST")
	# print(daughter_waypoints)
	# print("WAYPOINT DAUGHTERS")
	# print(d)
	# print("WAYPOINT_______________________")

	# Main code###########################
	# target_waypoint = all_waypoints[d]
	# x_target = (target_waypoint[0] + 5)*10
	# y_target = (target_waypoint[1] + 5)*10

	# arm_angle = np.arctan2(y_target - y1, x_target - x1)
	# pid(arm_angle)

	# all_waypoints.pop(d)
	##################################
	# print(all_waypoints)
	# print(daughter_waypoints)
	# print("WAYPOINT DAUGHTERS AFTER POP_____________")
	

	robot.step(timestep)	
	# arm_1.setPosition(1.375)
	# arm_2.setPosition(0)
	# arm_3.setPosition(0)
	# arm_4.setPosition(0)#-3.46)
	arm_1.setPosition(1.57)
	arm_2.setPosition(0)
	arm_3.setPosition(-3.2)
	# arm_4.setPosition(0.6)
	arm_4.setPosition(0.0)
	robot.step(timestep)
		# break

	counter_1 = 0
	while (robot.step(timestep) != -1):     
		counter_1 += 1
		theta_odom =IU.getRollPitchYaw()[0]  + pi/2  
		if theta_odom > pi:
			theta_odom = theta_odom - 2*pi
		motor_left.setVelocity(0.0)             
		motor_right.setVelocity(0.0)

		if counter_1 >= arm_wait_counter:
			break


	# begin{please do not change}
	gps_ee_vals = gps_ee.getValues()
	robot.setCustomData(waypoints_string + ' ' + str(gps_ee_vals[0]) + ' ' + str(gps_ee_vals[1]))
	# end{please do not change}
	

	# robot.step(timestep)
	# motor_left.setVelocity(0)             
	# motor_right.setVelocity(0)
	# robot.step(timestep)
		# break	

	# timestep = int(robot.getBasicTimeStep())
	# robot.step(timestep)
	# while(robot.step(timestep) != -1):


		# arm_1_pos = arm_1.getPosition()

	# motor_left.setVelocity(0.1)             
	# motor_right.setVelocity(0.1)

	# counter_ = 0
	# while(1):
	# 	counter_ += 1

	# 	if counter_ >= 100000000000:
	# 		break

	# while(robot.getBasicTimeStep() != -1):

	# timestep = int(robot.getBasicTimeStep())
	
	# while(robot.step(timestep)!=-1):
	# arm_1.setPosition(1.375)
	arm_1.setPosition(1.57)
	arm_2.setPosition(1.02)
	arm_3.setPosition(-3.2)#-189*3.14159/180)
	arm_4.setPosition(2.29)#0.0*3.14159/180.0)#-3.46)

	counter_1 = 0
	while (robot.step(timestep) != -1):     
		counter_1 += 1
		theta_odom =IU.getRollPitchYaw()[0]  + pi/2  
		if theta_odom > pi:
			theta_odom = theta_odom - 2*pi
		motor_left.setVelocity(0.0)             
		motor_right.setVelocity(0.0)

		if counter_1 >= arm_wait_counter:
			break
	
	# robot.step(timestep)
	# motor_left.setVelocity(0)             
	# motor_right.setVelocity(0)
	# robot.step(timestep)

		# break
	# arm_1.setPosition(1.375)
	# arm_2.setPosition(1.02)
	# arm_3.setPosition(-3.29)
	# arm_4.setPosition(2.29)#-3.46)
	
	# timestep = int(robot.getBasicTimeStep())
	# robot.step(timestep)

	# motor_left.setVelocity(0.0)             
	# motor_right.setVelocity(0.0)
	# 	break
	


'''
returns 2: waypoint_changed
returns 1: waypoint_not_changed
# returns 0: no_waypoint_available --> go to last waypoint
returns -1: course complete
'''
def check_reached_waypoint(x1, y1):
	global daughter_waypoints
	global current_waypoint_index
	global waypoint

	# Poping waypoint
	if(len(daughter_waypoints)):
		for i in range(len(daughter_waypoints)):
			if(math.sqrt(((daughter_waypoints[i][0] + 5) * 10 - x1)**2 + ((daughter_waypoints[i][1] + 5) * 10 - y1)**2) <= waypoint_change_threshold):
				# print("REACHED Waypoint: ", daughter_waypoints[i])
				# daughter_waypoints.pop(i)
				pop_and_arm(i, x1, y1)
				break
	else:
		for i in range(len(daughter_waypoints)):
			if(math.sqrt(((daughter_waypoints[i][0] + 5) * 10 - x1)**2 + ((daughter_waypoints[i][1] + 5) * 10 - y1)**2) <= waypoint_change_threshold):
				# print("REACHED Waypoint: ", daughter_waypoints[i])
				# daughter_waypoints.pop(i)
				pop_and_arm(i, x1, y1)
				return -1

	# Finding new waypoint
	if(len(daughter_waypoints)):
		# new_waypoint_index = KDTree(((np.array(daughter_waypoints[:-1]) + 5) * 10)).query(np.array([x1, y1]))[1]           #####
		new_waypoint_index = KDTree_manual((np.array(daughter_waypoints) + 5) * 10, x1, y1)
		waypoint = daughter_waypoints[new_waypoint_index]

		print("NEW WAYPOINT: ", waypoint)

		if(new_waypoint_index != current_waypoint_index):
			current_waypoint_index = new_waypoint_index
			return 2
		else:
			return 1
	else:
		# waypoint = daughter_waypoints[-1]
		current_waypoint_index = KDTree_manual((np.array(daughter_waypoints) + 5) * 10, x1, y1)
		waypoint = daughter_waypoints[current_waypoint_index]

		print("NEW WAYPOINT: ", waypoint)
		return 0



# def change_waypoint(x1, y1):
#     global all_waypoints
#     global current_waypoint_index
#     global waypoint


#     all_waypoints.pop(current_waypoint_index)

#     if(len(all_waypoints)):
#         current_waypoint_index = KDTree(((np.array(all_waypoints[:-1]) + 5) * 10)).query(np.array([x1, y1]))[1]           #####
#         waypoint = all_waypoints[current_waypoint_index]
#         return 1
#     else:
#         return 0


def KDTree_func(path,x,y):
	minimum_dist = 100
	minimum_index = 0
	for l in range(0,path.shape[0]):
		
		if ((x-path[l][0])**2 + (y-path[l][1])**2) < minimum_dist:
			minimum_dist = ((x-path[l][0])**2 + (y-path[l][1])**2)
			minimum_index = l

	return minimum_index



def getPointRel(angle_1,r):
	theta_1 = angle_1*pi/180
	x_1 = r*math.cos(theta_1)
	y_1 = r*math.sin(theta_1)+0.25
	return x_1,y_1

def transform(x,y,angle,bot_x,bot_y):									###
	theta = angle
	x,y=y,x
	x_ = math.cos(theta)*x-math.sin(theta)*y
	y_= math.sin(theta)*x+math.cos(theta)*y
	return x_+bot_x,y_+bot_y

def roundToTen(val):
	rem = val%10
	if(rem<5):
		val-=rem
	else:
		val+=10-rem
	return val


# cv2.namedWindow("w1",cv2.WINDOW_NORMAL)
# cv2.namedWindow("w2",cv2.WINDOW_NORMAL)


'''
Astar
'''
class Array2D:

	def __init__(self, w, h, mapdata=[]):
		self.w = w
		self.h = h
		if mapdata:
			self.data = mapdata
		else:
			self.data = [[0 for y in range(h)] for x in range(w)] 

	def __getitem__(self, item):
		return self.data[item]


class Point:
  

	def __init__(self, x4, y4):
		self.x = x4
		self.y = y4

	def __eq__(self, other):
		if self.x == other.x and self.y == other.y:
			return True
		return False

	def __str__(self):
		return '(x:{}, y:{})'.format(self.x, self.y)


class AStar:
	class Node:
		def __init__(self, point, endPoint, g=0):
			self.point = point  
			self.father = None  
			self.g = g  
			self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y)) * 10  

	def __init__(self, map2d, startPoint, endPoint, passTag=0):
  
		self.openList = []

		self.closeList = []

		self.map2d = map2d

		self.startPoint = startPoint
		self.endPoint = endPoint
		self.passTag = passTag

	def getMinNode(self):
		currentNode = self.openList[0]
		for node in self.openList:
			if node.g + node.h < currentNode.g + currentNode.h:
				currentNode = node
		return currentNode

	def pointInCloseList(self, point):

		for node in self.closeList:
			if node.point == point:
				return True
		return False

	def pointInOpenList(self, point):

		for node in self.openList:
			if node.point == point:
				return node
		return None

	def endPointInCloseList(self):
		for node in self.openList:
			if node.point == self.endPoint:
				return node
		return None

	def searchNear(self, minF, offsetX, offsetY):
	
		if minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.map2d.w - 1 or minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.map2d.h - 1:
			return

		if self.map2d[minF.point.x + offsetX][minF.point.y + offsetY] != self.passTag:
			return

		if self.pointInCloseList(Point(minF.point.x + offsetX, minF.point.y + offsetY)):
			return

		if offsetX == 0 or offsetY == 0:
			step = 10
		else:
			step = 14

		currentNode = self.pointInOpenList(Point(minF.point.x + offsetX, minF.point.y + offsetY))
		if not currentNode:
			currentNode = AStar.Node(Point(minF.point.x + offsetX, minF.point.y + offsetY), self.endPoint,
									 g=minF.g + step)
			currentNode.father = minF
			self.openList.append(currentNode)
			return

		if minF.g + step < currentNode.g:
			currentNode.g = minF.g + step
			currentNode.father = minF

	def setNearOnce(self, x5, y5):

		
		offset = 1
		points = [[-offset, offset], [0, offset], [offset, offset], [-offset, 0],
				  [offset, 0], [-offset, -offset], [0, -offset], [offset, -offset]]
		for point in points:
			if 0 <= x5 + point[0] < self.map2d.w and 0 <= y5+ point[1] < self.map2d.h:
				self.map2d.data[x5 + point[0]][y5 + point[1]] = 1

	def expansion(self, offset=0):
		
		for i in range(offset):
			barrierxy = list()  
			for x1 in range(self.map2d.w):
				for y1 in range(self.map2d.h):
					if self.map2d.data[x1][y1] not in [self.passTag, 'S', 'E']:
						barrierxy.append([x1, y1])

			for xy in barrierxy:
				self.setNearOnce(xy[0], xy[1])

	def correctPath(self,path):													################
		i=0
		while(i<len(path)-2):
			prev = path[i]
			cur=path[i+1]
			next=path[i+2]
			if(prev.x==cur.x and cur.x==next.x):
				prev.x=prev.x-prev.x%10+5
				path[i]=prev
			elif(prev.y==cur.y and cur.y==next.y):
				prev.y=prev.y-prev.y%10+5
				path[i]=prev
			else:
			## L case
				if(prev.x==cur.x):
					## first horizontal 
					## second vertical
					prev.x=prev.x-prev.x%10+5
					cur.x=cur.x-cur.x%10+5
					cur.y = cur.y - cur.y%10+5
				else:
					## first vertical 
					## second horizontal
					prev.y=prev.y-prev.y%10+5
					cur.x=cur.x-cur.x%10+5
					cur.y = cur.y - cur.y%10+5
				path[i]=prev
				path[i+1]=cur
				i+=1
			i+=1
		if i == len(path) - 2:
			cur=path[i]
			next=path[i+1]
			if(cur.x==next.x):
				cur.x=cur.x-cur.x%10+5
				next.x = next.x-next.x%10+5
			else:
				cur.y=cur.y-cur.y%10+5
				next.y = next.y-next.y%10+5
			path[i]=cur
			path[i+1]=next
		else:
			cur=path[i-1]
			next=path[i]
			if(cur.x==next.x):
				cur.x=cur.x-cur.x%10+5
				next.x = next.x-next.x%10+5
			else:
				cur.y=cur.y-cur.y%10+5
				next.y = next.y-next.y%10+5
			path[i-1]=cur
			path[i]=next
		return path

	
	'''
		Return point just before turn
	'''
	def segment_path(self, path):
		flag_ = 0 # 0: x left, 1: x right, 2: y top, 3: y bottom


		print('PATH:')
		for p in path:
		    print(p.x,p.y)
		print('###########################')

		if(len(path) < 2):
			return self.startPoint

		p1 = path[0]
		p2 = path[1]

		if (p2.x - p1.x > 0):
			flag_ = 1
		elif (p2.x - p1.x < 0):
			flag_ = 0
		else:
			if(p2.y - p1.y > 0):
				flag_ = 3
			else:
				flag_ = 2
		
		temp_counter = 1


		for i in range(2, len(path)):
			temp_counter += 1
			p1_ = path[i-1]
			p2_ = path[i]

			x_change = 0 # 0: no change, 1: left, 2: right
			y_change = 0 # 0: no change, 1: top, 2: bottom
  

			if(p2_.x - p1_.x > 0):
				x_change = 2
			elif(p2_.x - p1_.x < 0):
				x_change = 1
			
			if(p2_.y - p1_.y > 0):
				y_change = 2
			elif(p2_.y - p1_.y < 0):
				y_change = 1
			
			#print(x_change, y_change, flag_)

			if(x_change != 0 and y_change != 0):
				end_point = Point(0, 0)
			
				if(flag_ == 0):
					end_point.y = p1_.y
					end_point.x = p1_.x - 1

					if(y_change == 2):
						flag_ = 3
					else:
						flag_ = 2

				elif(flag_ == 1):
					end_point.y = p1_.y
					end_point.x = p1_.x + 1

					if(y_change == 2):
						flag_ = 3
					else:
						flag_ = 2

				elif(flag_ == 2):
					end_point.y = p1_.y - 1
					end_point.x = p1_.x

					if(x_change == 2):
						flag_ = 1
					else:
						flag_ = 0
				
				else:
					#print("P1: ", p1_.x, p1_.y, "FLAG: ", flag_)
					end_point.y = p1_.y + 1
					end_point.x = p1_.x

					if(x_change == 2):
						flag_ = 1
					else:
						flag_ = 0
			
				end_point.x = end_point.x - end_point.x%10 + 5
				end_point.y = end_point.y - end_point.y%10 + 5
				
				if temp_counter==2:  ####################################
					continue

				return end_point
			
			elif(x_change != 0):
				if (flag_ == 2 or flag_ == 3):

					if(x_change == 2):
						flag_ = 1
					else:
						flag_ = 0

					if temp_counter==2:  ####################################
						continue

					return p1_
				
				if(p2_.x - path[0].x + p2_.y - path[0].y >= max_path_length):
					return p2_
			
			elif(y_change != 0):
				if (flag_ == 0 or flag_ == 1):

					if(y_change == 2):
						flag_ = 3
					else:
						flag_ = 2

					if temp_counter==2:  ####################################
						continue

					return p1_
				
				if(p2_.x - path[0].x + p2_.y - path[0].y >= max_path_length):
					return p2_

			else:
				
				temp_counter -= 1

				if(p2_.x - path[0].x + p2_.y - path[0].y >= max_path_length):
					return p2_

				continue


		return path[-1]



	def start(self):
		startNode = AStar.Node(self.startPoint, self.endPoint)
		self.openList.append(startNode)
		while True:
			minF = self.getMinNode()
			self.closeList.append(minF)
			self.openList.remove(minF)

			#self.searchNear(minF, -1, 1)
			self.searchNear(minF, 0, 1)
			#self.searchNear(minF, 1, 1)
			self.searchNear(minF, -1, 0)
			self.searchNear(minF, 1, 0)
			#self.searchNear(minF, -1, -1)
			self.searchNear(minF, 0, -1)
			#self.searchNear(minF, 1, -1)

			'''
			self.searchNear(minF,0,-1)
			self.searchNear(minF, 0, 1)
			self.searchNear(minF, -1, 0)
			self.searchNear(minF, 1, 0)
			'''
			point = self.endPointInCloseList()
			if point: 
				cPoint = point
				pathList = []
				while True:
					if cPoint.father:
						pathList.append(cPoint.point)
						cPoint = cPoint.father
					else:
						# print((pathList)
						# print((list(reversed(pathList)))
						# print((pathList.reverse())
						return self.segment_path(self.correctPath(list(reversed(pathList))))				############
			if len(self.openList) == 0:
				return self.startPoint
###############################################


'''
Mapping
'''

def update_map(lidar_values, bot_x, bot_y, bot_theta):

	global grid
	global gridlines
	global dst_4
	global grid_unprocessed
	global grid_2
	global dst_3

	global wall_extra_width

	# Adding lidar points to grid without correction
	for idx in range(35,635, 3):							
		if lidar_values[int(idx)][0] <= 3.0:
			dist, angle = lidar_values[int(idx)], idx*240/666-30
			dist = dist[0]
			if(dist==np.inf):
				dist= -1000

			coordRel = getPointRel(angle,dist)
			x,y = transform(coordRel[0],coordRel[1],bot_theta,bot_x,bot_y)
			x_int= int((x+5)*10+0.5)
			y_int = int((y+5)*10+0.5)


			x_temp=x_int
			y_temp=y_int
			if(x_int<100 and y_int<100 and x_int>=0 and y_int>=0):
				grid[x_int][y_int]=1
				grid_unprocessed[x_int][y_int]=1
	
	dst = grid & gridlines

	# Applying 3x3 filter
	# output dst_2 
	grid = np.copy(dst)
	for i in range(1, 99):
		for j in range(1, 99):
			middle_sum = dst[i][j] + dst[i-1][j] + dst[i+1][j] + dst[i][j-1] + dst[i][j+1] #+ dst[i-2][j] + dst[i+2][j] + dst[i][j-2] + dst[i][j+2]
			corner_sum = dst[i-1][j-1] + dst[i+1][j+1] + dst[i-1][j+1] + dst[i+1][j-1] #+ dst[i-2][j-2] + dst[i+2][j+2] + dst[i-2][j+2] + dst[i+2][j-2]
			
			if not(middle_sum >= 2 and corner_sum == 0):
				grid[i][j] = 0

	# grid = np.copy(dst_2) 
	dst_3 = np.copy(grid)
	

	# Extrapolation
	for i in range(1, 99):
		for j in range(1, 99):
			if(grid[i][j] == 1):
				for k in range(-1 * wall_extra_width, wall_extra_width + 1):
					for l in range(-1 * wall_extra_width, wall_extra_width + 1):
						if (i + k < 100 and i + k >= 0 and j + l < 100 and j + l >=0):
							dst_3[i+k][j+l] = 1

	# Adding Boundaries
	for i in range(100):
		for j in range(3):
			dst_3[i][j] = 1
			dst_3[i][100 - j - 1] = 1
			dst_3[j][i] = 1
			dst_3[100 - j - 1][i] = 1
		
		dst_3[4][i] = 1
		dst_3[3][i] = 1

	
	'''
	For visualization
	'''

	for i in range(100):
		for j in range(100):
			if (dst_3[i][j] == 1):
				dst_4[i][j] = 1
			else:
				if (dst_4[i][j] == 1):
					dst_4[i][j] = 0
	
	x_bot_grid=int((bot_x+5)*10 + 0.5)				
	y_bot_grid=int((bot_y+5)*10 + 0.5)				
	dst_4[x_bot_grid][y_bot_grid] = 0.5



def generate_path(start_x, start_y, end_x, end_y):

	pass


def mpc(target_x,target_y):	
	
	global SPENT_TIME

	print (target_x,target_y)
	n_states = 3
	n_controls = 2
	N = 10       																					
	delta_T = 0.2 
	X_target = np.array([target_x,target_y,0], dtype = 'f')                                                                                                                                                
	error_allowed = 0.08 		 ###										                                                                                                                                                                  
	Q_x = 100  																
	Q_y = 100																	
	R1 = 50    
	R2 = 75
	error_allowed_in_g = 1e-100   
	n_bound_var = n_states                        
	x_bound_max = 4.725                                                                                                                                                                              
	x_bound_min = -4.725          
	y_bound_max = 4.725          
	y_bound_min = -4.725          
	theta_bound_max = inf              
	theta_bound_min = -inf             
	omega_wheel_max = 10.15
	robot_dia = 0.5
	wheel_rad = 0.15
	v_max = 1.5#omega_wheel_max*wheel_rad                                 
	v_min = 0#-v_max    																				
	omega_max = omega_wheel_max*wheel_rad/robot_dia	                            										
	omega_min = -omega_max
	global x,y,theta,V,omega
	while (robot.step(timestep) != -1):     
		x = gps.getValues()[0]
		y = gps.getValues()[1]
		theta =IU.getRollPitchYaw()[0]  + pi/2  
		if theta > pi:
			theta = theta - 2*pi
		break
	x_casadi =ca.SX.sym('x')                
	y_casadi = ca.SX.sym('y')
	theta_casadi = ca.SX.sym('theta')
	states =np.array([(x_casadi),(y_casadi),(theta_casadi)]) 
	n_states = states.size           
	v_casadi =ca.SX.sym('v')
	omega_casadi = ca.SX.sym('omega')
	controls = np.array([v_casadi,omega_casadi])      
	n_controls = controls.size      
	rhs = np.array([v_casadi*ca.cos(theta_casadi),v_casadi*ca.sin(theta_casadi),omega_casadi]) 
	f = ca.Function('f',[states,controls],[rhs]) 
	U = ca.SX.sym('U', n_controls,N)
	P = ca.SX.sym('P',1, n_states*2)
	X =ca.SX.sym('X', n_states, N+1)
	obj = 0
	g = []
	Q = ca.diagcat(Q_x, Q_y)                                                                                                                                                                         
	R = ca.diagcat(R1, R2)    
	for i in range(0,N):                                                                                                                                                           
		cost_pred_st = ca.mtimes(  ca.mtimes( (X[0:n_states-1,i] - P[n_states:n_states*2-1].reshape((n_states-1,1)) ).T , Q )  ,  (X[0:n_states-1,i] - P[n_states:n_states*2-1].reshape((n_states-1,1)) )  )  + ca.mtimes(  ca.mtimes( (U[0:n_controls,i]).T , R )  ,  U[0:n_controls,i]  )  
		obj = obj + cost_pred_st  
	obj = obj + ca.mtimes(  ca.mtimes( (X[0:n_states-1,N] - P[n_states:n_states*2-1].reshape((n_states-1,1)) ).T , Q )  ,  (X[0:n_states-1,N] - P[n_states:n_states*2-1].reshape((n_states-1,1)) )  )   
	pred_st = np.zeros((n_states,1))    
	for i in range(0,N+1):                                                                                                                                                                        
		if i == 0:
			g = ca.vertcat( g,( X[0:n_states,i] - P[0:n_states].reshape((n_states,1)) )  )                                                                                       
		else:
			f_value = f(X[0:n_states,i-1],U[0:n_controls,i-1])      
			pred_st = X[0:n_states,i-1] + delta_T*f_value           
			g = ca.vertcat( g,(X[0:n_states,i] - pred_st[0:n_states].reshape((n_states,1)) )  )                                                                               
	OPT_variables = X.reshape((n_states*(N+1),1))          
	OPT_variables = ca.vertcat( OPT_variables, U.reshape((n_controls*N,1)) )          
	nlp_prob ={
		   'f':obj,
		   'x':OPT_variables,
		   'g':g,
		   'p':P
		  }
	opts = {
		 'ipopt':
		{
		  'max_iter': 100,
		  'print_level': 0,
		  'acceptable_tol': 1e-8,
		  'acceptable_obj_change_tol': 1e-6
		},
		 'print_time': 0
		   }
	solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
	lbg = ca.DM.zeros(((n_states)*(N+1),1))                                                                                                                           
	ubg = ca.DM.zeros(((n_states)*(N+1),1))                                                                                                                           
	lbg[0:(n_states)*(N+1)] = - error_allowed_in_g                                                                                                                    
	ubg[0:(n_states)*(N+1)] =  error_allowed_in_g                                                                                                                     
	lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1)) 
	ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1)) 
	lbx[0:n_bound_var*(N+1):3] = x_bound_min                       
	ubx[0:n_bound_var*(N+1):3] = x_bound_max                       
	lbx[1:n_bound_var*(N+1):3] = y_bound_min                       
	ubx[1:n_bound_var*(N+1):3] = y_bound_max                       
	lbx[2:n_bound_var*(N+1):3] = theta_bound_min                   
	ubx[2:n_bound_var*(N+1):3] = theta_bound_max                       
	lbx[n_bound_var*(N+1):(n_bound_var*(N+1)+n_controls*N):2] = v_min                        
	ubx[(n_bound_var*(N+1)):(n_bound_var*(N+1)+n_controls*N):2] = v_max                      
	lbx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):2] = omega_min                
	ubx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):2] = omega_max                
	X_init = np.array([x,y,theta], dtype = 'f')
	P = np.concatenate((X_init, X_target))                                                                                                                                         
	initial_X = ca.DM.zeros((n_states*(N+1)))                           
	initial_X[0:n_bound_var*(N+1):3] = X_init[0]                        
	initial_X[1:n_bound_var*(N+1):3] = X_init[1]                        
	initial_X[2:n_bound_var*(N+1):3] = X_init[2]                        
	initial_con = ca.DM.zeros((n_controls*N,1))                       
	n_iter = 0

	counter_ = 0
	prev_error = 100000

	while ( ca.norm_2( P[0:n_states-1].reshape((n_states-1)) - X_target[0:n_states-1] ) > error_allowed and robot.step(timestep) != -1 ) :       
		
		SPENT_TIME += timestep

		current_error = ca.norm_2( P[0:n_states-1].reshape((n_states-1)) - X_target[0:n_states-1] )
		# print(current_error)

		if(abs(prev_error - current_error) <= 1e-3):  ####################################
			counter_ += 1
		else:
			counter_ = 0

		if counter_ >= 250:
			break

		prev_error = current_error        
		

		n_iter += 1 
		args = {
			'lbx':lbx,
			'lbg':lbg,	    
			'ubx':ubx,
			'ubg':ubg,
			'p':P,
			'x0':ca.vertcat(initial_X,initial_con),                                      
		   }
		sol = solver(
					
				 x0=args['x0'],
				   
				 lbx=args['lbx'],
				 ubx=args['ubx'],
				
				 lbg=args['lbg'],
				 ubg=args['ubg'],
				 p=args['p']
					  
				)           
		X_U_sol = sol['x']
		V = (X_U_sol[n_bound_var*(N+1)].full())[0][0]
		omega = (X_U_sol[n_bound_var*(N+1)+1].full())[0][0]
		omega_left_wheel = (V -omega*robot_dia)/wheel_rad                         
		omega_right_wheel = (V +omega*robot_dia)/wheel_rad

		omega_left_wheel = min(omega_left_wheel, omega_wheel_max)       											   
		omega_left_wheel = max(omega_left_wheel, -1 * omega_wheel_max)  
		omega_right_wheel = min(omega_right_wheel, omega_wheel_max)
		omega_right_wheel = max(omega_right_wheel, -1 * omega_wheel_max)

		motor_left.setVelocity(omega_left_wheel)             
		motor_right.setVelocity(omega_right_wheel)           
		x = gps.getValues()[0]
		y = gps.getValues()[1]                                                       
		theta =IU.getRollPitchYaw()[0] + pi/2                                        
		if theta > pi :
			theta = theta - 2*pi

		#########################################
		# global ld
		# lidar_values = ld.getRangeImageArray()

		# bot_x = x
		# bot_y = y
		# bot_theta = theta
		# update_map(lidar_values, bot_x, bot_y, bot_theta)
		#########################################

		P[0:n_states] =([x,y,theta])               
		#print (V)
		for i in range(0,N*n_bound_var):                          
			initial_X[i] = X_U_sol[i+n_bound_var]                 
		for i in range(0,(N-1)*n_controls):                     
			initial_con[i] = X_U_sol[n_bound_var*(N+1)+i+n_controls]   
		

		# begin{please do not change}
		gps_ee_vals = gps_ee.getValues()
		robot.setCustomData(waypoints_string + ' ' + str(gps_ee_vals[0]) + ' ' + str(gps_ee_vals[1]))
		# end{please do not change}
		
		# cv2.imshow('w1', (dst_4*255).astype(np.uint8))
		# cv2.imshow('w2', (grid_unprocessed*255).astype(np.uint8))
		# cv2.waitKey(1)

	motor_left.setVelocity(0)             
	motor_right.setVelocity(0)     
	print("reached")





	   

# mpc(-2.5,-4.5)

# pid(90*pi/180)	 

# mpc(-2.5,-2.5)
# pid(0*pi/180)
# mpc(-0.5,-2.5)
# pid(90*pi/180)
# mpc(-0.5,-0.5)	

# pid(-90*pi/180)


#print('YOOOOOOOOOO___0')
'''
Main Loop
'''
while ( robot.step(timestep) != -1 ) and SPENT_TIME < MAXIMUM_TIME:


	# print('YOOOOOOOOOO___-1')
	x = gps.getValues()[0]
	y = gps.getValues()[1]  

	theta =IU.getRollPitchYaw()[0] + pi/2                          
	if theta > pi :
		theta = theta - 2*pi
	theta_2 = theta


	lidar_values = ld.getRangeImageArray()
	bot_x = x
	bot_y = y
	bot_theta = theta_2

	# print('BOT x, y, theta: ', bot_x, bot_y, bot_theta)


	# print('YOOOOOOOOOO___1')
	'''
	Mapping
	'''
	update_map(lidar_values, bot_x, bot_y, bot_theta)

	x1=(bot_x+5)*10 + 0.5              
	y1=(bot_y+5)*10 + 0.5


	# print('YOOOOOOOOOO___2')

	# print(waypoint[0], waypoint[1], x1, y1)
	# if(math.sqrt(((waypoint[0] + 5) * 10 - x1)**2 + ((waypoint[1] + 5) * 10 - y1)**2) <= waypoint_change_threshold):
	#     waypoint_available_ =  change_waypoint(x1, y1)
	#     print("WP_reached, New waypoint: ", waypoint)
	#     if not (waypoint_available_):
	#         break

	waypoint_available = check_reached_waypoint(x1, y1)
	if(waypoint_available == -1):
		print('RUN COMPLETED')
		break
	elif(waypoint_available == 0):
		print('GOING to LAST POINT')


	# print('\nCURRENT WAYPOINT: ', waypoint)
	# print()


	# begin{please do not change}
	gps_ee_vals = gps_ee.getValues()
	robot.setCustomData(waypoints_string + ' ' + str(gps_ee_vals[0]) + ' ' + str(gps_ee_vals[1]))
	# end{please do not change}



	'''
	A star
	'''
	pStart, pEnd = Point(int(y1),int(x1)), Point(int((waypoint[1]+5)*10 + 0.5),int((waypoint[0]+5)*10 + 0.5)) 

	size=(100, 100)
	map2d = Array2D(size[0], size[1])
	for i in range(size[0]):
		for j in range(size[1]):
			map2d[i][j]=dst_3[j][i]
	

	aStar = AStar(map2d, pStart, pEnd)
	aStar.expansion(offset=0)
	end_point = aStar.start()
	print("END Point: ", end_point.y, end_point.x, "Bot current point: ", x1, y1)
	end_point.x, end_point.y = end_point.y, end_point.x 	

	# pid(bot_theta + 5 * pi / 180)
	
	# arm_1.setPosition(1.375)
	# arm_2.setPosition(1.02)
	# arm_3.setPosition(-3.29)
	# arm_4.setPosition(2.29)#-3.46)


	print("CURRENT WAYPOINT: ", (np.array(waypoint)+5)*10)
	print("DAUGHTERS LIST: ")
	print((np.array(daughter_waypoints)+5)*10)
	print("CURRENT POSITION: ", x1, y1)


	'''
	PID
	'''
	bot_angle_ = 0
	bot_theta = bot_theta * 180 / pi
	# if(abs(bot_theta - 90) <= bot_angular_resolution):
	#     bot_angle_ = 90
	# elif(abs(bot_theta + 90) <= bot_angular_resolution):
	#     bot_angle_ = -90
	# elif(abs(bot_theta - 0) <= bot_angular_resolution):
	#     bot_angle_ = 0
	# elif((abs(bot_theta - 180) <= bot_angular_resolution) or (abs(bot_theta + 180) <= bot_angular_resolution)):
	#     bot_angle_ = 180

	bot_angle_ = bot_theta

	
	x1_rounded = x1 - x1 % 10 + 5
	y1_rounded = y1 - y1 % 10 + 5

	req_angle = np.arctan2(end_point.y - y1_rounded, end_point.x - x1_rounded) * 180 / pi

	# print("REQUIRED ANGLE: ", req_angle)

	call_pid_ = 0

	if(abs(req_angle - 90) <= bot_angular_resolution):
		req_angle = 90
		call_pid_ = 1
	elif(abs(req_angle + 90) <= bot_angular_resolution):
		req_angle = -90
		call_pid_ = 1
	elif(abs(req_angle - 0) <= bot_angular_resolution):
		req_angle = 0
		call_pid_ = 1
	elif((abs(req_angle - 180) <= bot_angular_resolution) or (abs(req_angle + 180) <= bot_angular_resolution)):
		req_angle = 180
		call_pid_ = 1


	print("ANGLE: ", bot_angle_, req_angle)
	#if(bot_angle_ == req_angle):
	# if(abs(bot_angle_ - req_angle) < 1):
	# 	pass
	# else:
	# 	# if (call_pid_ == 1):
	# 		# if (req_angle == 180):
	# 		# 	if (bot_angle_ < 0):
	# 		# 		req_angle = -179.99
	# 		print('Calling PID for angle: ', req_angle)
	# 		pid(req_angle * pi / 180)
	# 		continue
	
	pid(req_angle * pi / 180)

	x = gps.getValues()[0]
	y = gps.getValues()[1]  

	theta =IU.getRollPitchYaw()[0] + pi/2                          
	if theta > pi :
		theta = theta - 2*pi
	theta_2 = theta


	lidar_values = ld.getRangeImageArray()
	bot_x = x
	bot_y = y
	bot_theta = theta_2

	# print('BOT x, y, theta: ', bot_x, bot_y, bot_theta)


	# print('YOOOOOOOOOO___1')
	'''
	Mapping
	'''
	update_map(lidar_values, bot_x, bot_y, bot_theta)

	x1=(bot_x+5)*10 + 0.5              
	y1=(bot_y+5)*10 + 0.5

	
	'''
	MPC
	'''
	# if(math.sqrt(((waypoint[0] + 5) * 10 - x1)**2 + ((waypoint[1] + 5) * 10 - y1)**2) <= waypoint_change_threshold):
	#     waypoint_available_ =  change_waypoint(x1, y1)
	#     print("WP_reached, New waypoint: ", waypoint)
	#     if not (waypoint_available_):
	#         break

	waypoint_changed_ = check_reached_waypoint(x1, y1)

	if(waypoint_changed_ == 2):
		continue
	elif(waypoint_changed_ == -1):
		print('RUN COMPLETED')
		break
	# elif(waypoint_changed_ == 0):
	#     print('GOING to LAST POINT')
	else:
	   
		end_point.x = float((end_point.x / 10.0) - 5)
		end_point.y = float((end_point.y / 10.0) - 5)

		x_temp = float((x1 / 10.0) - 5)
		y_temp = float((y1 / 10.0) - 5)

		if(math.sqrt((end_point.x - x_temp) ** 2 + (end_point.y - y_temp) ** 2) < 0.08):
			motor_left.setVelocity(-0.2)             
			motor_right.setVelocity(-0.2)
		else:
			mpc(end_point.x, end_point.y)
		
  
	# cv2.imwrite("map.jpg", (dst_4*255).astype(np.uint8))
	# cv2.imshow('w1', (dst_4*255).astype(np.uint8))
	# cv2.imshow('w2', (grid_unprocessed*255).astype(np.uint8))
	# cv2.waitKey(1)

	SPENT_TIME += timestep
		
