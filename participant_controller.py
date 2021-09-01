import casadi as ca
import numpy as np									
import time
import math 
from scipy.spatial import KDTree                          							                                                           
import cv2                                                              
import sys
from controller import Robot
robot = Robot()                                                              

pi = math.pi
t_start = time.time()
inf = np.inf
print( "***************************************************")
# waypoint = np.array([3.5,-4.5], dtype ='f')

# all_waypoints = [#np.array([-4.5,-4.5], dtype ='f'),
# 				 np.array([-3.5,-3.5], dtype ='f'),
# 				 #np.array([-2.5,-2.5], dtype ='f'),
# 				 np.array([-1.5,-1.5], dtype ='f'),
# 				 #np.array([-0.5,-0.5], dtype ='f'),
# 				 np.array([0.5,0.5], dtype ='f'),
# 				 #np.array([1.5,1.5], dtype ='f'),
# 				 np.array([3.5,3.5], dtype ='f'),
# 				 np.array([4.5,4.5], dtype ='f'),
# 				]

all_waypoints = []
waypoints_string = robot.getCustomData().split()
for i in range(10):
	waypoints_element = [float(waypoints_string[2*i]), float(waypoints_string[2*i+1])]
	all_waypoints.append(np.array(waypoints_element, dtype='f'))
print('Waypoints:', all_waypoints)

# all_waypoints = [#np.array([-4.5,-4.5], dtype ='f'),
# 				 np.array([2.5,-4.5], dtype ='f'),
# 				#  np.array([-3.5,-4.5], dtype ='f'),
# 				#  np.array([3.5,3.5], dtype ='f'),
# 				#  np.array([-1.5,-1.5], dtype ='f'),
# 				#  np.array([-0.5,-0.5], dtype ='f'),
# 				#  np.array([0.5,0.5], dtype ='f'),
# 				#  np.array([4.5,4.5], dtype ='f'),
# 				#  np.array([-3.5,-3.5], dtype ='f'),
# 				#  np.array([1.5,1.5], dtype ='f'),
# 				#  np.array([2.5,2.5], dtype ='f')
				# ]

current_waypoint_index = 0
waypoint = all_waypoints[current_waypoint_index]

def KDTree_func(path,x,y):
	minimum_dist = 100
	minimum_index = 0
	for l in range(0,path.shape[0]):
		
		if ((x-path[l][0])**2 + (y-path[l][1])**2) < minimum_dist:
			minimum_dist = ((x-path[l][0])**2 + (y-path[l][1])**2)
			minimum_index = l

	return minimum_index
	


#####################################################                                                       
grid = np.zeros(shape=(100, 100), dtype='uint8')				##########
gridlines = np.zeros(shape=(100, 100), dtype='uint8')				#########
dst_4 = np.zeros(shape=(100, 100))

# cv2.namedWindow("w1",cv2.WINDOW_NORMAL)

"""								###########
for i in range(100):
	grid[i][0] = 1
	grid[i][1] = 1
	grid[i][2] = 1

	grid[i][97] = 1
	grid[i][98] = 1
	grid[i][99] = 1

	grid[0][i] = 1
	grid[1][i] = 1
	grid[2][i] = 1

	grid[97][i] = 1
	grid[98][i] = 1
	grid[99][i] = 1
"""
grid_2 = grid.copy()

# def getDistAndAngle(data):
# 	loc = []
# 	for idx in range(35,630):
# 		loc.append([data[int(idx)],idx*240/667-30])
# 	return loc
global x_1,y_1	
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
	
# for i in range(0,100):
# 	for j in range(0,100):
# 		if(i%10==0 or j%10==0):
# 			grid[i][j]=0
			
cv2.namedWindow("w1",cv2.WINDOW_NORMAL)

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
						return self.correctPath(list(reversed(pathList)))				############
			if len(self.openList) == 0:
				return None
###############################################
   
timestep = int(robot.getBasicTimeStep())
motor_left = robot.getDevice('wheel_left_joint')
motor_right = robot.getDevice('wheel_right_joint')
motor_left.setPosition(inf)    
motor_right.setPosition(inf)   
motor_left.setVelocity(0)                              
motor_right.setVelocity(0)                               
gps = robot.getDevice('gps')
gps.enable(timestep)
IU = robot.getDevice('inertial unit')
IU.enable(timestep)
n_states = 3
n_controls = 2

ld = robot.getDevice('Hokuyo URG-04LX-UG01')                                
ld.enable(timestep)
															
ld.enablePointCloud()                                                       
ld.getHorizontalResolution()                                                


N = 9#12         
#delta_T = 0.2                                                                                          
n_pts_picked  = 10#10#15                                                                                                
d_reastar = 0.1#0.8                                                                                                 
wall_extra_width = 3                                            																
a_star_correction_search_window = 0													
waypoint_change_dist = 0.6#0.75078930665				#################											
U_ref = np.array([0.4,0], dtype ='f')#0.85								   
error_allowed = 2e-1                                                                                                                                                                    

Q_x = 10000000000000#100000                                                ############### changed tuning
Q_y = 10000000000000#100000
Q_theta = 5#250                                                                                                                                              
Q_x_2 = 0                                                                               ###################
Q_y_2 =  0                                                                              ##################
Q_theta_2 = 5000000000#5000                                                                                    #################
R1 = 0.00001    
R2 = 4000000000*1.5#250       

n_stopped = 0                                                                                                       # HARD CODED UTURN
u_turn_flag = 0                                                                                                   # HARD CODED UTURN
zero_V_flag = 0														 # HARD CODED UTURN

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
v_max =0.4#1#omega_wheel_max*wheel_rad                                 
v_min = 0#0.0001#-v_max    
omega_max = omega_wheel_max*wheel_rad/robot_dia                                                
omega_min = -omega_max

global x,y,theta,V,omega, theta_2

path=np.zeros((n_pts_picked,2))                                               

for i in range(0,100):										#
	for j in range(0,100):	#									
		if(i%10==0 or j%10==0):#
			gridlines[i][j]=1#



while (robot.step(timestep) != -1):     
	x = gps.getValues()[0]
	y = gps.getValues()[1]
	theta =IU.getRollPitchYaw()[0]  + pi/2  ## Commented
	if theta > pi:
		theta = theta - 2*pi
	theta_2 = theta
	#theta =IU.getRollPitchYaw()[2]  ## Changed to 2
	#theta_2=IU.getRollPitchYaw()[2]




	
	#############################################################
	lidar_values = ld.getRangeImageArray()  
		###### 
	bot_x = x
	bot_y = y
	bot_theta = theta_2                                                       
	for idx in range(35,635, 3):							
		if lidar_values[int(idx)][0] <= 3.0:
			# print('HERE')
			dist, angle = lidar_values[int(idx)], idx*240/666-30
			dist = dist[0]
			if(dist==np.inf):
				dist= -1000

			coordRel = getPointRel(angle,dist)
			x,y = transform(coordRel[0],coordRel[1],bot_theta,bot_x,bot_y)
			x_int= int((x+5)*10+0.5)
			y_int = int((y+5)*10+0.5)
			# X.append(x_int)
			# Y.append(y_int)
			x_temp = x_int
			y_temp=y_int
			if(x_int<100 and y_int<100 and x_int>=0 and y_int>=0):
				grid[x_int][y_int]=1
	dst = grid & gridlines      

	dst_2 = np.copy(dst)
	for i in range(1, 99):
		for j in range(1, 99):
			middle_sum = dst[i][j] + dst[i-1][j] + dst[i+1][j] + dst[i][j-1] + dst[i][j+1] #+ dst[i-2][j] + dst[i+2][j] + dst[i][j-2] + dst[i][j+2]
			corner_sum = dst[i-1][j-1] + dst[i+1][j+1] + dst[i-1][j+1] + dst[i+1][j-1] #+ dst[i-2][j-2] + dst[i+2][j+2] + dst[i-2][j+2] + dst[i+2][j-2]
			
			if not(middle_sum >= 2 and corner_sum == 0):
				dst_2[i][j] = 0

	grid = np.copy(dst_2)													
	dst_3 = np.copy(dst_2)

	for i in range(1, 99):
		for j in range(1, 99):
			if(dst_2[i][j] == 1):
				for k in range(-1 * wall_extra_width, wall_extra_width + 1):
					for l in range(-1 * wall_extra_width, wall_extra_width + 1):
						if (i + k < 100 and i + k >= 0 and j + l < 100 and j + l >=0):
							dst_3[i+k][j+l] = 1
	
	# dst = dst_2

	for i in range(100):
		for j in range(4):
			dst_3[i][j] = 1
			dst_3[i][100 - j - 1] = 1
			dst_3[j][i] = 1
			dst_3[100 - j - 1][i] = 1
		
		dst_3[4][i] = 1
		#############

	# cv2.imshow('w1',(dst_3*255).astype('uint8'))                  #
	# cv2.waitKey(1)

	size=dst_3.shape#grid.shape						#
	map2d = Array2D(size[0], size[1])
	for i in range(size[0]):
		for j in range(size[1]):
			map2d[i][j]=dst_3[j][i]#grid[j][i]			#
	   
		
	
	x1=(bot_x+5)*10 + 0.5
	y1=(bot_y+5)*10 + 0.5

	if len(all_waypoints) > 1:
		if (np.sqrt(abs(bot_x - waypoint[0]) ** 2 + abs(bot_y - waypoint[1]) ** 2) <= waypoint_change_dist):
			for i__ in range(len(all_waypoints)-1):                                                                                                         #####
				if (np.sqrt(abs(bot_x - all_waypoints[i__][0]) ** 2 + abs(bot_y - all_waypoints[i__][1]) ** 2) <= waypoint_change_dist):
					all_waypoints.pop(i__)
					i__ -= 1
					break

			if len(all_waypoints) > 1 :                                                                                                                     ###########    
				current_waypoint_index = KDTree(((np.array(all_waypoints[:-1]) + 5) * 10)).query(np.array([x1, y1]))[1]                                     #########
				waypoint = all_waypoints[current_waypoint_index]

			else:                                                                                                                                               ##########
				waypoint = all_waypoints[0] 	                                                                                                                #########
	
	else:
		if (np.sqrt(abs(bot_x - waypoint[0]) ** 2 + abs(bot_y - waypoint[1]) ** 2) <= 1e-1):

			for i__ in range(len(all_waypoints)):
				if abs(pStart.x - int((all_waypoints[i__][1] + 5) * 10 + 0.5)) <= waypoint_change_dist and abs(pStart.y - int((all_waypoints[i__][0] + 5) * 10 + 0.5)) <= waypoint_change_dist:
					all_waypoints.pop(i__)
					i__ -= 1
					break

			if len(all_waypoints):
				current_waypoint_index = KDTree(((np.array(all_waypoints) + 5) * 10)).query(np.array([x1, y1]))[1]
				waypoint = all_waypoints[current_waypoint_index]

	## check pstart in obstacle
	if dst_3[int(x1)][int(y1)] == 1:#grid[int(x1)][int(y1)] == 1:									#
		for j in range(-1 * a_star_correction_search_window, a_star_correction_search_window + 1):
			flag = 0
			for k in range(-1 * a_star_correction_search_window, a_star_correction_search_window + 1):
				if dst_3[int(x1) + j][int(y1) + k] == 0:#grid[int(x1) + j][int(y1) + k] == 0:					#
					x1 = int(x1) + j
					y1 = int(y1) + k
					flag = 1
					break
			if flag:
				break
	
	# print(np.array([x1, y1]))
	# print(KDTree((np.array(all_waypoints) + 5) * 10).query(np.array([x1, y1])))
	if KDTree(((np.array(all_waypoints[:-1]) + 5) * 10)).query(np.array([x1, y1]))[1] != current_waypoint_index:                            ###############										
		current_waypoint_index = KDTree(((np.array(all_waypoints[:-1]) + 5) * 10)).query(np.array([x1, y1]))[1]                             ##############
		waypoint = all_waypoints[current_waypoint_index]
		print(waypoint)


	pStart, pEnd = Point(int(y1),int(x1)), Point(int((waypoint[1]+5)*10 + 0.5),int((waypoint[0]+5)*10 + 0.5))
	aStar = AStar(map2d, pStart, pEnd)
	aStar.expansion(offset=0)
	pathList = aStar.start()
	#path=np.zeros((30,2), dtype ='f')	
	if pathList:
		path[0][0] = int(x1)                                                    
		path[0][1] = int(y1)                                                    
		i=1#0                                                                  
		for point in pathList:
			if i==n_pts_picked:
				break
			path[i][0]= float(point.y)
			path[i][1]= float(point.x)
			dst_4[int(point.y)][int(point.x)] = 0.75
			i=i+1
		
		if i<n_pts_picked:
			for j in range(i,n_pts_picked):
				path[j][0]=path[j-1][0]
				path[j][1]=path[j-1][1]

	path = np.divide(path,10)
	path = np.subtract(path,5)
	
	# print( path)

	############################################################
	
	break



global total_path_points												#  """??"""      
total_path_points = path.shape[0]                                                                      												
#path = np.zeros((95,2))	                                                                            										



path_resolution =  ca.norm_2(path[0,0:2] - path[1,0:2])         
global delta_T                                                  
delta_T = ( path_resolution / ((U_ref[0])) )/10                            

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
P = ca.SX.sym('P',1, n_states + n_states*(N) + n_controls*(N))
X =ca.SX.sym('X', n_states, N+1)
obj = 0
g = []
Q = ca.diagcat(Q_x, Q_y,Q_theta)    
																																									 
Q_2 = ca.diagcat(Q_x_2, Q_y_2,Q_theta_2)                                                                                                            #####################################                                                                                                                                                                        

R = ca.diagcat(R1, R2)                                                                                         
for i in range(0,N-5):                                                                                                                                #######################################                                                                                                                                                                                                                        
	cost_pred_st = ca.mtimes(  ca.mtimes( (X[0:n_states,i] - P[n_states*(i+1) :n_states*(i+1) + n_states ].reshape((n_states,1)) ).T , Q )  ,  (X[0:n_states,i] - P[n_states*(i+1) :n_states*(i+1) + n_states ].reshape((n_states,1)) )  )  + ca.mtimes(  ca.mtimes( ( (U[0:n_controls,i]) - P[n_states*(N+1)+n_controls*(i):n_states*(N+1)+n_controls*(i) + n_controls].reshape((n_controls,1)) ).T , R )  ,  U[0:n_controls,i] - P[n_states*(N+1)+n_controls*(i):n_states*(N+1)+n_controls*(i) + n_controls].reshape((n_controls,1))  )  
	obj = obj + cost_pred_st 
	
#for stopping at last point, we need higher cost (Q_2)                                                          #############################################
for i in range(N-5,N): 
	cost_pred_st = ca.mtimes(  ca.mtimes( (X[0:n_states,i] - P[n_states*(i+1) :n_states*(i+1) + n_states ].reshape((n_states,1)) ).T , Q_2 )  ,  (X[0:n_states,i] - P[n_states*(i+1) :n_states*(i+1) + n_states ].reshape((n_states,1)) )  )
	obj = obj + cost_pred_st 


for i in range(0,N+1):                                                                                                                                                                        
	if i == 0:
		g = ca.vertcat( g,( X[0:n_states,i] - P[0:n_states].reshape((n_states,1)) )  )                                                                                       
	else:
		K1 = f(X[0:n_states,i-1],U[0:n_controls,i-1])                                                                       
		K2 = f(X[0:n_states,i-1] + np.multiply(K1,delta_T/2),U[0:n_controls,i-1])                                          
		K3 = f(X[0:n_states,i-1] + np.multiply(K2,delta_T/2),U[0:n_controls,i-1])                                          
		K4 = f(X[0:n_states,i-1] + np.multiply(K3,delta_T),U[0:n_controls,i-1])                                            
		pred_st = X[0:n_states,i-1] + (delta_T/6)*(K1+2*K2+2*K3+K4)                                                          
			 
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
#X_target = np.array([ path[total_path_points-1][0], path[total_path_points-1][1], 0 ]  , dtype = 'f')                        #   """???"""  
path_last = np.array([ path[total_path_points-1][0], path[total_path_points-1][1]]  , dtype = 'f')                        

P = X_init                                                                                       

#close_index = KDTree_func(path,x,y)#KDTree(path).query(P[0:n_states-1])[1]                                                                                         #################################                               
close_index = 0                                                                                                                                                      ######################################

for i in range(0,N):                                                                              
	P = ca.vertcat(P,path[close_index+i,0:2])                                                          
	P = ca.vertcat(P, math.atan((path[close_index+i+1][1] - path[close_index+i][1])/(path[close_index+i+1][0] - path[close_index+i][0]+ 1e-9)) )        

for i in range(0,N):                                                                              
	P = ca.vertcat(P, U_ref[0])                                                                   
	P = ca.vertcat(P, U_ref[1])                                                                  

initial_X = ca.DM.zeros((n_states*(N+1)))
initial_X[0:n_states*(N+1):3] = X_init[0]
initial_X[1:n_states*(N+1):3] = X_init[1]
initial_X[2:n_states*(N+1):3] = X_init[2]

initial_con = ca.DM.zeros((n_controls*N,1)) 


n_iter = 0
"""???"""  
pStart, pEnd = Point(0, 0), Point(0, 0)
#while ( ca.norm_2( P[0:n_states-1].reshape((n_states-1,1)) - X_target[0:n_states-1] ) > error_allowed and robot.step(timestep) != -1 ):    				                                       
counter = 0                                                                                                                                                            
while ( robot.step(timestep) != -1 ): 
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
	# print('X: ', x, ', Y:  ', y)                                                     
	theta =IU.getRollPitchYaw()[0] + pi/2              ## Commented                           
	if theta > pi :
		theta = theta - 2*pi
	theta_2 = theta
	#theta =IU.getRollPitchYaw()[2]   ## Changed to 2
	#theta_2 = IU.getRollPitchYaw()[2]
	P[0:n_states] =([x,y,theta]) 

	
		#############################################################
	lidar_values = ld.getRangeImageArray()                                                         
	
		######                                                        
	bot_x = x
	
	bot_y = y
	
	bot_theta = theta_2

	for idx in range(35,635, 3):							
		if lidar_values[int(idx)][0] <= 3.0:
			# print('HERE')
			dist, angle = lidar_values[int(idx)], idx*240/666-30
			dist = dist[0]
			if(dist==np.inf):
				dist= -1000

			coordRel = getPointRel(angle,dist)
			x,y = transform(coordRel[0],coordRel[1],bot_theta,bot_x,bot_y)
			x_int= int((x+5)*10+0.5)
			y_int = int((y+5)*10+0.5)
			# X.append(x_int)
			# Y.append(y_int)
			x_temp = x_int
			y_temp=y_int
			if(x_int<100 and y_int<100 and x_int>=0 and y_int>=0):
				grid[x_int][y_int]=1
	dst = grid & gridlines      

	dst_2 = np.copy(dst)
	for i in range(1, 99):
		for j in range(1, 99):
			middle_sum = dst[i][j] + dst[i-1][j] + dst[i+1][j] + dst[i][j-1] + dst[i][j+1] #+ dst[i-2][j] + dst[i+2][j] + dst[i][j-2] + dst[i][j+2]
			corner_sum = dst[i-1][j-1] + dst[i+1][j+1] + dst[i-1][j+1] + dst[i+1][j-1] #+ dst[i-2][j-2] + dst[i+2][j+2] + dst[i-2][j+2] + dst[i+2][j-2]
			
			if not(middle_sum >= 2 and corner_sum == 0):
				dst_2[i][j] = 0

	grid = np.copy(dst_2)
	dst_3 = np.copy(dst_2)

	for i in range(1, 99):
		for j in range(1, 99):
			if(dst_2[i][j] == 1):
				for k in range(-1 * wall_extra_width, wall_extra_width + 1):
					for l in range(-1 * wall_extra_width, wall_extra_width + 1):
						if (i + k < 100 and i + k >= 0 and j + l < 100 and j + l >=0):
							dst_3[i+k][j+l] = 1
	
	# dst = dst_2
	
	for i in range(100):
		for j in range(4):
			dst_3[i][j] = 1
			dst_3[i][100 - j - 1] = 1
			dst_3[j][i] = 1
			dst_3[100 - j - 1][i] = 1
		
		dst_3[4][i] = 1

	# dst_4 = np.copy(dst_3).astype("float")				#

	for i in range(100):
		for j in range(100):
			if (dst_3[i][j] == 1):
				dst_4[i][j] = 1
			else:
				if (dst_4[i][j] == 1):
					dst_4[i][j] = 0
	
	x_bot_grid=int((bot_x+5)*10 + 0.5)				
	y_bot_grid=int((bot_y+5)*10 + 0.5)				
	dst_4[x_bot_grid][y_bot_grid] = 0.5                     #

		#############

	# cv2.imwrite('img1.png',(dst_4*255).astype('uint8'))
	# cv2.waitKey(1)

	if (ca.norm_2( P[0:n_states-1].reshape((n_states-1,1)) - path_last[0:n_states-1] ) < d_reastar) or counter >= 1000:     
		#size=grid.shape
		counter = 0																		
		print('HERE')
		map2d = Array2D(size[0], size[1])
		for i in range(size[0]):
			for j in range(size[1]):
				map2d[i][j]=dst_3[j][i]     #
		   
			
		
		x1=(bot_x+5)*10 + 0.5                  
		y1=(bot_y+5)*10 + 0.5                   				

		## check pstart in obstacle
		if dst_3[int(x1)][int(y1)] == 1:    #
			for j in range(-1 * a_star_correction_search_window, a_star_correction_search_window + 1):
				flag = 0
				for k in range(-1 * a_star_correction_search_window, a_star_correction_search_window + 1):
					if dst_3[int(x1) + j][int(y1) + k] == 0:
						x1 = int(x1) + j
						y1 = int(y1) + k
						flag = 1
						break
				if flag:
					break			
		
		
		if  KDTree(((np.array(all_waypoints[:-1]) + 5) * 10)).query(np.array([x1, y1]))[1] != current_waypoint_index:                                   ######
			current_waypoint_index = current_waypoint_index = KDTree(((np.array(all_waypoints[:-1]) + 5) * 10)).query(np.array([x1, y1]))[1]                #######     
			waypoint = all_waypoints[current_waypoint_index]
		
		print('Current Waypoint: ', waypoint)

		pStart, pEnd = Point(int(y1),int(x1)), Point(int((waypoint[1]+5)*10 + 0.5),int((waypoint[0]+5)*10 + 0.5))

		# if (abs(pStart.x - pEnd.x) <= waypoint_change_dist and abs(pStart.y - pEnd.y) <= waypoint_change_dist):
		# 	if current_waypoint_index < len(all_waypoints) - 1:
		# 		current_waypoint_index += 1
		# 		waypoint = all_waypoints[current_waypoint_index]
		# 		pEnd = Point(int((waypoint[1]+5)*10 + 0.5),int((waypoint[0]+5)*10 + 0.5))

		aStar = AStar(map2d, pStart, pEnd)
		aStar.expansion(offset=0)
		pathList = aStar.start()
		#path=np.zeros((30,2), dtype ='f')	
		if pathList:
			path[0][0] = int(x1)                                                   
			path[0][1] = int(y1)                                                   
			i=1#0                                                                   			
			for point in pathList:
				if i==n_pts_picked:
					break
				path[i][0]= float(point.y)
				path[i][1]= float(point.x)
				dst_4[int(point.y)][int(point.x)] = 0.75   ####
				i=i+1
			print(i)
			# if i<n_pts_picked:
			# 	if i<=10 and current_waypoint_index < len(all_waypoints) - 1:
			# 		current_waypoint_index += 1
			# 		waypoint = all_waypoints[current_waypoint_index]
			# 		pStart_2, pEnd_2 = pEnd, Point(int((waypoint[1]+5)*10 + 0.5),int((waypoint[0]+5)*10 + 0.5))
			# 		aStar = AStar(map2d, pStart, pEnd)
			# 		aStar.expansion(offset=0)
			# 		pathList = aStar.start()
			# 		j=0
			# 		for j in range(i, min(n_pts_picked, len(pathList) + i - 1)):
			# 			path[j][0]=float(pathList[j-i].y)
			# 			path[j][1]=float(pathList[j-i].x)
					
			# 		if j < n_pts_picked:
			# 			for k in range(j, n_pts_picked):
			# 				path[k][0]=path[k-1][0]
			# 				path[k][1]=path[k-1][1]
			# 	else:
			# 		for j in range(i,n_pts_picked):
			# 			path[j][0]=path[j-1][0]
			# 			path[j][1]=path[j-1][1]
			if i < n_pts_picked:
				for j in range(i,n_pts_picked):
					path[j][0]=path[j-1][0]
					path[j][1]=path[j-1][1]

			path = np.divide(path,10)
			path = np.subtract(path,5)

			path_last =  path[total_path_points-1,0:2]                              
			#print( path_last ,"      ",waypoint
			
			# if ca.norm_2(path_last - waypoint) < 2.5:
			# 	print( "***************************"
			# 	d_reastar = error_allowed
			# else:
			# 	d_reastar = 0.8    

			



			print(path)
		else:
			print("NO PATH")


	else:
		counter += 1																
		############################################################

	x1=(bot_x+5)*10 + 0.5     
	y1=(bot_y+5)*10 + 0.5     
	pStart, pEnd = Point(int(y1),int(x1)), Point(int((waypoint[1]+5)*10 + 0.5),int((waypoint[0]+5)*10 + 0.5))
	# if (abs(pStart.x - pEnd.x) <= waypoint_change_dist and abs(pStart.y - pEnd.y) <= waypoint_change_dist):
	if len(all_waypoints) > 1:
		if (np.sqrt(abs(bot_x - waypoint[0]) ** 2 + abs(bot_y - waypoint[1]) ** 2) <= waypoint_change_dist):
			# if current_waypoint_index < len(all_waypoints) - 1:
			# 	current_waypoint_index += 1
			# 	waypoint = all_waypoints[current_waypoint_index]
			# 	pEnd = Point(int((waypoint[1]+5)*10 + 0.5),int((waypoint[0]+5)*10 + 0.5))

			for i__ in range(len(all_waypoints)-1):                                                                                             #########
				# print(i__)
				if (np.sqrt(abs(bot_x - all_waypoints[i__][0]) ** 2 + abs(bot_y - all_waypoints[i__][1]) ** 2) <= waypoint_change_dist):
					all_waypoints.pop(i__)
					i__ -= 1
					break

			if len(all_waypoints) >1:                                                                                                              ####### 
				current_waypoint_index = KDTree(((np.array(all_waypoints[:-1]) + 5) * 10)).query(np.array([x1, y1]))[1]           #####
				waypoint = all_waypoints[current_waypoint_index]
			else:
				waypoint = all_waypoints[-1]                                                                                                           ####	
	
	else:
		if (np.sqrt(abs(bot_x - waypoint[0]) ** 2 + abs(bot_y - waypoint[1]) ** 2) <= 0.25):
			# if current_waypoint_index < len(all_waypoints) - 1:
			# 	current_waypoint_index += 1
			# 	waypoint = all_waypoints[current_waypoint_index]
			# 	pEnd = Point(int((waypoint[1]+5)*10 + 0.5),int((waypoint[0]+5)*10 + 0.5))

			for i__ in range(len(all_waypoints)):
				# print(i__)
				if abs(pStart.x - int((all_waypoints[i__][1] + 5) * 10 + 0.5)) <= waypoint_change_dist and abs(pStart.y - int((all_waypoints[i__][0] + 5) * 10 + 0.5)) <= waypoint_change_dist:
					all_waypoints.pop(i__)
					i__ -= 1
					break

			if len(all_waypoints):
				current_waypoint_index = KDTree(((np.array(all_waypoints) + 5) * 10)).query(np.array([x1, y1]))[1]                                          ###########
				waypoint = all_waypoints[current_waypoint_index]


	cv2.imshow('w1',(dst_4*255).astype('uint8'))                            #
	cv2.waitKey(1)


	if(len(all_waypoints) == 0):
		print('Breaking from while loop')
		break

																																### for dynamic path, we need to make sure that the path doesnot get changed inside the horizon length bcz we shifting points in P,not updating everytime from path
	#print (x,"   ",y)	
	# print(close_index)	
	#if KDTree_func(path,x,y)  != close_index:                                               
	close_index = KDTree_func(path,x,y)#KDTree(path).query(P[0:n_states-1])[1] 
	# print (close_index)
										

	if N+(close_index) < total_path_points : #and N+(close_index-1) < total_U_points                                    
		
		P[n_states:n_states*(N)] = P[n_states*2:n_states*(N+1)]                                                                                                                                                                                                               
		P[n_states*(N):n_states*(N+1)-1] = path[N+(close_index-1),0:2]                                             
		P[n_states*(N+1)-1] = math.atan( (path[N+(close_index-1)][1] - path[N+(close_index-1)-1][1])/(path[N+(close_index-1)][0] - path[N+(close_index-1)-1][0] + 1e-9) )           ###########

		#P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]                                                                                                                                                                                                              
		#P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = U_ref                                                                                     
		"""
		P[n_states:n_states*(N+1):3] = path[close_index:N+close_index,0]                    
		P[n_states+1:n_states*(N+1):3] = path[close_index:N+close_index,1]                  

		for i in range(0,N):                                                                
			P[n_states*(i+1+1)-1] = math.atan( (path[i+close_index+1][1] - path[i+close_index][1])/(path[i+close_index+1][0] - path[i+close_index][0] + 1e-9) )           
		  
		P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]  
		P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = U_ref                                                                 			"""								
		
	
	else:
		#print( "ELSEEEEEEEEE  ", close_index,"   ", P[n_states:n_states*2])
		P[n_states:n_states*(N)] = P[n_states*2:n_states*(N+1)]                                                                     
		P[n_states*(N):n_states*(N+1)-1] = path[(total_path_points-1),0:2]                                                                                                                                                                                                                      
		P[n_states*(N+1)-1] = math.atan( (path[total_path_points-1][1] - path[total_path_points-1-1][1])/(path[total_path_points-1][0] - path[total_path_points-1-1][0]+ 1e-9) )    
		
		#P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]                                                                                                                                                                                                                  
		#P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = U_ref                                                                                      



	# print( "Odometry = " , P[0:n_states]
	print( V, "    ", omega )

	#####################################  # HARD CODED UTURN
	if zero_V_flag == 1:
		motor_left.setVelocity(0)             
		motor_right.setVelocity(0)            
		zero_V_flag = 0

	if V < 0.05 and omega < 0.001:
		n_stopped += 1
		print (n_stopped)

	if V > 0.05 :
		n_stopped = 0
		u_turn_flag = 0
	
 

	if n_stopped > 50 and u_turn_flag == 0  :
		print ("BACKING UP")
		motor_left.setVelocity(-10)             
		motor_right.setVelocity(-10) 

		if omega > 0.005 and n_stopped > 121:
			u_turn_flag = 1
			n_stopped = 0 
			zero_V_flag = 1
	######################################

	for i in range(0,N*n_bound_var):                          
		initial_X[i] = X_U_sol[i+n_bound_var]                 
	for i in range(0,(N-1)*n_controls):                     
		initial_con[i] = X_U_sol[n_bound_var*(N+1)+i+n_controls]   
						 
#############################################################################

t_end = time.time()
print( "Total Time taken = " , t_end - t_start)
motor_left.setVelocity(0)             
motor_right.setVelocity(0)            
















