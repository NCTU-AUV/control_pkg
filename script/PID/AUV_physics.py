import numpy as np
from numpy import pi,sin,cos
import time
from numpy import linalg
import math
def eulerAnglesToRotationMatrix(theta) :
	
	
	R_x = np.array([[1,         0,                  0                   ],
					[0,         math.cos(theta[0]), math.sin(theta[0]) ],
					[0,         -math.sin(theta[0]), math.cos(theta[0])  ]
					])
	R_y = np.array([[math.cos(theta[1]),    0,      -math.sin(theta[1])  ],
					[0,                     1,      0                   ],
					[math.sin(theta[1]),   0,      math.cos(theta[1])  ]
					])
	R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
					[math.sin(theta[2]),    math.cos(theta[2]),     0],
					[0,                     0,                      1]
					])
	R = np.dot(R_z, np.dot( R_y, R_x ))
	return R 

class AUV():
	def __init__(self):
		#     there will be mass + damp + Coriolis +buoyancy
		# ===================================================== #
		#                        mass                           #
		# ===================================================== #
		self.mass_scaler = 32#kg
		self.mass = self.mass_scaler*np.eye(3)
		self.inertia = np.array([	[850.98,0,0],
									[0,935.11,0],
									[0,0,1257.2]
								])
		self.M_rb=np.append(np.append(self.mass,np.zeros((3,3)),axis=1),np.append(np.zeros((3,3)),self.inertia,axis=1),axis=0)

		# ===================================================== #
		#                        buoyancy                       #
		# ===================================================== #
		self.buoyancy =389#N
		self.buoyancy_center = np.array([0,0,-0.014])

		# ===================================================== #
		#                        Coriolis                       #
		# ===================================================== #
		
		# ===================================================== #
		#                        Drag                           #
		# ===================================================== #
		
		self.drag=np.eye(6)*np.array([[1500,1500,1500,0,0,0]]).T
		# ===================================================== #
		#                        motor                          #
		# ===================================================== #		
		# motor 1. [2x3] face and position
		self.motor = ["","","","","","","",""]
		#						Vector 					Position
		self.motor[0]=np.array([[0,0,1],				[0,-0.285,0]])
		self.motor[1]=np.array([[0,0,1.],				[-0.31,0,0]])
		self.motor[2]=np.array([[0,0,1],				[0,0.285,0]])
		self.motor[3]=np.array([[0,0,-1.],				[0.31,0,0]])
		self.motor[4]=np.array([[-2**0.5,-2**0.5, 0.],	[0.31,-0.285,0]])
		self.motor[5]=np.array([[-2**0.5,2**0.5, 0.],	[-0.31,-0.285,0]])
		self.motor[6]=np.array([[2**0.5,2**0.5, 0.],	[-0.31,0.285,0]])
		self.motor[7]=np.array([[-2**0.5,2**0.5, 0.],	[0.31,0.285,0]])
		self.Trust = np.empty(shape=(0,6))
		for i in self.motor:
			self.Trust = np.vstack((self.Trust,np.append(i[0],np.cross(i[0],i[1]))))
		self.Trust =self.Trust.T
		self.Trust_inv = linalg.pinv(self.Trust)

		# ===================================================== #
		#                        attitude                       #
		# ===================================================== #
		self.roll=0.
		self.pitch=0.
		self.yaw=0.
		#Thrust Mapping
	def Eular_update(self,roll,pitch,yaw):
		self.roll=roll
		self.pitch=pitch
		self.yaw=yaw
	def gravity(self):
		return 9.790
	def mass_effect(self,acc):
		return np.dot(self.M_rb,acc)
	def coriolis_effect(self,vel):
		mass = self.mass_scaler
		Coriolis_l = 2*np.cross(mass*vel[0][3:6],vel[0][0:3]*pi/180)
		print(Coriolis_l)
	def drag_effect(self,vel):
		return self.drag.dot(vel.T*vel.T)
	def buoyancy_effect(self):
		g=self.gravity()
		#                   
		return(np.array([	[(self.buoyancy-self.mass_scaler*g)*sin(self.pitch*pi/180.)],
							[-(self.buoyancy-self.mass_scaler*g)*cos(self.pitch*pi/180.)*sin(self.roll*pi/180.)],
							[-(self.buoyancy-self.mass_scaler*g)*cos(self.pitch*pi/180.)*cos(self.roll*pi/180.)],
							[self.buoyancy*cos(self.roll)*(self.buoyancy_center[2]*sin(self.roll*pi/180.)-self.buoyancy_center[1]*cos(self.pitch*pi/180.))],
							[self.buoyancy*(self.buoyancy_center[0]*cos(self.roll*pi/180)*cos(self.pitch*pi/180.)+self.buoyancy_center[2]*sin(self.pitch*pi/180.))],
							[-self.buoyancy*(self.buoyancy_center[0]*sin(self.roll*pi/180)*cos(self.pitch*pi/180.)+self.buoyancy_center[1]*sin(self.pitch*pi/180.))],
				]))

		'''
 		return (np.array([(self.buoyancy-self.mass*g)*sin(self.pitch*pi/180.),
 								-(self.buoyancy-self.mass*g)*sin(self.roll*pi/180.)*cos(self.pitch*pi/180)
 							])
 				)
 		'''
'''
Po=AUV()
np.set_printoptions(precision=1)
np.set_printoptions(linewidth=500)
print(Po.Trust)
print(Po.Trust_inv)
print(np.dot(Po.Trust_inv,Po.Trust))
#vel=np.array([[0.1,0,0,0,0,1]])
'''
#print(Po.buoyancy_effect())
#print(Po.M_rb)
#print(Po.mass_effect(np.array([[1,1,1,1,1,1]]).T))
#print(Po.coriolis_effect(vel))
#print(Po.drag_effect(vel))
