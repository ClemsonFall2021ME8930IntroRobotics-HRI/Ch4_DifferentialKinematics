# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

# then press play in Coppelia Sim, then run code to import sim and connect 
# Spyder to CoppeliaSim 



import sim
import math
import time
import numpy as np
from itertools import product, combinations
from sympy.vector import CoordSys3D, gradient
from sympy import tanh, diff, lambdify, symbols
from scipy.spatial.transform import Rotation as M
from scipy import integrate
from itertools import product, combinations
from matplotlib.path import Path


print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

# Getting Handles for Joints
errorCode, Joint1=sim.simxGetObjectHandle(clientID, 'NiryoOneJoint1', sim.simx_opmode_blocking)
errorCode, Joint2=sim.simxGetObjectHandle(clientID, 'NiryoOneJoint2', sim.simx_opmode_blocking)
errorCode, Joint3=sim.simxGetObjectHandle(clientID, 'NiryoOneJoint3', sim.simx_opmode_blocking)
errorCode, Joint4=sim.simxGetObjectHandle(clientID, 'NiryoOneJoint4', sim.simx_opmode_blocking)
errorCode, Joint5=sim.simxGetObjectHandle(clientID, 'NiryoOneJoint5', sim.simx_opmode_blocking)
errorCode, Joint6=sim.simxGetObjectHandle(clientID, 'NiryoOneJoint6', sim.simx_opmode_blocking)

# Getting Handle for Dummy Objects
errorCode, EndEffector=sim.simxGetObjectHandle(clientID, 'EndEffector', sim.simx_opmode_blocking)


# Giving joints an initial negative velocities
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint2,-0.04,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint3,-0.04,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint4,-0.05,sim.simx_opmode_streaming )

time.sleep(0.3)

# Finding positions of the various moving joints and end effector
errorCode, Joint_Position_2 = sim.simxGetObjectPosition(clientID, Joint2, -1, sim.simx_opmode_blocking)
errorCode, Joint_Position_3 = sim.simxGetObjectPosition(clientID, Joint3, -1, sim.simx_opmode_blocking)
errorCode, Joint_Position_4 = sim.simxGetObjectPosition(clientID, Joint4, -1, sim.simx_opmode_blocking)
errorCode, EndEffector_Position = sim.simxGetObjectPosition(clientID, EndEffector, -1, sim.simx_opmode_blocking)

# Finding rotation matrix of moving joints
errorCode, Joint_orientation_2 = sim.simxGetObjectOrientation(clientID, Joint2, -1, sim.simx_opmode_blocking)
r1 = M.from_euler('zyx', [Joint_orientation_2[2], Joint_orientation_2[1], Joint_orientation_2[0]], degrees=False)
Rig_2=r1.as_matrix()

errorCode, Joint_orientation_3 = sim.simxGetObjectOrientation(clientID, Joint3, -1, sim.simx_opmode_blocking)
r1 = M.from_euler('zyx', [Joint_orientation_3[2], Joint_orientation_3[1], Joint_orientation_3[0]], degrees=False)
Rig_3=r1.as_matrix()

errorCode, Joint_orientation_4 = sim.simxGetObjectOrientation(clientID, Joint4, -1, sim.simx_opmode_blocking)
r1 = M.from_euler('zyx', [Joint_orientation_4[2], Joint_orientation_4[1], Joint_orientation_4[0]], degrees=False)
Rig_4=r1.as_matrix()

# Using the rotation matrices to get the orientation of their respective z-axes with respect to the abolute frame.
# Note this is the geometric Jacobian for the angulkar velocity

Rig_2_z = np.dot(Rig_2, np.transpose([0,0,1]))
Rig_3_z = np.dot(Rig_3, np.transpose([0,0,1]))
Rig_4_z = np.dot(Rig_4, np.transpose([0,0,1]))

# Finding geometric Jacobian for linear velocity 
Jv_1 = np.cross(Rig_2_z, np.subtract(EndEffector_Position, Joint_Position_2) )
Jv_2 = np.cross(Rig_3_z, np.subtract(EndEffector_Position, Joint_Position_3) )
Jv_3 = np.cross(Rig_4_z, np.subtract(EndEffector_Position, Joint_Position_4) )

# Finding float velocity parameter for the links
errorCode, Joint_Velocity_2 = sim.simxGetObjectFloatParameter(clientID, Joint2, 2012, sim.simx_opmode_blocking)
errorCode, Joint_Velocity_3 = sim.simxGetObjectFloatParameter(clientID, Joint3, 2012, sim.simx_opmode_blocking)
errorCode, Joint_Velocity_4 = sim.simxGetObjectFloatParameter(clientID, Joint4, 2012, sim.simx_opmode_blocking)
Velocity_Tot = [Joint_Velocity_2,Joint_Velocity_3,Joint_Velocity_4]

# Finding absolute velocity of the end-effector using the Geometric Jacobian
Geometric_J = [[Jv_1[0],Jv_1[1],Jv_1[2],Rig_2[0,2],Rig_2[1,2],Rig_2[2,2]],[Jv_2[0],Jv_2[1],Jv_2[2],Rig_3[0,2],Rig_3[1,2],Rig_3[2,2]],[Jv_3[0],Jv_3[1],Jv_3[2],Rig_4[0,2],Rig_4[1,2],Rig_4[2,2]]]
Geometric_J = np.transpose(Geometric_J)
Derived_V = np.dot(Geometric_J, Velocity_Tot)

#Finding the linear and angular velocity of the end effector using built-in program
errorCode, linearV, angularV=sim.simxGetObjectVelocity(clientID, EndEffector, sim.simx_opmode_blocking)

#Setting velocity of joints back to 0 to stop the simulation
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint1,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint2,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint3,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint4,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint5,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint6,0.0,sim.simx_opmode_streaming )
