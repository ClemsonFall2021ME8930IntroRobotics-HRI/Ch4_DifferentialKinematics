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
import time


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

time.sleep(1)

errorCode= sim.simxSetJointTargetVelocity(clientID, Joint1,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint2,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint3,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint4,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint5,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint6,0.0,sim.simx_opmode_streaming )

time.sleep(0.5)
 #moving joints back to approximate initial position with positive velocities
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint2,0.04,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint3,0.04,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint4,0.05,sim.simx_opmode_streaming )

time.sleep(0.87)

#Finding positions of the various moving joints and end effector
errorCode, Joint_Position_2 = sim.simxGetObjectPosition(clientID, Joint2, -1, sim.simx_opmode_blocking)
errorCode, Joint_Position_3 = sim.simxGetObjectPosition(clientID, Joint3, -1, sim.simx_opmode_blocking)
errorCode, Joint_Position_4 = sim.simxGetObjectPosition(clientID, Joint4, -1, sim.simx_opmode_blocking)
errorCode, EndEffector_Position = sim.simxGetObjectPosition(clientID, EndEffector, -1, sim.simx_opmode_blocking)

#Finding the linear and angular velocity of the end effector
errorCode, linearV, angularV=sim.simxGetObjectVelocity(clientID, EndEffector, sim.simx_opmode_blocking)

#Setting velocity of joints back to 0
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint1,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint2,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint3,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint4,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint5,0.0,sim.simx_opmode_streaming )
errorCode= sim.simxSetJointTargetVelocity(clientID, Joint6,0.0,sim.simx_opmode_streaming )
