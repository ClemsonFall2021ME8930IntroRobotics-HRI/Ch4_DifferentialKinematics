# Ch4_DifferentialKinematics
## Problem Statement
Approximate the differential kinematics of the Niryo_One robotic manipulator using the python coding to derive the geometric Jacobian and velocity of the end-effector. Create a simulation of the Niryo_One manipulator with the same input target angular velocities of its revolute links. Use Python to get the differential kinematics of the end effector and compare them to the derived approximation. 
## Pre-Requisites
- Software: Coppeliasim (for simulating the pre-made Niryo_One robotic manipulator), Spyder (for Python coding)
- Mathematica/MATLAB recommended (for calculating the geometric Jacobian)
- Basic coding capabilities
- Package commands in Python: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm
## Solution to Approximation

Note that the code attached in the gethub will do these calculations for us using the manipulator's current joint and end-effector positions and orientations. 

In the simultaion, only joints 2, 3, and 4 are given velocity values. The velolcities of joints 1, 5, and 6, are kept at zero, and therefore, they can be considered rigid. These joints were chosen to be rigid in order to simplify the example and calcuations, and so that the results could be easily compared and simpler to troubleshoot. Note that all joints in the simulation are revolute joints. Therefore, the geometric Jacobian entry for each joint is found as follows:

  ![image169](https://user-images.githubusercontent.com/95330513/145286636-e9938530-f0c1-4f3f-ae65-17d74f402aa5.png)      
  
  ![image157](https://user-images.githubusercontent.com/95330513/145286529-5d3e6ec4-e7fe-4384-b53d-754552eca6a9.png)

The position and orientation of each joint with respect to the world frame is found using the simulation and python code. The ouput of the positions and orientations have the handles "Joint_Position_#" and "Joint_orientation_#" respectively in the Python coding. The position of the end-effector with respect to the world frame is also needed and its handle in the Python code is "EndEffector_Position"

These positions values are needed directly from the code because the orientation and position of the world frame in Coppeliasim may not be aligned with the world frame of the robotic manipulator itself. Also, the output of the velocities found using the simulation will be with respect to the world frame of Coppeliasim.

Once the positions are known, ![image](https://user-images.githubusercontent.com/95330513/145290458-a6199ff8-583d-40b3-8322-887f1e01b605.png)
can be calculated by simply subtracting the corresponding joint positions from the end effector's position. 

Next, we can use the orientations of the joint frames to get rotation matrices for them and thus get the z-orientation of each joint with respect to the world frame by multiplying the rotation matrix by the transpose of [0,0,1]. Note that the result represents the bottom half of each link's respective parts of the geometric Jacobian.

The last step to finding the inputs of the rest of the Jacobian (the top half) is to take the cross product of the z-orientation and the difference between the position of the end-effector and joints:

![image169](https://user-images.githubusercontent.com/95330513/145292103-231dfd74-501b-40c4-a63a-9bfd1d5d907b.png) 

Finally, to find the differential kinematics, we can multiply the Jacobian by the velocities for each of the joints in the simulation. These velocities are also found in the python coding and noted together as the vector "Velocity_Tot". This final calculation will give the results for the derived linear and angular velocities of the system or "Derived_V" in the code. 

## Simulation
### Scene Setup (Alternatively open and use attached "Diff_Kin_Scene2" file in Coppeliasim)
To set up Coppeliasim, first open the program and it will create a new scene to work in. Then you can choose the Niryo_One robotic manipulator from the list of non-mobile robots on the left-hand side. Drag the robot and place it in any location in the scene. 

Next, make sure all of the joints (labelled "NiryoOneJoint#") are not control loop enabled, and the motors are locked when the target velocity is 0. To do this, click the plus icon next to the "NiryoOne" object handle in the Scene hierarchy. This will drop down the list of the compontents which combine to form the entire robot manipulator. 

Next, double-click the joint icon of joint 1. This will pull up the Scene object pererties page. Click "Show dynamic properties dialog" and make sure the page has the following boxes marked/unmarked as shown in the screenshot below:

![image](https://user-images.githubusercontent.com/95330513/145306985-2deaf01e-e6f8-4ebe-8f76-93446c26c3ed.png)

Repeat this for all joints in the manipulators scene hierarchy. 

To simplify the scene a little bit, delete the Gripper by clicking on "NiryoLGripper" in the scene hierarchy and clicking the delete button on your keyboard. Do the same with "Cuboid" and "NiryoOne_connection" in the scene hierarchy. 

In place of the gripper, we will need to add a dummy object so that later on, in the Python code, we will be able to attain its velocities. To do this, click "Add" in the top left of the program, and select "Dummy". This will add a standard dummy object to the scene. To make sure it is located at the end-effector, first drag the handle "Dummy" and place it on top of the "NiryoOneLink6_Visible". Next, make sure dummy is highlighted in the scene hierarchy, and click the ![image](https://user-images.githubusercontent.com/95330513/145307673-644cc5c1-4bec-4a27-9ace-0ff0e352ac88.png) button. Go to the position tab and adjust the settings to match this image (make sure parent frame is selected):

![image](https://user-images.githubusercontent.com/95330513/145307801-ca3567b6-9a4f-4136-97dc-08a7488ab84c.png)

This places the dummy's frame directly on the last link of the manipulator which is where the end-effector would attach. Now to make the Dummy object more visible, one can change the size and color of it by double clicking its icon in the scene hierarchy and adjusting object size and dummy color. Finally, change the name of the dummy object by double-clicking the text "Dummy" in the scene hierarchy, typing in a new name, and then pressing enter. For the code on this page, the name will need to be changed to "EndEffector".  

Once these steps are done, we can add some code within Coppeliasim to hlep connect it to Spyder. First, click the script icon ![image](https://user-images.githubusercontent.com/95330513/145308469-8b4ef9aa-4c48-4456-89f8-fd5afbce9d94.png) on the left-hand side of the screen. Make sure any scripts within it are diabled. Next, right-click on "NiryoOneJoint1" in the scene hierarchy. select add, associated child script, non-threaded as shown:

![image](https://user-images.githubusercontent.com/95330513/145308760-ba5f9643-39f3-446e-88d3-7d7b9d30629f.png)

Double click on the script icon that is now next to "NiryoOneJoint1" in the scene hierarchy. Add this line to the code: simRemoteApi.start(19999)

![image](https://user-images.githubusercontent.com/95330513/145308904-c1b925a4-3ef6-45e2-9337-9bb9add52538.png)

Now that the above steps are completed, the simulation is ready to be ran with the github code on this page

## Explanation of Code Structure
The code "MySim.py" is split into several sections as follows:

- Several lines are dedicated to importing packages for use in the code which will allow commands to be sent to Coppeliasim. (Lines 17-27)
- Next, the code connects to the remote API server for Coppeliasim using the client ID from the code added to the simulation's script file (Lines 30-35). The IPython console will output the text "program started" and "Connected to remote API server" if Python has successfully connected to Coppeliasim. 
- The code then assigns variable names to the handles from Coppeliasim's scene hierarchy. This allows the rest of the code to reference the Coppeliasim components using those defined variable names. This is done for all of the joints of the manipulator as well as the EndEffector dummy that was added. (Lines 37-46)
- Now that these components can be referenced throughout the code, we can assign a velocity to them using the "sim.simxSetJointTargetVelocity" function. This function sets the joints target angular velocity. Note that because the manipulator is dynamically modeled, it may not reach the exact velocities which were input. Also, note that in the code, the target velocities of the joints are set to -0.04, -0.04, and -0.05 respectively. (Lines 48-51) 
- Next, the code pauses for 0.3 second while the manipulator is moving.
- To begin calculating the Jacobian, the code gets the joint and end-effector's positons ("Joint_Position_#") (lines 56-59) and the joints' rotation matrices ("Rig_#") (lines 61-72)
- The code gats the z-orientation for each of the joints by multiplying Rig_# by the transpose of [0,0,1] (lines 77-79) which will be the bottom half of the Jacobian ("Rig_#_z"), and then finds the top half of the Jacobian by taking the cross product of the z-orienations with the difference in position of the joints and end-effector (lines 82-84)
- The joint float parameter velocities are gathered and formed into one vector named "Velocity_Tot". (Lines 87-90)
- The code combines all values into a single Jacobian matrix and takes the transpose of it to make sure it can be multiplied by the velocities. Then it finds the "Derived_V" by conducting the matrix multiplication. (Lines 93-95)
- Line 98 gets Coppeliasim's value for the linear and angular velocity at that time
- Lines 101 to 106 stop the simulation by setting all velocities to zero. 

## Results

The attached excel file and video file shows the results of running the simulation for different amounts of time before calcuating and gathering the velocities. The angular velocities are very comparable, but the linear velocities are a bit off. They do however have similar slopes.

Note that most of the results for linear velocity are off by about 1mm/s. This is not terrible considering the dynamics of the model are not taken into consideration so each joint in the model is not moving at the exact input velocity of the joints. Also, there is a time gap between the Jacobian being calculated and the Coppeliasim getting the velocites with the single command. During this time, the simulator moves slightly meaning there are errors in position, orientation, and float parameter velocities. 

This is why using a simulation such as Coppeliasim is very useful when modelling robotic manipulators. Their results will most likely relate more closely to what an actual robot would do, how it would move, and how fast it sould go based on input torques, orienations, etc. However, if the program being used does not have such as command to get the velocities, it is possible to get fairly similar results with other information as was done in the derivation of the Jacobian in this code. 







