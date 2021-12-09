# Ch4_DifferentialKinematics
## Problem Statement
Approximate the differential kinematics of the Niryo_One robotic manipulator in a specific orientation using the geometric Jacobian. Create a simulation of the Niryo_One manipulator with the same input target angular velocities of its revolute links. Use Python to get the differential kinematics of the end effector and compare them to the approximation made previously. 
## Pre-Requisites
- Software: Coppeliasim (for simulating the pre-made Niryo_One robotic manipulator), Spyder (for Python coding)
- Mathematica/MATLAB recommended (for calculating the geometric Jacobian)
- Basic coding capabilities
- Package commands in Python: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm
## Solution to Approximation

![image](https://user-images.githubusercontent.com/95330513/145285885-e8007800-17c9-4e74-ae6c-3a723d8de2ab.png)

Shown above is a simplified representation of the Niryo_One manipulator where only the world frame (frame 0), the end-effector frame (frame 7), and the frames of joints 2, 3, and 4 from Coppeliasim are shown. These joints were chosen to simplify the example and calcuations, so that the results could be easily compared and simpler to troubleshoot. Note that all joints in the simulation and in the model are revolute joints. Therefore, the geometric Jacobian entry for each joint is found as follows:

  ![image169](https://user-images.githubusercontent.com/95330513/145286636-e9938530-f0c1-4f3f-ae65-17d74f402aa5.png)      
  
  ![image157](https://user-images.githubusercontent.com/95330513/145286529-5d3e6ec4-e7fe-4384-b53d-754552eca6a9.png)

While the orientation of the manipulator joints is approximated using the diagram above, the position of each joint with respect to the world frame (![image](https://user-images.githubusercontent.com/95330513/145287126-0ee31e31-d637-47d0-8e4e-658379b1202b.png)
) is found using the simulation. The ouput of these positions have the handles "Joint_Position_#" respectively in the Python coding. The position of the end-effector with respect to the world frame is also needed and its handle in the Python code is "EndEffector_Position"

These positions are needed directly from the code because the orientation of the world frame in Coppeliasim may not be aligned with the world frame of the robotic manipulator. Also, the output of the velocities found using the simulation will be with respect to the world frame of Coppeliasim. The following position vectors represent each of the positions:

![image](https://user-images.githubusercontent.com/95330513/145290590-1a74bcb3-f9e9-4102-948c-6497526ec635.png)

Once the positions are known, ![image](https://user-images.githubusercontent.com/95330513/145290458-a6199ff8-583d-40b3-8322-887f1e01b605.png)
can be calculated by simply subtracting the corresponding joint positions from the end effector's position. This gives the following results:

![image](https://user-images.githubusercontent.com/95330513/145290655-6f98bbec-5c75-45fe-a710-c824b8ed4d13.png)

Next, we can use the diagram to get an opproximation of the final orientation of the manipulator. Alternatively, one could use the Python code to get the orienations as well, doing so will lead to even more accurate results. Finding the orientation of each joint with respect to the world frame is fairly simple from the diagram as all of the links are either in line with each other or in a perpendicular orientation. For finding the geometric Jacobian of this simple model, we only need the z-orientation of each joint with respect to the world frame. In this diagram, those vecotors represent the orienation of the current z-axes with respect to the world frame

- For joint #2, the current z-axis is aligned with the world frame's positive x-axis, therefore: ![image](https://user-images.githubusercontent.com/95330513/145291633-c9b58426-837b-4fe6-a41b-07b1f223d9a0.png)

- For joint #3, the current z-axis is aligned with the world frame's positive x-axis, therefore: ![image](https://user-images.githubusercontent.com/95330513/145291737-1d67426e-27ca-484b-892c-4873a094ef04.png)

- For joint #4, the current z-axis is aligned with the world frame's positive y-axis, therefore: ![image](https://user-images.githubusercontent.com/95330513/145291822-3bce1a13-fc1d-4780-86ea-edf521739311.png)

Note that these values are equivalent to the bottom 3 values of each column of the final geometric Jacobian matrix. Therefore, the last step to finding the inputs of the rest of the Jacobian is to take the cross product: ![image169](https://user-images.githubusercontent.com/95330513/145292103-231dfd74-501b-40c4-a63a-9bfd1d5d907b.png) The results are as follows:

![image](https://user-images.githubusercontent.com/95330513/145292836-e4853b69-0ec4-4620-8b9b-8db0e71f051f.png)

Finally, we can combine all of the values to get the final geometric Jacobian:

![image](https://user-images.githubusercontent.com/95330513/145293575-ea59948a-deef-4cd3-b517-d2abf4e598c3.png)

Finally, to find the differential kinematics, we can multiply the Jacobian by the input velocities for each of the joints in the simulation. The input angular velocties of the joints in the Python coding are 0.04 rad/s for joints 2 and 3, and 0.05 rad/s for link joint 4. Multiplying by the approximated geometric Jacobian gives the following as our final results for the approximation:

![image](https://user-images.githubusercontent.com/95330513/145294523-a807f719-b586-40c7-a72f-a22daf311696.png)

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

- Several lines are dedicated to importing packages for use in the code which will allow commands to be sent to Coppeliasim. (Lines 17 and 18)
- Next, the code connects to the remote API server for Coppeliasim using the client ID from the code added to the simulation's script file (Lines 21-25). The IPython console will output the text "program started" and "Connected to remote API server" if Python has successfully connected to Coppeliasim. 
- The code then assigns variable names to the handles from Coppeliasim's scene hierarchy. This allows the rest of the code to reference the Coppeliasim components using those defined variable names. This is done for all of the joints of the manipulator as well as the EndEffector dummy that was added. (Lines 27-36)
- Now that these components can be referenced throughout the code, we can assign a velocity to them using the "sim.simxSetJointTargetVelocity" function. This function sets the joints target angular velocity. Note that because the manipulator is dynamically modeled, it may not reach the exact velocities which were input. Also, note that in the code, the first target velocities of the joints are set to -0.04, -0.04, and -0.05 respectively. (Lines 40-42) 
- Next, the code pauses for 1 second while the manipulator is moving. The velocities of the joints are then set back to 0. 
- The code pauses for another 0.5 sec, and then the velcities are set to 0.04, 0.04, and 0.05 respectively for 0.87 seconds. This is don so that the robot will move approximately back to its starting position with apporximately the same initial orientation as the diagram used to solve for the geometric Jacobian previously. 
- After 0.87 sec, the code finds the position of joints 2, 3, and 4, and the end-effector to be used in the Jacobian calculation. 
- The code also finds the absolute linear and angular velocities of the end-effector and outputs them as "linearV" and "angularV" respectively. This is the results we will compare to the hand-derived differential kinematics previously calculated. 
- Finally, the code sets all of the joints' velocities back to 0, ending the simulation. 

## Results

When ran, the simulation outputs the following vector of velocities:

![image](https://user-images.githubusercontent.com/95330513/145311751-3da8a687-9c44-44c3-bd5d-6bf4106e00da.png)

Comparing to the approximated velocities we have the following error for each:

![image](https://user-images.githubusercontent.com/95330513/145312114-3fa9fc29-0a62-431e-9a28-f72659d382fa.png)

Note that most of the results have errors between 3% and 15%. This is not terrible considering all of the factors that were not considered while deriving the results by hand. The dynamics of the model are not taken into consideration so each joint in the model is not moving at the exact input velocity of the joints. Also, as can be clearly viewed in the simulation, the manipulator does not go back to the exact position it started in, therefore there is error in the approximated ![image149](https://user-images.githubusercontent.com/95330513/145312429-3cc69739-ea56-453b-84b0-cbf3f7db9437.png). 

This is why using a simulation such as Coppeliasim is very useful when modelling robotic manipulators. Their results will most likely relate more closely to what an actual robot would do, how it would move, and how fast it sould go based on input torques, orienations, etc. Now, we could get better derived results if we had used more information from the simulation such as the orientation of the joints at the time of measuring the velocity, or finding the exact velocity of each of the joints at that time as well. However, without a program designed to understand the dynamics of the robot, we have to make some approximations as was done in this example. 







