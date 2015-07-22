# tube_polishing
The tube_polishing repository holds the source code and other files related to Capstone Project of MS Robotics program. 

This project implements robotics automation solution to manipulate tubular object for surface process. The code is written in C++ for Fuerte version of ROS (Robot Operating System), and it utilizes various ROS packages like pcl_ros and arm_navigation to perceive tubular object, resting on the table in front of PR2 humanoid robot, and manipulate it to perform machining task.  

Directory structure follows 'rosbuild'. Simulation video can be found on YouTube at this address, https://youtu.be/zXvfQSpo0w8, and system block diagram in 'System Block Diagram' directory of this repository.

The process can be divided into three major parts.

1) Perception : Process starts with capturing point cloud by Kinect sensor of PR2 Robot and processing it to find tubular object and approximating geometric model of that object.  

2) Grasp Analysis : Grasp analysis is done on approximated model by generating number of sample grasps on the surface and testing them for required force and manipulability to perform machining task. 

3) Manipulation : After finding optimal grasp pair, lift the object using arms of the PR2 robot and manipulate it in front of machine; in this case grinder wheel. It further iterates manipulation as necessary, to process marked surface area.
