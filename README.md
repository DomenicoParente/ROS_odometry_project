# ROS_project

Description
 ./bags: it contains the bags provided for the project
 ./custom_odometry: it contains the msg file for the custom message that includes odometry and integration method.
 ./nodes: it contains the various nodes for the calculation of the odometry.
	/src:
	> velocities_node: it calculates the linear and angular velocities of the robot and publishes them in a TwistStamped type message. It also provides two functions to calibrate the apparent baseline and the gear reduction. The two values are calculated doing an average of the valid values. These two functions are unuseful at runtime.
	> odom_node: it calculates the odometry and publish it in the odometry message and in the custom message. It publishes also on TF. The node also includes the services servers and the dynamic parameter server.
	> client_reset0_service: simple client to use the Reset0 service that set the position to (0,0,0).
	> client_resetpos_service: simpe client to use the ResetPos service that set the position to (x,y,theta).
	/launch:
	> robot.launch: it is the launch file for the first bag. It sets the parameter to initialize the	pose of the robot and executes the nodes.
	/cfg:
	> integration_param.cfg: cfg file to create the dynamic parameter to set the type of integration.
	/srv:
	> Reset0.srv: srv file to create the service that reset the position to (0,0,0)
	> ResetPos.srv: srv file to create the service that reset the position to (x,y,theta)
 ./robotics_hw1: it contains the custom message file provided for the project. 


ROS parameters
Initial position parameter:
	> /position/x : it sets the position on x-axis 
	> /position/y : it sets the position on y-axis
	> /position/theta : it sets the orientation of the robot

Dynamic integration parameter:
	> integration_param : if equal to 0 it setup the Euler integration
			      if equal to 1 it setup the Runge-Kutta integration
	The server of this parameter is in odom_node, so to set it we have to use the following command:
	> rosrun dynamic_reconfigure dynparam set odom_node 0  //Euler integration
	> rosrun dynamic_reconfigure dynparam set odom_node 1  //Runge-Kutta integration
	
	
Structure of the TF tree
	odom
	  |
	  |	broadcaster: /odom_node
	  V
	base_link
	
	
Start instructions
> First of all we need to compile the entire project trough:
	> catkin_make
	> echo "source ~/devel/setup.bash" >> ~/.bashrc
	> source ~/.bashrc
> Run the launch file in /src/nodes/launch through the command:
	> roslaunch nodes robot.launch
> Run the bag in /src/bags through the commands:
	> cd ./src/bags
	> rosbag play bag1.bag
	 

Additional instructions
The services provided in the project can be used through the commands:
	- rosservice call /reset0_service 1
	- rosservice call /resetpos_service 2.8113269 0.78471197 3.053794
or through the execution of the service clients nodes:
	- rosrun nodes client_reset0_service
	- rosrun nodes client_resetpos_service 2.2 4.5 6.6 
	

Additional information
In /src/nodes/launch are present also the launch files for bag2 and bag3.

