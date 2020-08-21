# triangulation

This ROS node is meant to be used with any autonmous car system to locate objects in 3D world coordinates from 2D rosbag images. It's purpose is for adding missing map elements, building maps specific objects in the car's environment, etc. 

The required elements for using this code package are:
1. A camera feed or rosbag that includes the object(s) of interest
2. Published Odometry messages
3. A published transform tree including:
	a. The camera frame 
	b. The odometry frame
	c. Any necessary frames to transform between the previous two frames

The overview of the use of this node is as follows:
1. A rosbag is chosen or recorded that contains the object(s) of interest. A diverse range of perspectives to take a large number of samples should be taken (30 - 50 images). This diversity will depend on the direction of appraoch to the object and the speed of approach. 
2. The ROS node is used with the rosbag to collect images and store them in a folder. The images are collected at a regular frequency every second. The ROS node will also record the image location, transformation matrices, and car location in the odometry frame in a CSV file. 
3. The triangulation python script takes in the information collected from the ROS node and bounding boxes must be manually drawn onto the sample images. 
4. The script will automatically calculate the 3D world coordinate location of the object of interest. 

Getting Started:

ROS node:
Once you have collected or choosen the rosbag you will be using for this task, you need to make some changes to the launch file:
1. 

  
