# Triangulation

This ROS node is meant to be used with any autonmous car system to locate objects in 3D world coordinates from 2D rosbag images. It's purpose is for adding missing map elements, building maps specific objects in the car's environment, etc. 

The required elements for using this code package are:
1. A camera feed or rosbag that includes the object(s) of interest
2. Published Odometry messages
3. A published transform tree including:
   -The camera frame 
   -The odometry frame
   -Any necessary frames to transform between the previous two frames
   -Geometry, sensor, and nav messages, message filters, and Eigen libraries installed 

The overview of the use of this node is as follows:
1. A rosbag is chosen or recorded that contains the object(s) of interest. A diverse range of perspectives to take a large number of samples should be taken (30 - 50 images). This diversity will depend on the direction of appraoch to the object and the speed of approach. 
2. The ROS node is used with the rosbag to collect images and store them in a folder. The images are collected at a regular frequency every second. The ROS node will also record the image location, transformation matrices, and car location in the odometry frame in a CSV file. 
3. The triangulation python script takes in the information collected from the ROS node and bounding boxes must be manually drawn onto the sample images. 
4. The script will automatically calculate the 3D world coordinate location of the object of interest. 

## Getting Started:

### ROS node:
Once you have collected or choosen the rosbag you will be using for this task, you have two choices: 
1. Use this node as a standalone node that launches from it's own launch file 
2. Use this integrated into an existing package in your repository 

This README will give instructions on the first option but will also go over changes that need to be made for all uses of this node.

### Working Directory:
To contain all your work, make a directory to contain the python script, a folder to store the images collected by the ROS node, and the CSV file produced by the node. 
In your home directory:
```
mkdir [directory name] ex. "triangulation"
cd [directory name]
mkdir [name of folder to store images] ex. "images" 
```

### Launch File:
Open the launch file, "triangulation.launch" and make the following changes to the parameters:
1. Change the value of "camera frame" to the name of the camera frame in your repository 
2. Change the value of "camera topic" and "odom" to the name of the camera feed topic and odometry topic in your repository 
3. Change the "image rate" to the frequency of image collection you desire
4. Change "triang file"  to the path to the location and name you want to give to the csv file that is created
5. Change "folder" to the path to the location for soring the produced images created above
6. Change the yaml file under the command load to a configuration file name that contains the camera that you are using's intrinsics matrix. This is a set matrix that will not change as the camera moves and needs to be computed using calibration techniques. 








  
