# Triangulation

This ROS node is meant to be used with any autonmous car system to locate objects in 3D world coordinates from 2D rosbag images. It's purpose is for adding missing map elements, building maps specific objects in the car's environment, etc.

The math for the triangulation algorithm can be found in "triangulation_math.pdf" in this repo. This document contains two approaches to triangulation, one iterative and one not. The second approach, the 3D rays approach is implemented in this repo.

The required elements for using this code package are:
1. A camera feed or rosbag that includes the object(s) of interest
2. Published Odometry messages
3. A published transform tree including:
   - The camera frame
   - The odometry frame
   - Any necessary frames to transform between the previous two frames
4. Geometry, sensor, and nav messages, message filters, and Eigen libraries installed

The overview of the use of this node is as follows:
1. A rosbag is chosen or recorded that contains the object(s) of interest. **A diverse range of perspectives to take a large number of samples should be taken (30 - 50 images)**. This diversity will depend on the direction and speed of approach to the object.
2. The ROS node is used with the rosbag to collect images and store them in a folder. The images are collected at a regular frequency every second. The ROS node will also record the image location, transformation matrices, and car location in the odometry frame in a CSV file.
3. The triangulation python script takes in the information collected from the ROS node and bounding boxes must be manually drawn onto the sample images.
4. The script will automatically calculate the 3D world coordinate location of the object of interest.

## Getting Started:

### ROS node:

Once you have collected or choosen the rosbag you will be using for this task, you have two choices:
1. Use this node as a standalone node that launches from it's own launch file
2. Use this integrated into an existing package in your repository

This README will give instructions on the first option but will also go over changes that need to be made for all uses of this node.

#### Working Directory:

To contain all your work, make a directory to contain the python script, a folder to store the images collected by the ROS node, and the CSV file produced by the node.
In your home directory:
```
mkdir [directory name] ex. "triangulation"
cd [directory name]
mkdir [name of folder to store images] ex. "images"
```
#### Launch File and Configuration file:

Open the launch file, "triangulation.launch" and make the following changes to the parameters:
1. Change the value of "camera frame" to the name of the camera frame in your repository
2. Change the value of "camera topic" and "odom" to the name of the camera feed topic and odometry topic in your repository
3. Change the "image rate" to the frequency of image collection you desire
4. Change "triang file"  to the path to the location and name you want to give to the csv file that is created
5. Change "folder" to the path to the location for soring the produced images created above
6. Change "collect_image" to 0 if you don't want to collect images and csv and just want to look at the camera feed from your rosbag. Otherwise, set to 1.
6. Change the yaml file under the command load to a configuration file name that contains the camera that you are using's intrinsics matrix. This is a set matrix that will not change as the camera moves and needs to be computed using calibration techniques.

#### Running the node:

Now that all the changes have been made to allow this node to work in your system, you can start running the code. Make sure roscore is running and you've sourced your setup.bash file (source devel/setup.bash). First, build the package using _catkin build_. Then follow the below code:

```
roslaunch triangulation triangulation.launch
```
Open a new terminal

```
rosbag play "path/to/your/rosbag" --clock -r 0.5 -s start_time -u interval
```
In the last command, the start time for collecting images should be determined beforehand by playing the rosbag with collect_image set to 0 in the launch file. Note the time relative to the start of your rosbag that you want to start collecting images and the interval from that point over which you want to continue collecting images. If the entire rosbag is used, omit the -s and -u from the command.

A much more manual way of getting the images you want would be to delete the unwanted images from the folder where you stored the images and the lines of the CSV that pertain to it. This is subject to more error but also allows you to manipulate the samples used for triangulation so this is a useful option for finely selecting the samples to use.

A reminder that it is strongly suggested to use at least two different perspectives of the object at 90 degrees to each other. A similar effect can be achieved by driving past the object or turning past the object as the dimension that is often lacking in information is its depth.

##### Working with multiple rosbags:

As is preferred, you want to get as much information through diverse perspectives of the object of interest. Dimensions with less diversity of information provided will be less accurate. Therefore, you may need to collect information from multiple rosbags or from two intervals in a rosbag. To do this, run the same code above using a different CSV file name. The same images folder can be used. These files will be combined in the python step of this process.

Now you can start processing the images for the triangulation algorithm.

### Python Script:

If you used multiple rosbags or intervals of a rosbag, first run the following command:
```
python triangulation.py --combine -1 <file1> -2 <file2> -o <outputfile>
```

This will append two CSV files into the output CSV that you choose. The output should be "Successfully concatenated csv files".

Now that all the data was been prepared, the sample images can be processed. This means labelling the images with object centres. The triangulation algorithm will automatically run after this. Run the following command:
```
python triangulation.py --box True --csv "your_csv_file.csv" --show False
```
This will run the draw_bb function and will output instructions on how to label the images. It will ask you to select two points on each image, the first being the top left corner of the object and the second being the bottom right corner of the object.
Once you've labelled all the images, you will see the 3D world coordinate location printed in the terminal. If you want to see the projection of this result on the samples images, run the same command with --show set to True and --box set to False
```
python triangulation.py --box False --csv "your_csv_file.csv" --show True  
```
Now you should have the location of your object of interest and be able to visualize how well the estimate matches to the labelled locations.   
