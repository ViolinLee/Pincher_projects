The main purpose of this chapter (project) is to integrate 3d camera with pincher arm through ros. Here I just use arbotix_arm github repository to perform pincher arm pick and place.   
## Preparations
### ROS Packages:    
1.[arbotix](http://wiki.ros.org/arbotix) -- ArbotiX Drivers   
2.[turtlebot_arm](http://wiki.ros.org/turtlebot_arm) -- The turtlebot arm meta package.    
3.[turtlebot](http://wiki.ros.org/turtlebot) -- The turtlebot meta package.  
Note that the Kinect driver should be installed the in advance so that we can use it to perform environment perception task.    
In this project, I use the [turtlebot/turtlebot_arm](https://github.com/turtlebot/turtlebot_arm) and turtlebot/turtlebot repository, and do some modification on resource code. The following tutorial assumes that you have installed the ROS firmware for the arbotix-m control board, and you have understood the basic operation of the turnlebot_arm package.

### Pick and Place   
#### Step 1. Package installation and Test   
Download arbotix, turtlebot and turtlebot_arm package from github, or directly download this three package from **Pincher\_projects/4\_turtlebot\_pincher\_arm/turtlebot\_****ws\_src** in this repository and then paste them to you **[catkin workspace name]/src** folder.   
 
Install freenect_launch package to produce rectified, registered or disparity images captured by Kinect V1 camera. Also produce point clouds and registered point clouds.   

    sudo apt-get install ros-kinetic-freenect-camera ros-kinetic-freenect-stack ros-kinetic-freenect-launch

Using the following command to launch Kinect v1.   

    roslaunch freenect_launch freenect.launch

Using the following command to view image from Kinect and select the topics.   

    rosrun rqt_image_view rqt_image_view


Or you can use image_view to view image. Firstly view the rgb image.      

    rosrun image_view image_view image:=/camera/rgb/image_color

Then view the depth image.   

    rosrun image_view image_view image:=/camera/depth_regisred/image

But here I suggest you use rviz to view the rgb and depth image.   

    rosrun rviz rviz

Click button of "add", select "camera" type, then choose **/camera/rgb/image\_color** or **/camera/depth\_registered/image** to view rgb image and depth image.    

You can also add PointCould2 and choose **/camera/depth\_registered/points** to view pointcloud. Since the point cloud is displayed in space, **"Fixed Frame"** must be set, and **Fixed Frame** must be a frame on the camera or a frame having a certain TF transformation with any frame on the camera.   

#### Step 2. Kinect V1 external parameter calibration   
The purpose of Kinect calibration is to obtain the spatial relationship between Kinect and pincher arm, specifically to obtain the frame transformation between Kinect and pincher arm. You can visit this [website](http://wiki.ros.org/turtlebot_kinect_arm_calibration/Tutorials/CalibratingKinectToTurtleBotArm) to learn more about the specific operation. Although this website uses the old version of launch file and the turnlebot arm, the principle and process are basically the same.       
First, prepare a checkerboard. You can use this one.      

using the following command to launch everything needed for calibration task.   

    roslaunch turtlebot_arm_kinect_calibration calibrate.launch

After the popped up window showed that the checkboard has been successfully detected and labled the four conner, you should move the edge of the gripper to the four specified points in order.   

After moving to the fourth point, the calibration script will output some infomation include frame transform between camera frame and fixed frame, and a **static transform publisher**. In this project, you're calibrating an external kinect. Open up a new terminal window, and run the static transform output by the script. As long as this command runs, the static_transform_publisher will publish the transform between a frame on the Pincher arm and the kinect frame. If you move the physical camera or pincher arm, you will need to recalibrate again. When the calibration process is correct, you should see the kinect pointcloud (/camera/depth\_registered/points) line up with the actual position of the arm in rviz.    

#### Step 3. Perform Block Pick-and-Place
Firstly, run the following command to launch pincher arm the main MoveIt executable to provide move groups.    

`roslaunch turtlebot_arm_block_manipulation block_manipulation_moveit.launch`  

Secondly, run the following command to launch Kinect V1, and provide an arbitrary link between pincher arm and 3D camera (You should modify object_manipulation_demo.launch by yourself to make sure that the transfomation between 3D camera and pincher arm are correct. The **static\_transform\_publisher** parameters can be obtained from Step 2). 

`roslaunch turtlebot_arm_block_manipulation block_manipulation_demo.launch`

In order to perform the pick and place tasks properly, you may need to modify other parameters, such as the height of the table, the size of your blocks, and so on.




 