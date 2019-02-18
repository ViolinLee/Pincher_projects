The main purpose of this chapter (project) is to understand the kinematics characteristics of pincher arm, and be familiar with the modeling method of robot arm. Finally, using robot toolbox in MATLAB to control pincher arm.   
Reference resources: [petercorke](http://petercorke.com/wordpress/resources)


## Preparations:
### Step 1. Familiar with arm kinematics / DH parameter   
Read the following articles to gain an understanding of DH parameters:   
[DH parameters](http://petercorke.com/wordpress/?ddownload=545)   
   

### Step 2. Install Robotics Toolbox   
Recommended Installation Method: Download [RTB-10.3.1 mltbx format (23.2 MB)](http://petercorke.com/wordpress/?ddownload=574) in MATLAB toolbox format (.mltbx) . From within the MATLAB file browser double click on this file, it will install and configure the paths correctly.   
Click [Robotics Toolbox by Peter Corke](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) to know more about the Robotics Toolbox (RBT).    

### Step 3. Familiar with Robotics Toolbox
Run the demo `rtbdemo` to see what the toolbox can do. For example, when we push the Rotations button, MATLAB will run the results shown as below.
![avatar](figures/rtbdemo.PNG)   
You can also read the file [robot.pdf](https://github.com/ViolinLee/Pincher_projects/blob/master/1_kinematics_in_matlab/resources/robot.pdf) which describes the functions in RBT in some detail. You can find this in the Toolbox as rvctools/robot/robot.pdf. 

## Usage
### Step 0. Familiar with Pincher Arm's DH parameters
However, there are different forms of DH parameters convention. The standard DH is used to described the structure of the pincher arm, as the following graphs shown.
<center>![pincher_dh](figures/pincher_DH.PNG)</center>
<center><font color=grey>**Fig 1. Pincher Arm Links**</font></center>   
<center>![dh](figures/standard_dh.png)</center>
<center><font color=grey>**Fig 2. Definition of standard Denavit and Hartenberg link parameters.**</font></center>     
You can check the DH parameters of the pincher arm in MATLAB by commanding:

    >> mdl_phantomx
    >> px

![dh](figures/pincher_dh_rbt.png)   


### Step 1. Compute the Inverse Kinematics of Pincher Arm      
The following document describes how to determine inverse kinematics for pincher arm using the Robotics Toolbox for MATLAB: [4 is harder than 6DOF -- Inverse kinematics for underactuated robots](http://petercorke.com/wordpress/?ddownload=546). However, this documentation uses an old version RBT, and the coordinate configuration is different, which will lead to a inconsistent result.   
#### 1.Modification
The code need to been modified, you can test the inverse kinematics computation by texting the following commands in sequence:   
 
1) Moving to a position

    >> mdl_phantomx   
    >> px.plot(qz)   
    >> px.teach   
    >> Td = transl([100 80 -70])   
    >> q = px.ikine(Td, 'q0', qz, 'mask', [1 1 1 0 0 0])   
    >> px.plot(q)   
    >> px.fkine(q)   
    >> q = px.ikine(Td, 'q0', [0.5 1 -1 -0.5], 'mask', [1 1 1 0 0 0])   
    >> px.plot(q)   

2) Moving to a pose

    >> Td   
    >> Td = transl([100 80 -70]) * oa2tr([-1 0 0], [0 -1 0])   
    >> q = px.ikine(Td, 'q0', [0.5 1 -1 -0.5], 'mask', [1 1 1 1 0 0], 'tol', 0.8)   
    >> px.fkine(q)

#### 2.Instruction  
"Underactuation complicates the process of finding an inverse kinematic solution, and
it frustrates those who are new to robotics — those who just want to run the code and
get an answer. For a robot with 6 joints it’s quite straightforward, but underactuation
requires some careful thought about the problem that you are trying to solve — you
can’t just blindly use the tools."   
<p align="right">-- Peter Corke</p>

We will consider the problem in two parts. First the problem of moving the robot
tool to a particular position. Second, moving the tool to a particular position and tool
orientation.   

1) Moving to a position   
First, load a model of pincher arm (The coordinate system configurations of pincher arm in RBT 10 release is consistent with what is described in Fig 1).   

    >> mdl_phantomx

and then plot it for default, all zero, joint angles (we will see that the robot is pointing straight upwards).

    >> px.plot(qz)

![qz](figures/qz.png)
We can use the sliders in the teach pendant function.    

    >> px.teach

![qz](figures/teachPendant.png)
Now we will define the position we want the tool tip to move to

    >> Td = transl([100 80 -70]) 

and then compute the numerical inverse kinematics

    >> q = px.ikine(Td, 'q0', qz, 'mask', [1 1 1 0 0 0])
    
    q =    0.6747    1.7646   -2.2161    2.2219

Note that we have specified a mask value of (1 1 1 0 0 0) which indicates that we onlu care about errors in the x, y and z directions, rotational errors are to be ignored.   

    >> px.plot(q)

![pawk](figures/position_awk.png)
While this looks rather awkward the tool position is indeed what we requested.   

    >> px.fkine(q)   
    
    ans =    
       -0.7628   -0.1497    0.6290       100
       -0.6103   -0.1549   -0.7769        80
        0.2137   -0.9765    0.0268       -70
             0         0         0         1

Inverse kinematics is, in general, not uniquely defined. There are several arm configurations that will give the same tool position, and the numerical method has just chosen the awkward one. With a bit of trial and error, guided by the configuration we achieved using the teach pendant, we find that   

    >> q = px.ikine(Td, 'q0', [0.5 1 -1 -0.5], 'mask', [1 1 1 0 0 0])   
    q =    0.6747    0.8000   -1.6280   -0.8665
    >> px.plot(q)

![pguid](figures/position_guid.png)





2) Moving to a pose

### Step 2. Control Pincher using Robotics Toolbox   
The following article describes how to interfacing pincher arm to MATLAB. However, when running a more advanced version of the Robotics Toolbox, you need to make appropriate changes to the code.   
[Interfacing a hobby robot arm to MATLAB -- Using the Robotics Toolbox with a real robot](http://petercorke.com/wordpress/?ddownload=547)    


