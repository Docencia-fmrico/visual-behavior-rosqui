<p align="center"><a href="https://www.youtube.com/channel/UC4Loc3tyy1vvGsMoBC5KCSw" target="_blank">
    <img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/logo.jpg">
</a></p>

<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/kobuki.jpg" align="right"
     alt="Kobuki Image" width="270" height="200">
    

<h1>VISUAL BEHAVIOUR</h1>

<p> We were asked to complete the following task which, in this case we were obliged to use <b>behaviour trees</b>, a different way of approaching the solution of these exercises instead of using finite state machine.
 </p>

This task is called visual navigation as we have had to handle 3 exercises:

<h2>FOLLOW PERSON</h2>
First of all we have done a behaviour tree called <b>follow person</b>. As you can see, by its name we can know that the task it performs is following a person. 
We used darknet ros filtered just only for people.

<details><summary><b>Instalation</b></summary>
For this task we had to install the following packages:
    
    
    $ sudo apt-get install ros-noetic-behaviortree-cpp-v3
    $ sudo apt-get install ros-noetic-move-base-msgs
    $ sudo apt-get install ros-noetic-kobuki-*
    $ sudo apt-get install openni2-*
    $ sudo apt-get install ros-noetic-rgbd-launch 
    $ sudo apt-get install --fix-missing ros-noetic-rgbd-launch
    $ sudo apt-get install ros-noetic-cameras-*
    
If you want to try it by your own personal camera, you can download the following package:
   
    $ sudo apt-get install ros-noetic-usb-cam-*
   
By the way, if you get in trouble with any package you may need to download a github repository related to the package. 

</details>

<details><summary><b>Commands Used</b></summary>
    
- <b>See Darknet Ros using 3D camera</b>
    
    You should follow the followings steps:
    
        $ roslaunch openni2_launch openni2.launch
        $ roslaunch darknet_ros darknet_ros.launch image:=/camera/rgb/image_raw/
    
- <h4>See Darknet Ros using usb camera</h3>
    
    You should follow the followings steps:
    
        $ roscore
        $ rosrun usb_cam usb_cam_node
        $ rosrun cameras_cpp nodo_camera
        $ roslaunch darknet_ros darknet_ros.launch iamge:=/usb_cam/image_raw/
      
</details>

<details><summary><b>Behaviour tree</b></summary>
    
This is the tree we have decided to use:
 
<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/bt_fb.gif" align="center"
alt="Follow person bt" width="600" height="600">

And this is how it looks like in <a href="https://github.com/BehaviorTree/Groot">Groot</a>:

<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/groot_fp_bt.gif" align="center"
alt="Follow person bt groot" width="600" height="600">
        
    
</details>

<details><summary><b>Filtered Darknet Ros</b></summary>
 We want to outline that using the Darknet Ros <a href="https://github.com/leggedrobotics/darknet_ros">github repo</a> only for people, we had to edit all yamls and we just left <b>person</b> in <b>detection clases names</b> 
    
Here you can see a picture of it:
    
    
</details>
 
<h2>FOLLOW BALL</h2>
First of all we have done a behaviour tree called <b>follow ball</b>. As you can see, by its name we can know that the task it performs is following a ball. 
We used .
    
<details><summary><b>Instalation</b></summary>
 
    
</details>
    
<details><summary><b>Commands Used</b></summary>
        
- <b>Filter Ball using 3D camera</b>
    
    You should follow the followings steps:
    
        $ roslaunch openni2_launch openni2.launch
        $ rosrun cameras_cpp nodo_camera (filter image)
        $ rviz 
    
    In rviz add image and its topic is /hsv/image_filtered/
    And these are the values used for filtering the ball:
    
 - <b>Filter Ball using rviz</b>
    
     You should follow the followings steps:
    
        $ roslaunch robots sim.launch
        $ rosrun cameras_cpp nodo_camera (filter image)
        $ rosrun cameras_cpp nodo_rgbd_filtered (publish in the image filtered topic)
        $ roslaunch robots kobuki_xtion.launch (makes the transform)
        $ rviz (and choose 0 channel)
        $ roslaunch kobuki_keyop keyop.launch (for moving in the simulation and the image)
        
    
</details>
    
<details><summary><b>Behaviour tree</b></summary>

This is the tree we have decided to use:
 
<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/bt_fp.gif" align="center"
alt="Follow ball bt" width="600" height="600">

And this is how it looks like in <a href="https://github.com/BehaviorTree/Groot">Groot</a>:

<img src="" align="center"
alt="Follow ball bt groot" width="600" height="600">
</details>

<h2>FOLLOW BOTH</h2>

<details><summary><b>Behaviour tree</b></summary>
</details>

