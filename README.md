# YuMi
This package contains the manuals and the source codes of Yumi robot.


# Before running the controller:

There are several steps you need to take before you run the controller. 

1. Set-upping YuMi in Robot-Studio
2. Calibrating the arms from the teach-pad
3. Running the tasks from the teach-pad

Then, you can run the interface on the Ubuntu machine. I also put the back-up of the RAPID in this repo. To be able to run the robot, there are several nice and comprehensive Wikis which you can follow:

[https://github.com/kth-ros-pkg/yumi/wiki](https://github.com/kth-ros-pkg/yumi/wiki) 

[https://github.com/ethz-asl/yumi/wiki](https://github.com/ethz-asl/yumi/wiki)


To run the interface, you need to run:
````
roslaunch yumi_launch yumi_vel_control.launch
````

To run the example you need to run:
````
rosrun yumi_motion_example yumi_motion_example_node
````
