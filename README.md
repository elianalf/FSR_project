# Starting from this :
# To install:
```
$ cd <catkin_ws>/src
$ git clone https://github.com/elianalf/FSR.git
$ cd ..
$ catkin_make
```

# To use:
```
To make sure your workspace is properly overlayed by the setup script.
$ source <catkin_ws>/devel/setup.sh
Launching Environment
$ roslaunch navigation_ smallwarehouse.launch
Run all the files
$ rosrun navigation_ nav_main

It's possible also to launch Rviz
$ rviz
```

# Note about videos:
```
fb_-4_8.mp4: tracking trajectory from (0,0) to (-4,8), view on RVIZ with feedback linearization
nl_-4_8.mp4: tracking trajectory from (0,0) to (-4,8), view on RVIZ with nonlinear control
turt_-3_8.mp4: tracking trajectory from (0,0) to (-3,8), view on GAZEBO with feedback linearization
turt_0_-5.mp4: tracking trajectory from (0,0) to (0,-5), view on GAZEBO with feedback linearization



```
