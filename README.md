# SLAM-trajectory-comparsion

**fastlio launch**
```
CloudToScan
wslivox
fastlio
ros2 launch fast_lio turtlebot4_fastlio.launch.py
``` 

```
turtlebot4
ros2 bag play MyROSBag
```


**liosam launch**
```
CloudToScan
liosam
ros2 launch lio_sam run.launch.py
```
```
turtlebot4
ros2 bag play MyROSBag
```

**rtabmap launch**
```
turtlebot4
ros2 bag play MyROSBag --clock
```

```
rtabmap
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  rgb_topic:=/oakd/rgb/preview/image_raw \
  depth_topic:=/oakd/rgb/preview/depth \
  camera_info_topic:=/oakd/rgb/preview/camera_info \
  subscribe_scan:=true \
  scan_topic:=/scan \
  odom_topic:=/odom \
  rtabmap_viz:=true
```


---


**~/.bashrc**
```
source /opt/ros/humble/setup.bash
alias turtlebot4='source ~/turtlebot4_ws/install/setup.bash'
alias liosam='source ~/liosam_ws/install/setup.bash'
alias wslivox='source ~/ws_livox/install/setup.bash'
alias fastlio='source ~/fastlio_ws/install/setup.bash'
alias rtabmap='source ~/rtabmap_ws/install/setup.bash'
alias CloudToScan='source ~/CloudToScan_ws/install/setup.bash'
```
