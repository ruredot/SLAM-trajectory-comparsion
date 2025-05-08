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
