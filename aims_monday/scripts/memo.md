```
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 '[0.1, 0, 0]' '[0, 0, 0]'

rosrun rviz rviz -d jackal_viz.rviz
```
