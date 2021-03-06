#!/bin/bash

SESSION=global

# Set this variable in order to have a development workspace sourced, surplus/instead of the .bashrc one
DEVELOPMENT_WS=/home/cdt/catkin_ws
#DEVELOPMENT_WS=/opt/ros/kinetic/setup.bash
_SRC_ENV="tmux send-keys source Space $DEVELOPMENT_WS/devel/setup.bash C-m "


tmux -2 new-session -d -s $SESSION
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'sim'
tmux new-window -t $SESSION:2 -n 'mongo'
tmux new-window -t $SESSION:3 -n 'maps'
tmux new-window -t $SESSION:4 -n 'particle_filter'
tmux new-window -t $SESSION:5 -n 'nav'
tmux new-window -t $SESSION:6 -n 'planner'
tmux new-window -t $SESSION:7 -n 'rviz'

tmux select-window -t $SESSION:0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch --wait aims_jackal_sim jackal_sim.launch joystick:=true" C-m


tmux select-window -t $SESSION:2
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch --wait mongodb_store mongodb_store.launch db_path:=`rospack find aims_jackal_sim`/db"

tmux select-window -t $SESSION:3
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux selectp -t 0
tmux send-keys "roslaunch --wait topological_navigation topological_map_viz_only.launch topo_map:=aims_map"

# sleep to start map after gazebo to avoid tf warning
tmux selectp -t 1
tmux send-keys  "sleep 10" C-m
tmux send-keys "roslaunch --wait aims_navigation map.launch" C-m
heigth
tmux select-window -t $SESSION:4
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch --wait pf_localisation particle_filter.launch" C-m

tmux select-window -t $SESSION:5
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch --wait aims_navigation navigation.launch" C-m

tmux select-window -t $SESSION:6
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch --wait aims_planning planner.launch"

tmux select-window -t $SESSION:7
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "rviz -d $DEVELOPMENT_WS/src/aims_ori_wheel/jackal_viz.rviz"

# Set default window
tmux select-window -t $SESSION:7
tmux a
