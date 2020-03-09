#!/bin/bash

SESSION=global

# Set this variable in order to have a development workspace sourced, surplus/instead of the .bashrc one
#DEVELOPMENT_WS=/opt/ros/kinetic/setup.bash
_SRC_ENV="tmux send-keys source Space $DEVELOPMENT_WS/devel/setup.bash C-m "


tmux -2 new-session -d -s $SESSION
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'robot'
tmux new-window -t $SESSION:2 -n 'mongo'
tmux new-window -t $SESSION:3 -n 'maps'
tmux new-window -t $SESSION:4 -n 'particle_filter'
tmux new-window -t $SESSION:5 -n 'nav'
tmux new-window -t $SESSION:6 -n 'planning'

tmux select-window -t $SESSION:0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux selectp -t 0
tmux send-keys "roslaunch ori_jackals sensors.launch" C-m

tmux selectp -t 1
tmux send-keys "roslaunch aims_jackal_sim qr_scanner.launch " C-m

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

tmux select-window -t $SESSION:4
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch --wait pf_localisation particle_filter.launch" C-m

tmux select-window -t $SESSION:5
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch aims_navigation navigation.launch"

# Set default window
tmux select-window -t $SESSION:6
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch --wait aims_planning planner.launch" C-m

tmux a
