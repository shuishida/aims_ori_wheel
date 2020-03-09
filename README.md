![alt text](ORI_AIMS_Week_Logo_Banner.png "logo")

# aims_cdt_week
Contains ROS packages to be used for the CDT week.

## Run
Currently, the simulation is run using either of:

roslaunch aims_jackal_sim jackal_sim.launch

roslaunch aims_jackal_sim jackal_sim_empty.launch

To use the PS4 controller, add the optional argument joystick:=true

Door configuration can be reset with:

rosrun aims_jackal_sim reset_sim.py

## Setup
Install the Jackal packages:

sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation

Install additional dependencies:

sudo apt-get install libzbar-dev

sudo apt-get install ros-melodic-hector-models

Currently it is necessary add to the Gazebo model path as follows:

export GAZEBO_MODEL_PATH=/path/to/catkin_workspace/src/aims_cdt_week/aims_jackal_sim/models:$GAZEBO_MODEL_PATH 

## pf_localisation package
Package for particle filter locaisation.

To run, add the following to your bashrc:
export PYTHONPATH=/path/to/catkin/workspace/src/aims_cdt_week/pf_localisation/src:$PYTHONPATH

You may get errors related to `lasertrace.so`. If this is the case, run `./compile.sh` in the laser trace directory.


## using the existing topological map
To run mongo_db, you will first need to make an empty folder called 'db' in the aims_jackal_sim folder. Start by running a mongo_db database:

roslaunch mongodb_store mongodb_store.launch db_path:=\`rospack find aims_jackal_sim\`/db

Insert the map from the .yaml file:

rosrun topological_utils load_yaml_map.py aims_navigation/maps/cdt/batlab_topmap.yaml

## creating a topological map

Start by running a mongo_db database:

roslaunch mongodb_store mongodb_store.launch db_path:=\`rospack find aims_jackal_sim\`/db

Insert a new map:

rosrun topological_utils insert_empty_map.py name_of_topological_map

Kill any mongo_db instances running. Then run:

roslaunch topological_rviz_tools strands_rviz_topmap.launch standalone:=true topmap:=name_of_topological_map db_path:=\`rospack find aims_jackal_sim\`/db map:=\`rospack find aims_jackal_sim\`/maps/map.yaml

Edit the map in rviz.
