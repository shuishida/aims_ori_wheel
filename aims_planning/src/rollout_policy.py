#!/usr/bin/env python
from build_search_and_rescue_ssp import build_spoof_topo_map
import numpy as np


node_to_loc_sv, name_to_loc_sv, topological_edges = build_spoof_topo_map()


def rollout_policy(state, action):

    # If can finish
        
        # Finish
    
    # If 2 people found
        
        # Can go to initial state (0)
        
            # Go to initial state
        
        # Can go out of room (uneven number)
            
            # Go closer to initial point, if possible
            
            # Go at least out of room
    
    # If less than 2 people found
        
        # If in room
        
            # If room unsearched (unknown)
            
                # Search room (check for person)
                
        # If can clear rubble
        
            # Clear rubble
            
        # If can check for rubble
        
            # Check for rubble
            
        # If in front of unsearched room
        
            # Go into room
            
        # If can move in front of unsearched room
        
            # Move in front of unsearched room

    if 'finish' in action:
        return 'finish'
    if state['num_people_found'] == 2: # 2 persons found
        if 'WayPoint9' in action:
            return 'WayPoint9' # Move to initial position
        if 'WayPoint4' in action:
            return 'WayPoint4' # Move outside of room 1
        if 'WayPoint2' in action:
            return 'WayPoint2' # Move outside of room 2
        if 'WayPoint1' in action:
            return 'WayPoint1' # Move outside of room 3
        if 'WayPoint3' in action:
            return 'WayPoint3' # Move outside of room 4
    else: # Less than 2 persons found
        if 'check_for_person' in action:
            return 'check_for_person'
        if 'clear_rubble' in action:
            return 'clear_rubble'
        if 'check_for_rubble' in action:
            return 'check_for_rubble'
    if state['room_1_person'] == 'unknown' and 'WayPoint8' in action: # Room 1 unsearched and in front of room 1
        return 'WayPoint8'
    if state['room_2_person'] == 'unknown' and 'WayPoint7' in action: # Room 2 unsearched and in front of room 2
        return 'WayPoint7'
    if state['room_3_person'] == 'unknown' and 'WayPoint5' in action: # Room 3 unsearched and in front of room 3
        return 'WayPoint5'
    if state['room_4_person'] == 'unknown' and 'WayPoint6' in action: # Room 4 unsearched and in front of room 4
        return 'WayPoint6'
    if state['room_1_person'] == 'unknown' and 'WayPoint4' in action: # Room 1 unsearched and can move in front of room 1
        return 'WayPoint4'
    if state['room_2_person'] == 'unknown' and 'WayPoint2' in action: # Room 2 unsearched and can move in front of room 2
        return 'WayPoint2'
    if state['room_3_person'] == 'unknown' and 'WayPoint1' in action: # Room 3 unsearched and can move in front of room 3
        return 'WayPoint1'
    if state['room_4_person'] == 'unknown' and 'WayPoint3' in action: # Room 4 unsearched and can move in front of room 4
        return 'WayPoint3'
   return np.random.choice(action)
