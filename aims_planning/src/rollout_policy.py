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
   return np.random.choice(action)
