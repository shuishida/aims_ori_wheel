#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import copy
import math
import time

from build_search_and_rescue_ssp import make_search_and_rescue_ssp
from build_search_and_rescue_ssp import get_rubble_clearing_time_cost
from build_search_and_rescue_ssp import get_check_for_person_time_cost
from build_search_and_rescue_ssp import get_length_time_cost
from build_search_and_rescue_ssp import read_in_topo_map
from strands_navigation_msgs.msg import TopologicalMap
from aims_rubble_check.msg import *
from aims_msgs.msg import SearchRoomAction, SearchRoomGoal
from aims_msgs.msg import NavigateAction, NavigateGoal
from aims_door_detector.msg import DoorClearAction, DoorClearGoal
from geometry_msgs.msg import PoseStamped

from state import State
from mcts import mcts
from random import random
import actionlib

BURN_IN_TRIALS = 2500
DOOR_NUM_READINGS = 30



def create_clients():
    """
    Returns a list of action servers for the search and rescue task
    """
    clients = {}

    rospy.loginfo("Waiting for rubble detector")
    clients['rubble_detect'] = actionlib.SimpleActionClient('rubble_detect',
                                                            RubbleDetectAction)

    rospy.loginfo("Waiting for rubble checker")
    clients['rubble_check'] = actionlib.SimpleActionClient('rubble_check',
                                                            RubbleCheckAction)
    clients['rubble_check'].wait_for_server()
    rospy.loginfo("rubble_check connected")

    rospy.loginfo("Waiting for room searcher")
    clients['search_room'] = actionlib.SimpleActionClient('search_room',
                                                          SearchRoomAction)
    clients['search_room'].wait_for_server()
    rospy.loginfo("search_room connected")

    rospy.loginfo("Waiting for navigator")
    clients['navigation'] = actionlib.SimpleActionClient('navigation',
                                                         NavigateAction)
    clients['navigation'].wait_for_server()
    rospy.loginfo("navigation connected")

    rospy.loginfo("Waiting for door clearer")
    clients['rubble_clear'] = actionlib.SimpleActionClient('rubble_clear',
                                                         RubbleClearAction)
    clients['rubble_clear'].wait_for_server()
    rospy.loginfo("rubble clear connected")

    rospy.loginfo("All clients for policy executor set up")

    return clients


trials = BURN_IN_TRIALS
def burn_in_finished():
    """Used as a 'burn in' action, to run 'BURN_IN_TRIALS' many trials"""
    global trials
    if trials <= 0:
        return True
    trials -= 1
    return False



def call_navigation_action(planning_action, clients):
    """
    Skelly function

    Calls some action server for navigation.
    I haven't worked on getting the right nav_goal because I didnt have the
    topological map stuff

    In my ssp, the nodes are encoded in the numbers 0,...,8 and the action '5'
    is to navigate to location 5.
    """

    top_map = rospy.wait_for_message('/topological_map', TopologicalMap, timeout=20)

    nodes = top_map.nodes
    pose = None
    for node in nodes:
        if node.name == planning_action:
            pose = node.pose
            break

    goal = NavigateGoal()
    goal.goal = pose
    clients['navigation'].send_goal(goal)

    def is_finished():
        return ((clients['navigation'].get_state() == 3) or
               (clients['navigation'].get_state() == 2) or
               (clients['navigation'].get_state() == 4))

    def return_fn():
        return planning_action

    return (lambda: is_finished()), (lambda: return_fn())



def call_check_for_rubble_action(planning_action, clients):
    """
    Checks if rubble is cleared or not.

    Checks if a pile of rubble is large or small
    """

    goal = RubbleDetectGoal()
    clients['rubble_detect'].send_goal(goal)
    clients['rubble_detect'].wait_for_result()

    rubble_goal = RubbleCheckGoal()
    clients['rubble_check'].send_goal(rubble_goal)
    clients['rubble_check'].wait_for_result()

    def is_finished():
        door_fin = ((clients['rubble_detect'].get_state() == 3) or
                    (clients['rubble_detect'].get_state() == 2) or
                    (clients['rubble_detect'].get_state() == 4))
        rubble_fin = ((clients['rubble_check'].get_state() == 3) or
                      (clients['rubble_check'].get_state() == 2) or
                      (clients['rubble_check'].get_state() == 4))
        return door_fin and rubble_fin


    def result_fn():
        cleared = clients['rubble_detect'].get_result().open
        if cleared:
            return 'cleared'
        else:
            return clients['rubble_check'].get_result().rubble_size + "_pile"

    return (lambda: is_finished()), (lambda: result_fn())


def call_clear_rubble_action(planning_action, clients):
    """
    Calls door detector until open.

    Scans for a person when in a room
    """

    goal = RubbleClearGoal()
    clients['rubble_clear'].send_goal(goal)

    def is_finished():
        return ((clients['rubble_clear'].get_state() == 3) or
               (clients['rubble_clear'].get_state() == 2) or
               (clients['rubble_clear'].get_state() == 4))

    def return_fn():
        return 'cleared'

    return is_finished, return_fn


def call_search_for_person_action(planning_action, clients):
    """
    Calls search_room action server.

    Scans for a person when in a room
    """

    goal = SearchRoomGoal()
    clients['search_room'].send_goal(goal)
    clients['search_room'].wait_for_result()

    def is_finished():
        return ((clients['search_room'].get_state() == 3) or
               (clients['search_room'].get_state() == 2) or
               (clients['search_room'].get_state() == 4))

    def is_found():
        res = clients['search_room'].get_result()
        if res.target_found:
            return 'found'
        else:
            return 'missing'

    finished_fn = lambda: is_finished()
    result_fn = lambda: is_found()

    return finished_fn, result_fn


def call_action_server(action, clients):
    if action == 'check_for_rubble':
        return call_check_for_rubble_action(action, clients)
    elif action == 'clear_rubble':
        return call_clear_rubble_action(action, clients)
    elif action == 'check_for_person':
        return call_search_for_person_action(action, clients)
    elif action == 'finish':
        pass
    else: # navigation action (action = node number)
        return call_navigation_action(action, clients)

def update_state_from_action_callback(state, action, action_result_callback_fn):
    state_mapping = copy.copy(state._state_mapping)

    if action == 'check_for_rubble':
        rubble_size = action_result_callback_fn() # rubble size == 'small_pile' || rubble size == 'large_pile' || rubble size == "cleared"
        print("Found {p} pile".format(p=rubble_size))
        cur_node = state['location']
        room_number = int((cur_node + 1) / 2)
        state_factor_name = 'room_{i}_rubble'.format(i=room_number)
        state_mapping[state_factor_name] = rubble_size

    elif action == 'clear_rubble':
        cleared = action_result_callback_fn() # cleared == 'cleared'
        cur_node = state['location']
        room_number = int((cur_node + 1) / 2)
        state_factor_name = 'room_{i}_rubble'.format(i=room_number)
        large_pile = (state_mapping[state_factor_name] == 'large_pile')
        state_mapping[state_factor_name] = cleared
        state_mapping['time'] += get_rubble_clearing_time_cost(large_pile)

    elif action == 'check_for_person':
        state_mapping['time'] += get_check_for_person_time_cost()
        person_status = action_result_callback_fn() # person_status == 'missing' || person_status == 'found'
        print("CHecked for person and they were {p}".format(p=person_status))
        cur_node = state['location']
        room_number = int(cur_node / 2)
        state_factor_name = 'room_{i}_person'.format(i=room_number)
        state_mapping[state_factor_name] = person_status

        if person_status == 'found':
            state_mapping['num_people_found'] += 1

        if 'num_rooms_searched' in state_mapping:
            state_mapping['num_rooms_searched'] += 1

    elif action == 'finish':
        _ = action_result_callback_fn()
        state_mapping['location'] = -1

    else: # navigation action (action = node number)
        node_map, name_to_location_val, topological_edges = read_in_topo_map()
        node_name_ended_up_at = action_result_callback_fn() # any valid node, assumed int type already
        last_location_val = int(state_mapping['location'])
        next_location_val = name_to_location_val[node_name_ended_up_at]
        state_mapping['location'] = next_location_val
        for e in topological_edges:
            if ((node_map[e.n1] == last_location_val and node_map[e.n2] == next_location_val) or
                (node_map[e.n2] == last_location_val and node_map[e.n1] == next_location_val)):
                state_mapping['time'] += math.ceil(e.length()*get_length_time_cost())
                break

    return State(state_mapping)





def policy_execution():
    """
    TODO (michael): add the action option to mcts, so dont waste lots of trials
    """
    # Create action clients
    clients = create_clients()

    # create ssp
    ssp = make_search_and_rescue_ssp()
    state = ssp.get_initial_state()

    # Start with a burn in period
    action_finished_callback_fn = burn_in_finished
    mcts_root_node = mcts(
            ssp = ssp,
            state = state,
            stop_mcts_callback_fn=action_finished_callback_fn)

    # Execute actions until hit a sink state
    while not mcts_root_node.is_leaf():
        print("MCTS time value is {v}".format(v=mcts_root_node.state['time']))
        # Pick best action and run it
        action = mcts_root_node.get_best_action()
        if action == 'finish':
            print("Policy Completed Executing")
            return
        print("Taking action: {a}".format(a=action))
        action_finished_callback_fn, action_result_callback_fn = call_action_server(action, clients)

        # Run MCTS untill we need to call a new action server
        mcts_root_node = mcts(
            ssp = ssp,
            state = state,
            # action = action,
            root_decision_node=mcts_root_node,
            stop_mcts_callback_fn=action_finished_callback_fn)

        # Current action is done, sample next state
        state = update_state_from_action_callback(state, action, action_result_callback_fn)

        # Update root node
        chance_node = mcts_root_node.get_child_for_action(action)
        mcts_root_node = chance_node.get_child_for_succ(state)
    print("exiting because leaf node")
    print(mcts_root_node.state)



if __name__ == '__main__':
    rospy.init_node('policy_executor')
    policy_execution()
    rospy.spin()
