#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import deepcopy
import math
import numpy as np

from mcts_decision_node import MCTSDecisionNode
from mcts_chance_node import MCTSChanceNode

from stringbuilder import StringBuilder


def _sample_from_distribution(dict):
    keys, probs = zip(*dict.items())
    return np.random.choice(keys, p=probs)


def _ucb_selection_policy(mcts_decision_node, epsilon=0.2):
    if np.random.random() <= epsilon:
        return np.random.choice(mcts_decision_node.enabled_actions)
    else:
        return _ucb_selection_policy(mcts_decision_node)


def ucb_selection_policy(mcts_decision_node):
    """
    TODO: your code here

    Your code needs to return a valid action that can be taken from this 
    decision node.
    """

    C = max(10, abs(mcts_decision_node.value)) # 1000.0 # Parameter
    # C = 1.0
    action_set = deepcopy(mcts_decision_node.enabled_actions) # All possible actions
    np.random.shuffle(action_set)

    for current_action in action_set:
        if current_action not in mcts_decision_node.children.keys():
            return current_action

    n_s = mcts_decision_node.observations # The number of times that this node has been visited
    best_action = 'NaN'
    best_value = float('Inf')
    for current_action in action_set:
        current_chance_node = mcts_decision_node.children[current_action] # Chance nodes - correspond to state-action pairs and are used to keep estimate of Q(s,a)
        Q_hat = current_chance_node.value # The current estimate of the value at this node.
        n_s_a = current_chance_node.observations # The number of times that this node has been visited
        current_value = Q_hat - C * np.sqrt(np.log(n_s)/n_s_a)
        # print(current_value)
        if current_value < best_value:
            best_value = current_value
            best_action = current_action
    if best_action is 'NaN':
        raise("No best action available.")
    # print("%s %s Number of available actions: %s" % (best_action, best_value, len(action_set)))
    return best_action

def random_rollout_policy(state, enabled_actions):
    action = enabled_actions
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
    if state['num_people_found'] == 2:  # 2 persons found
        if 'WayPoint9' in action:
            return 'WayPoint9'  # Move to initial position
        if 'WayPoint4' in action:
            return 'WayPoint4'  # Move outside of room 1
        if 'WayPoint2' in action:
            return 'WayPoint2'  # Move outside of room 2
        if 'WayPoint1' in action:
            return 'WayPoint1'  # Move outside of room 3
        if 'WayPoint3' in action:
            return 'WayPoint3'  # Move outside of room 4
    else:  # Less than 2 persons found
        if 'check_for_person' in action:
            return 'check_for_person'
        if 'clear_rubble' in action:
            return 'clear_rubble'
        if 'check_for_rubble' in action:
            return 'check_for_rubble'
    if state['room_1_person'] == 'unknown' and 'WayPoint8' in action:  # Room 1 unsearched and in front of room 1
        return 'WayPoint8'
    if state['room_2_person'] == 'unknown' and 'WayPoint7' in action:  # Room 2 unsearched and in front of room 2
        return 'WayPoint7'
    if state['room_3_person'] == 'unknown' and 'WayPoint5' in action:  # Room 3 unsearched and in front of room 3
        return 'WayPoint5'
    if state['room_4_person'] == 'unknown' and 'WayPoint6' in action:  # Room 4 unsearched and in front of room 4
        return 'WayPoint6'
    if state[
        'room_1_person'] == 'unknown' and 'WayPoint4' in action:  # Room 1 unsearched and can move in front of room 1
        return 'WayPoint4'
    if state[
        'room_2_person'] == 'unknown' and 'WayPoint2' in action:  # Room 2 unsearched and can move in front of room 2
        return 'WayPoint2'
    if state[
        'room_3_person'] == 'unknown' and 'WayPoint1' in action:  # Room 3 unsearched and can move in front of room 3
        return 'WayPoint1'
    if state[
        'room_4_person'] == 'unknown' and 'WayPoint3' in action:  # Room 4 unsearched and can move in front of room 4
        return 'WayPoint3'
    if state[
        'room_1_person'] != 'unknown' and 'WayPoint8' in action and len(action) > 1: # Room 1 already searched and can move into room 1 as well as to other location
        action = [a for a in action if a != 'WayPoint8']
    if state[
        'room_2_person'] != 'unknown' and 'WayPoint7' in action and len(action) > 1: # Room 2 already searched and can move into room 2 as well as to other location
        action = [a for a in action if a != 'WayPoint7']
    if state[
        'room_3_person'] != 'unknown' and 'WayPoint5' in action and len(action) > 1: # Room 3 already searched and can move into room 3 as well as to other location
        action = [a for a in action if a != 'WayPoint5']
    if state[
        'room_4_person'] != 'unknown' and 'WayPoint6' in action and len(action) > 1: # Room 4 already searched and can move into room 4 as well as to other location
        action = [a for a in action if a != 'WayPoint6']
    #if len(action) > 1:
        #print("No scripting available, but more than 1 action.")
        #print(state)
        #print(action)
    return np.random.choice(action)



def get_rollout_value(ssp, rollout_policy, state, action=None):
    """
    Performs a rollout from 'state' or '(state,action)', using the rollout 
    policy in the 'ssp'. 

    Returns:
        A sampled value from the rollout.
    """
    s = state
    a = action
    enabled_actions = ssp.get_actions(s)
    tot_cost = 0.0

    while len(enabled_actions) > 0:
        if action is not None:
            a = action
            action = None
        else:
            a = rollout_policy(s, enabled_actions)

        tot_cost += ssp.get_cost(s, a)
        s = _sample_from_distribution(ssp.get_transition_probs(s, a))
        enabled_actions = ssp.get_actions(s)

    return tot_cost


def mcts(ssp, 
         state, 
         selection_policy=None, 
         rollout_policy=None, 
         root_decision_node=None, 
         stop_mcts_callback_fn=None, 
         max_iters=np.inf,
         print_most_selected_path_num_iters=None):
    """
    Runs monte carle tree search over the tree rooted at 'root_decision_node' 
    which should correspond to state 'state', in the SSP 'ssp'. Policy functions
    are passed in that can be used for the selection and rollout steps.

    Args:
        ssp: A SSP object that we want plan in
        state: A valid state in the ssp that we want to plan from
        selection_policy: A function that takes a decision node and returns a 
            valid action in the ssp that can be chosen from the decision node. 
            This policy is used to select nodes in the selection phase.
        rollout_policy: A function that takes a state and a list of enabled 
            actions, and returns an action in enabled actions to be used in the 
            rollout phase.
        root_decision_node: An optional decision node to pass in, so that 
        stop_mcts_callback_fn: A function that takes no arguments and should 
            return a boolean specifying if it is time to finish 
        max_iters: A maximum number of iterations to use 

    Returns:
        The root_decision_node, which is the root of the MCTS tree.
    """
    if selection_policy is None:
        selection_policy = ucb_selection_policy
    if rollout_policy is None:
        rollout_policy = random_rollout_policy
    if root_decision_node is None:
        root_decision_node = MCTSDecisionNode(ssp, state)
    if stop_mcts_callback_fn is None:
        stop_mcts_callback_fn = (lambda: False)

    i = 0
    while ((not stop_mcts_callback_fn()) and i < max_iters):
        if (print_most_selected_path_num_iters is not None 
            and (i % print_most_selected_path_num_iters) == 0):
            print(root_decision_node.print_most_selected_path())

        # selection
        selected_nodes = [root_decision_node]
        costs = [0.0]
        s = root_decision_node.state
        a = None
        decision_node = root_decision_node
        chance_node = None

        while not decision_node.is_leaf():
            a = selection_policy(decision_node)
            if not decision_node.has_child_for_action(a):
                break

            chance_node = decision_node.get_child_for_action(a)
            selected_nodes.append(chance_node)
            costs.append(ssp.get_cost(s,a))

            s = _sample_from_distribution(ssp.get_transition_probs(s,a))
            if not chance_node.has_child_for_succ(s):
                break

            decision_node = chance_node.get_child_for_succ(s)
            selected_nodes.append(decision_node)
            costs.append(0.0)

        # expansion
        # note that decision and chance nodes alternate
        # also note that selection could reach a leaf node, in which case we do 
        # not expand
        added_new_node = True
        if (len(selected_nodes) % 2) == 0:
            new_decision_node = MCTSDecisionNode(ssp, s)
            chance_node.add_child(new_decision_node, s)
            selected_nodes.append(new_decision_node)
            a = None
        elif not decision_node.is_leaf():
            new_chance_node = MCTSChanceNode(ssp, s, a)
            decision_node.add_child(new_chance_node, a)
            selected_nodes.append(new_chance_node)
        else:
            added_new_node = False

        # rollout
        # only rollout to initialize the value of a newly expanded node
        # if a decision node was added, then a==None. 
        if added_new_node:
            return_from_rollout = get_rollout_value(ssp, rollout_policy, s, a)
            costs.append(return_from_rollout)

        # backup
        sampled_return_from_node = 0.0
        while len(selected_nodes) > 0:
            sampled_return_from_node += costs.pop()
            node = selected_nodes.pop()
            node.backup(sampled_return_from_node)

        # Completed iteration, increment i
        i += 1

    return root_decision_node
