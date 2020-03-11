#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np

from mcts_decision_node import MCTSDecisionNode
from mcts_chance_node import MCTSChanceNode

from stringbuilder import StringBuilder






def ucb_selection_policy(mcts_decision_node):
    """
    TODO: your code here

    Your code needs to return a valid action that can be taken from this 
    decision node.
    """
    C = 1.0 # Parameter
    action_set = mcts_decision_node.enabled_actions # All possible actions
    n_s = mcts_decision_node.observations # The number of times that this node has been visited    
    best_action = float('NaN')
    best_value = float('Inf')    
    for current_action in action_set:
        current_chance_node = mcts_decision_node.children[current_action] # Chance nodes - correspond to state-action pairs and are used to keep estimate of Q(s,a)
        Q_hat = current_chance_node.value # The current estimate of the value at this node.
        n_s_a = current_chance_node.observations # The number of times that this node has been visited
        current_value = Q_hat - C * np.sqrt(np.log(n_s)/n_s_a)
        if current_value <= best_value:
            best_value = current_value
            best_action = current_action
    return best_action

def random_rollout_policy(state, enabled_actions):
    """
    TODO: your code here

    Your code needs to return a valid action from this state.
    """
    return np.random.choice(enabled_actions)



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
