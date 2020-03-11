#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np

from stringbuilder import StringBuilder





class MCTSChanceNode(object):
    """
    A node associated with a 'state' and 'action' in an ssp, used the mcts 
    tree. It is a chance node because successor states are sampled after 
    the state action pair associated with this node.

    Attributes:
        state: The state (a State object) associated with this chancce node
        action: The action (string) associated with this chance node
        immediate_cost: The immediate cost C(s,a) for 'state' and 'action'
        successor_state_distribution: A dictionary with State object keys,
            where successor_state_distribution[s] is the probability that state
            's' is the successor state after this state-action. I.e. this
            dictionary encodes Pr(.|state,action).
        observations: The number of times that this node has been visited
        value: The current estimate of the value at this node.
        children: A map to be used for keeping track of children nodes.  (Keys 
            are the successor states associated with each of the child decision 
            nodes).
    """
    def __init__(self, ssp, state, action):
        """
        Initialiser for chance node, which initialises the attributes of the 
        chance node.

        Currently the successor state distribution is not used, so is commented 
        out, but you may want to uncomment it if you wish to use it.
        """
        self.state = state
        self.action = action
        self.immediate_cost = ssp.get_cost(state, action)
        self.successor_state_distribution = \
            ssp.get_transition_probs(state, action)
        self.observations = 0
        self.value = 0.0
        self.children = {}

    def add_child(self, new_child_node, succ_state):
        """
        Adds 'new_child_node' as a child node corresponding to state 
        'succ_state'.
        """
        self.children[succ_state] = new_child_node


    def has_child_for_succ(self, successor_state):
        """
        Returns a boolean for if this chance node has a child decision node 
        associated with 'successor_state'
        """
        return (successor_state in self.children)

    def get_child_for_succ(self, successor_state):
        """
        Returns the chance node associated with taking action 'action' from
        this decision node.
        """
        if successor_state not in self.children:
            print(self.children)
            raise Exception("Trying to get chance node for unknown "
                            "successor state: " + str(successor_state))
        return self.children[successor_state]

    def backup(self, sampled_return):
        """
        YOUR CODE HERE
        You should update self.observations and self.value here

        Args:
            sampled_return: A return (the sum of costs from the state-action
                pair for this chance node) sampled from this node by MCTS
        """
        self.observations += 1
        self.value += (sampled_return - self.value) / self.observations

    def pretty_print(self, depth, num_tabs=0, new_lines=False):
        """
        Returns a string that pretty prints the tree rooted from this node and 
        values to a certain depth. This can be useful for debugging.

        Args:
            depth: the depth of the search tree to print out
            num_tabs: the amount of indentation use when printing this level
            new_lines: if we should add new lines to try make it more readable
        Returns:
            String, a pretty printed tree, using D for decision nodes and C for
            chance nodes. We use parenthesis '()' to denote children and use
            square parenthesis '[]' to denote the value of a node and how many 
            times it has been selected.
        """
        sb = StringBuilder()
        indent = '\t' * num_tabs
        sb.append("C[")
        if isinstance(self.value, float):
            sb.append("{val:.3f}".format(val=self.value))
        else:
            sb.append("{val}".format(val=self.value))
        sb.append(",")
        sb.append(str(self.observations))
        sb.append("](")

        if depth == 0:
            sb.append("...)")
            return sb.to_string()

        first_iter = True
        for state, prob in self.successor_state_distribution.items():
            if not self.has_child_for_succ(state):
                continue

            if not first_iter:
                sb.append(", ")
            else:
                first_iter = False

            if new_lines:
                sb.append("\n")
                sb.append(indent)

            sb.append("{prob:.3f}".format(prob=prob))
            sb.append("->")
            child_node_string = self.children[state].pretty_print(
                    depth=depth-1,
                    num_tabs=num_tabs+1,
                    new_lines=new_lines)
            sb.append(child_node_string)

        sb.append(")")
        return sb.to_string()

    def print_most_selected_path(self, delimiter='->'):
        """
        Returns a string that visualizes the path in the tree that has been 
        taken most frequently. This can be useful for debugging.
        """
        sb = StringBuilder()
        best_val = np.inf
        best_child = None
        best_action = None
        best_prob = 0.0
        most_observations = 0
        for state in self.children:
            child = self.children[state]
            prob = self.successor_state_distribution[state]
            if most_observations < child.observations:
                most_observations = child.observations
                best_child = self.children[state]
                best_val = best_child.value
                best_prob = prob

        if best_val == -np.inf:
            best_val = 0.0

        sb.append("(p={prob},v={val}){d}".format(prob=best_prob,
                                                 val=best_val,
                                                 d=delimiter))
        if best_child is not None:
            sb.append(best_child.print_most_selected_path(delimiter=delimiter))
        return sb.to_string()