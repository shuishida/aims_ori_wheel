#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np

from stringbuilder import StringBuilder



class MCTSDecisionNode(object):
    """
    A node associated with a 'state' in an ssp, used the mcts tree. It is called
    a decision node because you should pick an action from this node.

    Attributes:
        state: The state (a State object) associated with this chancce node
        enabled_actions: A list of actions that are valid from 'state'
        observations: The number of times that this node has been visited
        value: The current estimate of the value at this node.
        children: A map to be used for keeping track of children nodes. (Keys 
            are the actions needed to reach the child chance nodes).
    """

    def __init__(self, ssp, state):
        """
        Initialiser for decision node, which initialises the attributes of the 
        decision node.
        """
        self.state = state
        self.enabled_actions = ssp.get_actions(state)
        self.observations = 0.0
        self.value = 0.0
        self.children = {}

    def is_leaf(self):
        """
        Returns if a decision node is a leaf. In this case if num actions is
        zero.
        """
        return len(self.enabled_actions) == 0

    def add_child(self, new_child_node, action):
        """
        Adds 'new_child_node' as a child node corresponding to the state action 
        pair '(self.state, action)'. 
        """
        self.children[action] = new_child_node

    def has_child_for_action(self, action):
        """
        Returns a boolean for if there is a child chance node associated with 
        the action name 'action'.
        """
        return (action in self.children)

    def get_child_for_action(self, action):
        """
        Returns the chance node associated with taking action 'action' from
        this decision node.
        """
        if action not in self.children:
            raise Exception("Trying to get chance node for unknown "
                            "action: " + str(action))
        return self.children[action]

    def get_best_action(self):
        """
        Returns the best action that can be chosen greedily from this 
        state/decision node. I.e. the action corresponding to the child with 
        the lowest value.
        """
        best_value = np.inf
        best_action = None
        for action in self.children:
            if self.children[action].value < best_value:
                best_value = self.children[action].value
                best_action = action
        return best_action

    def backup(self, sampled_return):
        """
        YOUR CODE HERE
        You should update self.observations and self.value here

        Args:
            sampled_return: A return (the sum of costs from this state in a MCTS
                rollout) sampled from this node by MCTS
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
        sb.append("D[")
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
        for action in self.children:
            if not first_iter:
                sb.append(", ")
            else:
                first_iter = False

            if new_lines:
                sb.append("\n")
                sb.append(indent)

            sb.append(action)
            sb.append("->")
            child_node_string = self.children[action].pretty_print(
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
        most_selections = 0.0
        best_child = None
        best_action = None
        for action in self.children:
            if self.children[action].value is not None:
                if most_selections < self.children[action].observations:
                    most_selections = self.children[action].observations
                    best_child = self.children[action]
                    best_val = best_child.value
                    best_action = action

        if best_child is not None:
            sb.append("(a={action},v={val}){d}".format(action=best_action,
                                                    val=best_val,
                                                    d=delimiter))
            sb.append(best_child.print_most_selected_path(delimiter=delimiter))
        else:
            sb.append("Leaf decision node" + delimiter)
        return sb.to_string()