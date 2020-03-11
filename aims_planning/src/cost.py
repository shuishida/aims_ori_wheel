#!/usr/bin/env python
# -*- coding: utf-8 -*-


import copy
from collections import defaultdict

from condition import Condition
from state import State


class StateActionCost(object):
    """Defines a cost associated with a state action pair.

    Defines a the cost function of the form C(s,a). This is specified as a 
    list of (p,a,c) tuples. p is a pre-condition specifying a set of states, 
    and for each s satisfying p, we have C(s,a) = c. a is an action name, and 
    c is a scalar cost recieved for that state-action pair.

    Note that for any state action pair (s,a), it should only be covered by 
    a single (p,a,c) tuple.

    To specify a state only cost, use (p,None,r), to specify an action only 
    cost, use (None,a,c) and to specify a cost for any state-action pair 
    (None, None, c).

    Additionally, we assume, that any state action pair (s,a) is only satisfies 
    one (p,a,c) tuple. If any (s,a) pair is satisfied by multiple (p,a,c) 
    tuples we assume that the costs should be summed. 

    Attributes:
        sac_tuples: A list of tuples of the form (p,a,c), where p is a 
            Condition object, and a is a string. p and a can be set to None 
            to ignore States and Actions respectively. E.g. (None,'woo',1) 
            specifies a cost of one when action 'woo' is taken irrespective 
            of the current state.
        sac_dict: The list of tuples reformatted into a dict, mapping from 
                  (p,a) to c. 
    """

    def __init__(self, 
                 sac_tuples=None):
        if sac_tuples is None:
            sac_tuples = []

        self._sac_tuples = sac_tuples
        self._sac_dict = {}
        for (p,a,c) in sac_tuples:
            self._sac_dict[(p,a)] = c

        self._index = None


    def append(self, pre_cond=None, action_name=None, cost_value=None):
        """
        Adds the (pre_cond, action_name, cost) tuple to this cost.

        Args:
            pre_cond: A Condition object, specifying a set of states for which 
                this cost is recieved when leaving it. (I.e. this specifies the 
                s in R(s,a)). To specify an action only cost, leave this 
                value as None.
            action_name: A string specifying the name of the action associated 
                with this cost. (I.e. this specifies the a in C(s,a)). To 
                specify a state only cost, set action_name to None.
            cost_value: The value of the instantaneous cost recieved for 
                          the associated state action pairs.
        """
        if cost_value is None:
            raise Exception("Cannot append something to state action cost "
                            "without specifying a cost.")

        self._sac_tuples.append((pre_cond,action_name,cost_value))
        self._sac_dict[(pre_cond,action_name)] = cost_value
        
        
    def get_cost(self, state, action):
        """Gets the cost for a specific state-action pair.

        We assume that there is only one (p,a,c) tuple in self._sac_tuples that 
        is satisfied for the given state and action. If multiple (p,a,c) tuples 
        are satisfied by the state, aciton pair then we sum the costs 'c'. 

        Args:
            state: None or a valid State object.
            action: None or a valid action name.
        Returns:
            The cost C(state, action) for the state action pair given. 
        """
        sac_tuples_to_check = self.get_sac_tuples_to_check(state,action)

        total = 0.0
        for (pre_cond, action_name, cost_val) in sac_tuples_to_check:
            if action_name is None or action == action_name:
                if pre_cond is None or pre_cond.check_cond_sat(None, state):
                    total += cost_val
        return total

    def _get_partial_state_hash_key(self, state):
        """
        Assumes that self._index_factors is not none. 
        Returns a key for self._transition_hash_table for the given state.
        """
        val_list = [state[sf.name] for sf in self._index_factors]
        return tuple(val_list)
    
    def _index_cost_(self, indexing_state_factors):
        """
        Builds an index from action names, and state factors to a list of 
        (precondition,cost) pairs that could be satisfied. 
        """
        # Keep a copy of indexing state factors, to get keys for index
        self._index_factors = indexing_state_factors

        # Compute all possible partial states formed from the indexing factors
        partial_state_dicts = [{}]
        for factor in indexing_state_factors:
            new_partial_state_dicts = []
            for value in factor:
                for pstate in partial_state_dicts:
                    new_pstate = copy.copy(pstate)
                    new_pstate[factor.name] = value
                    new_partial_state_dicts.append(new_pstate)
            partial_state_dicts = new_partial_state_dicts
        partial_states = [State(ps_dict) for ps_dict in partial_state_dicts]

        # Iterate through all partial state, action name pairs to create index
        self._index = defaultdict(lambda: defaultdict(list))
        for (pre_cond, action_name, cost_val) in self._sac_tuples:
            for pstate in partial_states:
                if pre_cond is None or pre_cond.check_partial_sat(pstate):
                    state_key = self._get_partial_state_hash_key(pstate)
                    sac_tuple = (pre_cond, action_name, cost_val)
                    self._index[action_name][state_key].append(sac_tuple)

    def get_sac_tuples_to_check(self, state, action):
        """Returns the sac_tuples that 'state' and 'action' could satisfy 
        according to the indexing. Without indexing this will just be 
        self._sac_tuples
        """
        # Need to check all tuples when no indexing
        if self._index is None:
            return self._sac_tuples

        # Need to heck all costs of the form C(a) and C(s,a) for this s,a pair
        key = self._get_partial_state_hash_key(state)
        return list(self._index[None][key]) + list(self._index[action][key])

