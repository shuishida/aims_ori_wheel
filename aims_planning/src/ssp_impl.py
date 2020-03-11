#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import numpy as np

from collections import defaultdict
from numbers import Number

from condition import Condition
from condition import ConjunctionCondition
from condition import EqualityCondition
from cost import StateActionCost
from state import State
from state_factor import StateFactor
from transition import ProbTransition

class SSPImpl(object):
    """ General class for a Markov Decision Process.

    This class contains the general structure and functionality
    for a Markov Decision Process.

    Attributes:
        _state_factors: a dictionary of all state factors making up
                        the factored representation of our state space.
                        The keys are state factor names, and the value
                        the corresponding StateFactor object.
        _initial_state_probs: a dictionary consisting of states as keys,
                            and probabilities as values, representing the
                            distribution over initial states. A state here
                            is a dictionary from state factor names to values of
                            the state factor (i.e. the instantiation of the
                            state factor).
        _index_factors: A list of StateFacotr objects used to index
            self._transitions and self._costs with
        _transitions: A list of ProbTransitions (or subclasses of it). This
            defines the probabilistic transitions between the states of the
            NonDetModel.
        _transition_hash_table: A dictionary mapping from possible values of
            factors in self._index_factors to ProbTransition objects.
        _costs: A list of StateActionCosts to represent all state-action
                costs in the MDP
        _cost_hash_table: A dictionary mapping from possible values of
            factors in self._index_factors to StateActionCost objects.
        _bypass_sanity: If we have turned off sanity checking for this MDP
                        instance.
    """

    def __init__(self,
                 state_factors,
                 initial_state_probs,
                 transitions,
                 costs,
                 index_factors_names=None,
                 bypass_sanity_checks=False):
        """ Constructor initialises attributes of MDP.

        This constructor calls the super constructor in Model and initialises
        all new attributes.

        Args:
            state_factors: The dictionary of state factor names to State Factor
                objects defining the factored state space of the NonDetModel.
            initial_state_probs: A dictionary from instantiated states to
                probabilities.
            transitions: A definition of the transitions for each state-action
                pair. Given as a list of ProbTransitions.
            costs: A list of StateActionCosts representing all the
                state-action costs in the NonDetModel.
            index_factors_names: A subset of state factor names, that should be
                used to index costs/transitions. (So then significantly fewer
                preconditions need to be checked)
            bypass_sanity_checks: Specifies the ability to turn off sanity
                checks in this NonDetModel instance.
        """

        self._bypass_sanity = bypass_sanity_checks

        # Sanity Check State Factors
        self._check_valid_state_factor_dict(state_factors)
        self._state_factors = state_factors

        # Set and sanity Check Initial State Probabilities
        self._initial_state_probs = None
        self.set_initial_state(initial_state_probs)

        # Sanity Check List of ProbTransitions
        if not self._bypass_sanity:
            self._check_valid_transition_list(transitions)
        self._transitions = transitions

        # If index_factors is not None, then check valid and construct indices
        self._transition_hash_table = None
        self._cost_hash_table = None
        if index_factors_names is not None:
            # if not self._bypass_sanity:
            #     self._check_valid_index_factors(index_factors_names)
            self._index_factors = [state_factors[nm]
                                   for nm in index_factors_names]
            self._index_transitions_(transitions, self._index_factors)
            self._index_costs_(costs, self._index_factors)

        # Sanity Check List of costs
        self._check_valid_cost_list(costs)
        self._costs = costs


    def add_state_factor(self, factor):
        """Adds a new state factor to this Model.

        Args:
            factor: the factor to add
        """
        self._state_factors.append(factor)


    def get_num_state_factors(self):
        """ Gets number of state factors.

        This function returns the number of state factors in our factored
        state space representation of the Model.

        Return:
            num_state_factors: The number of state factors in
                               the factored Model.

        """
        return len(self._state_factors)


    def get_state_factors(self):
        """ Gets all state factor information.

        This function returns the dict of factor names to StateFactor objects.

        Return:
            factors: The list of state factors used in the factored Model.
        """
        return self._state_factors


    def has_deterministic_initial_state(self):
        """ Function checks if Model has deterministic initial state.

        This function determines whether the Model has been initialised with
        a deterministic initial state.

        Returns:
            is_deterministic: A boolean stating whether the initial state
                            is deterministic or not

        """
        return len(self._initial_state_probs) == 1


    def set_determinstic_initial_state(self, state):
        """ Sets deterministic Model initial state.

        Function sets the Model initial state deterministically
        rather than probabilistically. This gives us a dictionary
        of length 1.

        Args:
            state: The state (dictionary from state factor names to values)
                to set as the deterministic initial state.

        Raises:
            invalid_state: Raised when an invalid state is passed in

        """
        if not self._is_valid_state(state):
            raise Exception('Invalid State Passed in')

        self._initial_state_probs = {state: 1.0}


    def get_deterministic_initial_state(self):
        """ Returns deterministic initial state of Model.

        This function returns the deterministic initial state of the Model,
        if such a state exists.

        Returns:
            initial_state: The deterministic initial state, if it exists.

        Raises:
            not_deterministic: Raised if Model doesn't have a
                            deterministic initial state

        """
        if self.has_deterministic_initial_state():
            return list(self._initial_state_probs.keys())[0]
        else:
            raise Exception("""The Model doesn't have a
                            deterministic initial state!""")


    def set_initial_state(self, state_prob_dict):
        """ Sets initial state of Model.

        This function sets the distribution over the initial Model states.
        If this is found to be a point distribution, then the distribution
        is reduced to a single deterministic initial state with probability 1.

        Args:
            state_prob_dict: The dictionary from states to initial probabilities.
                            A state is in the form of a dictionary from
                            state factors to values

        Raises:
            Invalid_distribution: Raised if dictionary doesn't give a valid
                                probability distribution

        """

        # Allows a reduced model internally, without effecting parameters
        reduced_state_prob_dict = state_prob_dict.copy()

        sum = 0.0

        for state in state_prob_dict:
            assert self._is_valid_state(state), ("Invalid state featured " +
                                                 "in state_prob_dict.")

            sum += reduced_state_prob_dict[state]

            # If 0 probability, this is redundant in the dictionary
            if reduced_state_prob_dict[state] == 0.0:
                del reduced_state_prob_dict[state]

        if not np.isclose(sum, 1.0):
            raise Exception('Invalid initial distribution given.')

        self._initial_state_probs = reduced_state_prob_dict



    def get_initial_state(self):
        """ Gets general initial state distribution.

        This function returns the initial state in its general form
        as a dictionary, regardless of whether it is deterministic or not

        Returns:
            initial_state_probs: The distribution over the initial state,
                                in the form of a dictionary from states to
                                probabilities.

        """
        return self._initial_state_probs


    def get_num_actions(self):
        """ Returns number of actions.

        This function returns the number of actions in the NonDetModel.

        Returns:
            num_actions: The number of actions in the NonDetModel.

        """
        return len(self.get_actions())


    def get_actions(self):
        """ Returns a list of all action names in the SSP.

        This function returns the names of all available actions within the
        NonDetModel.
        
        Returns:
            actions: All of the action names as a list
        """
        actions = set() # Avoiding duplicates
        for transition in self._transitions:
            actions.add(transition.action_name)

        return list(actions)


    def enabled_actions(self, state):
        """ Returns all action names enabled in the state.

        This function looks through all transitions to return
        the actions that are available at a particular state.
        Args:
            state: The state we are considering actions from.
        Returns:
            enabled_actions: A list of actions enabled at that state
        
        Raises:
            invalid_state: Raised if state input is invalid
        
        """
        if not self._is_valid_state(state):
            raise Exception('Invalid State Passed in.')

        enabled_actions = set()

        # Get list of ProbTransitions to checks
        transitions_to_check = self._get_transitions_to_check_for_state(state)

        for transition in transitions_to_check:
            if transition.pre_cond.check_cond_sat(None, state):
                enabled_actions.add(transition.action_name)

        return list(enabled_actions)


    def get_transitions(self):
        """ Gets all transitions from this NonDetModel.

        Returns:
            The list of transitions of this MDP
        """
        return self._transitions


    def get_transition_probs(self, state, action):
        """ Returns all transition probabilities for a state action pair.

        Given a state action pair, this function finds the corresponding
        transition and then returns a dictionary of post conditions to floats,
        where the floats represent the probabilities of said post condition
        being satisfied.

        Args:
            state: The state at the start of the state action pair
            action: The action we are considering executing
        
        Returns:
            state_probs: A dictionary of states mapped to floats where the 
                floats are probabilities. None returned if state action pair 
                isn't valid for any pre condition (or it doesn't exist)
        
        Raises:
            invalid_state: Raised if the state input is invalid
        """

        if not self._is_valid_state(state):
            raise Exception('Invalid State Passed in.')

        # Get list of ProbTransitions to checks
        transitions_to_check = self._get_transitions_to_check_for_state(state)

        # Find prob post conds from transition
        prob_post_conds = None
        for transition in transitions_to_check:
            # Action check will be quicker than the state one
            if transition.action_name == action:
                if transition.pre_cond.check_cond_sat(None, state):
                    prob_post_conds = transition.prob_post_conds
                    break

        # If state action pair not found
        if prob_post_conds is None:
            return None

        # Apply post conds to get a mapping from states to probabilities
        state_probs = defaultdict(float)
        for post_cond, prob in prob_post_conds.items():
            next_state = state.apply_post_cond(post_cond)
            state_probs[next_state] += prob
        return state_probs


    def get_transition_prob(self, state, action, next_state):
        """ Returns the transition probability T(s,a,s').

        This function returns the transition probability for a state
        action pair going to a particular next state. If it doesn't
        exist, the probability will be 0.

        Args:
            state: The first or previous state
            action: The action we are considering executing
            next_state: The next state that results from executing action
        
        Returns:
            prob: The transition probability T(state,action,next_state)
                  If the transition doesn't exist, 0 is returned.
        Raises:
            invalid_state: Raised if state or next_state is invalid
        """

        if not (self._is_valid_state(state) and
                self._is_valid_state(next_state)):
            raise Exception('Invalid State Passed in.')

        prob = 0.0

        for transition in self._transitions:
            if transition.action_name == action:
                if transition.pre_cond.check_cond_sat(None, state):
                    for post_cond in transition.prob_post_conds:
                        if self._check_post_cond_with_states(state,
                                                             post_cond,
                                                             next_state):
                            prob +=  transition.prob_post_conds[post_cond]

        return prob


    def add_cost(self, cost):
        """Adds a cost structure to this NonDetModel

        Args:
            cost: The cost structure to be added
        """
        self._costs.append(cost)


    def get_costs(self):
        """ Returns all of the costs in the SSP.

        This function returns the list of StateActioncost objects
        used in the NonDetModel.

        Returns:
            costs: The StateAction costs in the NonDetModel
        """
        return self._costs


    def get_cost(self, state, action):
        """ Gets cost for state action pair.

        If a state action pair has a cost associated with it,
        this function returns it. We return a dictionary of cost objects to
        the costs recieved for that cost if we have multi-objective costs.

        If not, we sum costs recieved from all of the costs in
        self._costs.

        Args:
            state: The state of the NonDetModel we are considering
            action: The action_name we are considering
            multi_objective: Set to true if we want to consider multi-objective
                             costs.

        Returns:
            cost_value: We return the sum of all of the costs from all of the
                StateActionCost objects in self._costs. 

        Raises:
            invalid_state: Raised if an invalid state is passed in

        """
        costs = {}
        for cost in self._costs:
            costs[cost] = cost.get_cost(state, action)

        total = 0.0
        for val in costs.values():
            total += val
        return total



    def _check_post_cond_with_states(self,
                                     previous_state,
                                     post_cond,
                                     next_state):
        """ Checks next_state is a valid successor.

        This function checks that, given a post_cond,
        next_state satisfies the condition, and for all
        state factorss not touched in post_cond, it is the
        same as in previous_state.

        Args:
            previous_state: The previous state of the MDP
            post_cond: The effect of the current action
            next_state: A successor state

        Returns:
            is_valid_successor: True if the checks pass, False otherwise

        """
        if post_cond.check_cond_sat(None, next_state):
            post_cond_factors = post_cond.get_state_factors()
            state_factors = set(self._state_factors.values())
            factors_to_check = list(state_factors.difference(post_cond_factors))
            for factor in factors_to_check:
                if (previous_state[factor.name] != next_state[factor.name]):
                    return False
            return True
        return False


    def _enumerate_states(self):
        """ Enumerates states of a factored Model to enable flattening.

        This function enumerates all states of a factored Model, such that
        it can be used to create a mapping to flat states.

        The function enumerates states by maintaining a list of indexes into
        each state factor. We then loop through these from 'right' to 'left'
        until we have systematically enumerated all states. For example
        if we had state factors s with range [0,1] and x with range [0,1,2] we
        would iterate in the following order: [0,0], [0,1], [0,2], [1,0],
        [1,1], [1,2].

        Returns:
                states: A list of States
        """
        if self._state_factors == {}:
            return []

        state_index = [0] * len(self._state_factors)
        states = []
        state_factor_values = [(f, self._state_factors[f].get_values())
                               for f in self._state_factors]

        while state_index[0] < len(state_factor_values[0][1]):
            current_state = {}
            for i in range(len(state_index)):
                val = state_factor_values[i][1][state_index[i]]
                current_state[state_factor_values[i][0]] = val

            current_state = State(current_state)
            assert self._is_valid_state(current_state)
            states.append(current_state)

            # Update the state index
            for j in range(len(state_index) - 1, -1, -1):
                state_index[j] += 1
                if state_index[j] == len(state_factor_values[j][1]):
                    # Terminating condition
                    if j == 0:
                        break
                    state_index[j] = 0
                else:
                    # If we are at an index which doesn't loop, we can stop
                    break

        return states

    def _state_to_condition(self, state):
        """Converts state to equality Conditions.

        This function takes a State object and converts it into
        a Conjunction condition (if >1 state factor), with each
        sub-condition being an equality condition over one of the state factors.
        NOTE: If there is only one state factor, an Equality condition is
        returned.

        Args:
            state: The state to convert

        Returns:
            cond: The condition
        """
        cond = ConjunctionCondition()

        for factor in self._state_factors:
            eq = EqualityCondition(self._state_factors[factor],
                                   state[factor])
            cond.add_cond(eq)

        if len(cond._cond_list) == 1:
            return cond._cond_list[0]

        return cond


    def _is_valid_state(self, state):
        """ Function checks validity of state.

        This function checks the validity of the state. It does this by
        firstly checking that all state factors are included within the state
        and then that the instantiation of said state factors is one of the
        valid values for that state factor.

        Args:
            state: The state of the Model, represented as a state object

        Return:
            valid_state: True if the state is valid, and False otherwise
        """
        if self._bypass_sanity:
            return True

        if not isinstance(state, State):
            return False

        try:
            for factor in self._state_factors:
                # Will get exception if state factor not in state
                value = state[factor]

                if not self._state_factors[factor].valid_value(value):
                    return False

                # Will get exception if not in values for state factor
                self._state_factors[factor].index(value)
        except:
            return False

        return True


    def _is_valid_state_factor(self, state_factor):
        """ Carries out basic sanity checks on state factor.

        This function carries out tests on state factors to check it is usable.
        This includes:
        * state_factor is an instance of StateFactor (subclasses are fine)
        * It has a non_empty name
        * The number of values it may take is non-empty

        Args:
            state_factor: The state factor to check

        Returns:
            valid_state_factor: True if the state factor
                                is valid, False otherwise
        """
        if self._bypass_sanity:
            return True

        # If this isn't true we need to stop immediately
        if not isinstance(state_factor, StateFactor):
            return False

        if (not isinstance(state_factor.name, str) or
           (state_factor.name == '' or
            state_factor.num_values() == 0)):

            return False

        return True


    def _check_valid_state_factor_dict(self, state_factors):
        """ Function checks all state factors in dict are valid.

        As well as checking each state factor is valid, it also ensures
        that we don't have state factors with duplicate names, as this will
        cause issues when representing states. It also ensures we have at least
        one state factor, as well as checking names match to objects.

        Args:
            state_factor: A list of StateFactor objects

        Raises:
            duplicate_state_factor: Raised if multiple
                                    state factors have the same name
            invalid_state_factor: Raised if a state factor is invalid
            no_state_factors: Raised if no state factors are provided
            no_match: Raised if state factor name doesn't match object

        """
        if self._bypass_sanity:
            return

        if len(state_factors) == 0:
            raise Exception('No State Factors Provided.')

        state_factor_names = []
        for factor in state_factors:
            if not self._is_valid_state_factor(state_factors[factor]):
                raise Exception("""Invalid State Factors Given.
                                   Ensure they have a non-empty name
                                   and at least one possible value.""")
            if factor in state_factor_names:
                raise Exception('Duplicate State Factor Names Present.')
            if factor != state_factors[factor].name:
                raise Exception("Factor Name Doesn't match StateFactor Object.")
            state_factor_names.append(factor)


    def _is_valid_transition(self, transition):
        """ Function checks validity of transition.
        
        The checks carried out here are:
        * non-empty action name (we need something descriptive)
        * pre-cond is a valid pre-condition
        * all post-conds are valid post-conditions
        * post-cond distribution sums to 1 if applicable
        Args:
            transition: A transition object
        
        Returns:
            valid_transition: True if transition is valid, False otherwise
        """

        if not isinstance(transition, ProbTransition):
            return False

        if not (isinstance(transition.action_name, str)
                and transition.action_name != ''):
            return False

        if not isinstance(transition.pre_cond, Condition):
            return False

        if not transition.pre_cond.is_valid_pre_condition():
            return False

        if not isinstance(transition.prob_post_conds, dict):
            return False

        sum_probs = 0.0
        for post_cond in transition.prob_post_conds:
            if not isinstance(post_cond, Condition):
                return False
            if not post_cond.is_valid_post_condition():
                return False

            sum_probs += transition.prob_post_conds[post_cond]

        if not np.isclose(sum_probs, 1.0):
            return False

        return True


    def _check_valid_transition_list(self, transitions):
        """ Checks all transitions in a list are valid.

        This function iterates _is_valid_transition
        over a list of transitions. Additionally,
        it ensures that for a particular action, all
        pre conditions are disjoint.

        Args:
            transitions: A list of ProbTransition objects

        Raises:
            invalid_transition: Raise if an invalid transition is found
            overlapping_preconds: Raised if there are overlapping
                                  preconditions for an action.

        """
        preconds_for_action = {}

        for transition in transitions:
            if not self._is_valid_transition(transition):
                raise Exception('Invalid Transition Found')

            action_name = transition.action_name
            if not (action_name in preconds_for_action.keys()):
                # If we haven't seen the action before, create a list for it
                preconds_for_action[action_name] = [transition.pre_cond]
            else:
                # If we've seen the action before, append it to our list
                preconds_for_action[action_name].append(transition.pre_cond)

        for action in preconds_for_action:
            pre_conds = preconds_for_action[action]
            for i in range(len(pre_conds)):
                for j in range(i+1,len(pre_conds)):
                    ranges_one = pre_conds[i].range_of_values()
                    ranges_two = pre_conds[j].range_of_values()

                    intersection = pre_conds[i].intersect_two_lists(ranges_one,
                                                                    ranges_two)

                    if len(intersection) != 0:
                        raise Exception("""Overlapping preconditions
                                        for same action present.""")


    def _get_transition_hash_key(self, state):
        """
        Assumes that self._index_factors is not none.
        Returns a key for self._transition_hash_table for the given state.
        """
        val_list = [state[sf.name] for sf in self._index_factors]
        return tuple(val_list)


    def _index_transitions_(self, transitions, transition_index_factors):
        """
        Indexes transitions with transition_index_factors, and stores the
        resulting dictionary in self._transition_hash_table
        Args:
            transitions: A list of ProbTransition objects in this model
            transition_index_factors: A list of StateFactors to index the
                         transitions by
        Returns:
            Nothing. It will create self._transition_hash_table
        """
        # compute all partial states formed from the factors indexing by
        partial_state_dicts = [{}]
        for factor in transition_index_factors:
            new_partial_state_dicts = []
            for value in factor:
                for pstate in partial_state_dicts:
                    new_pstate = copy.copy(pstate)
                    new_pstate[factor.name] = value
                    new_partial_state_dicts.append(new_pstate)
            partial_state_dicts = new_partial_state_dicts
        partial_states = [State(ps_dict) for ps_dict in partial_state_dicts]

        # For each partial state, check which transitions could be satisfied
        hash_table = {}
        for pstate in partial_states:
            potential_transitions = []
            for t in transitions:
                if t.pre_cond.check_partial_sat(pstate):
                    potential_transitions.append(t)

            key_for_pstate = self._get_transition_hash_key(pstate)
            hash_table[key_for_pstate] = tuple(potential_transitions)

        # Save the computed hash table for efficient lookups later
        self._transition_hash_table = hash_table


    def _index_costs_(self, costs, index_factors):
        """Passed the StateFactor objects to index by onto the costs so they
        can build their indices
        """
        for cost in costs:
            cost._index_cost_(index_factors)


    def _get_transitions_to_check_for_state(self, state):
        """Returns the transitions that 'state' could satisfy. Without indexing
        this will just be self._transitions"""
        if self._transition_hash_table is None:
            return self._transitions

        key = self._get_transition_hash_key(state)
        transitions_to_check = self._transition_hash_table[key]
        return list(transitions_to_check)


    def _is_valid_cost(self, cost):
        """ Function checks validity of state action cost object.

        This function carries out the following checks:
        * costs is a StateActionCost
        * cost_value is a number
        * pre_cond is a condition
        * action_name is a valid string and not empty
        * cost-type is either None or a non-empty string

        Args:
            cost: The cost object to test

        Returns:
            valid_cost: True if the cost is valid, False otherwise

        """
        if self._bypass_sanity:
            return True

        if not isinstance(cost, StateActionCost):
            return False

        for pre_cond, action_name, cost_val in cost._sac_tuples:
            if pre_cond is not None and not isinstance(pre_cond, Condition):
                return False

            if action_name is not None and not isinstance(action_name, str):
                return False
            if action_name is not None and action_name == '':
                return False

            if not isinstance(cost_val, Number):
                return False

        return True


    def _check_valid_cost_list(self, costs):
        """ Checks all costs in a list are valid.

        This function simply iterates _is_valid_cost
        over a list of transitions.

        Args:
            costs: A list of StateActionCost objects

        Raises:
            invalid_cost: Raise if an invalid cost is found

        """
        if self._bypass_sanity:
            return

        for cost in costs:
            if not self._is_valid_cost(cost):
                raise Exception('Invalid cost Found in List of costs')
