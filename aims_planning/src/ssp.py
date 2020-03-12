#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ssp_impl import SSPImpl

class SSP(object):
    """ General class for a SSP.

    (Useful) Attributes:
        _state_factors: a dictionary of all state factors making up 
            the factored representation of our state space. The keys are state 
            factor names, and the value the corresponding StateFactor objects.
        _initial_state_probs: a dictionary consisting of states as keys,
            and probabilities as values, representing the distribution over 
            initial states. A state here is a dictionary from state factor 
            names to values of the state factor (i.e. the instantiation of the
            state factor).
        _index_factors: A list of StateFactor objects used to index 
            self._transitions and self._costs with. These are used internally 
            to prevent unecessary checks from being performed and therefore 
            improve performance.
        _transitions: A list of ProbTransitions (or subclasses of it). This 
            defines the probabilistic transitions between the states of the 
            NonDetModel.
        _costs: A list of StateActionCosts to represent all state-action
            costs in the MDP
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
        """ Constructor initialises attributes of SSP, by passing it to the 
        implementation class.

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
        self.ssp = SSPImpl(state_factors=state_factors,
                           initial_state_probs=initial_state_probs,
                           transitions=transitions,
                           costs=costs,
                           index_factors_names=index_factors_names,
                           bypass_sanity_checks=bypass_sanity_checks)
    
    def get_initial_state(self):
        """
        Returns the initial state (a State object) of this ssp
        """
        return self.ssp.get_deterministic_initial_state()
                           
    def get_actions(self, state=None):
        """
        Given a valid state 'state' for the SSP, returns the list of enabled 
        actions that can be used.
        """
        if state is not None:
            return self.ssp.enabled_actions(state)
        return self.ssp.get_actions()

    def get_transition_probs(self, state, action):
        """Returns all transition probabilities for a state action pair.

        Given a state action pair, this function finds the corresponding
        transition and then returns a dictionary of post conditions to floats,
        where the floats represent the probabilities of said post condition
        being satisfied.

        Returns:
            transition_probs: A dictionary mapping State objects to 
                probabilities. If transition_probs[s] = p, then 's' is the next 
                state with probability p.

        """
        return self.ssp.get_transition_probs(state=state, action=action)

    def get_cost(self, state, action):
        """
        Returns the scalar cost for taking 'action' in state 'state'.
        """
        return self.ssp.get_cost(state=state, action=action)