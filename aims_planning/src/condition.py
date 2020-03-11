#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This file contains Condition classes. A condition represents a 
boolean function over State objects, and are specified with respect to 
StateFactors.
"""

from state_factor import IntegerStateFactor

class Condition(object):
    """A condition is a boolean predicate over the values of state factors.

    An abstract class outlying what any condition class should implement 
    itself. To provide a common interface.
    """
    def __init__(self):
        pass

    def check_cond_sat(self, previous_state, state):
        """Checks if 'previous_state' and 'new_state' satisfy the condition.

        If the condition only depends on the current state, only the variable 
        'state' should be considered. 

        We assume that states are represented as dictionarys mapping from 
        StateFactor names to values. We therefore check if the state feature 
        represented by 'self.state_feature', in the new state 'state', is equal 
        to value 'self.value'. 

        Args:
            previous_state: A dictionary mapping from StateFactor names to 
                            values, representing the "previous" state of the 
                            MDP system.
            state: A dictionary mapping from StateFactor names to values, 
                   representing the current state of the MDP system. (I.e. a 
                   dictionary with StateFactor object names for keys).
        Returns:
            boolean specifying if this condition is satisfied by 
            'previous_state' and 'state'. 
        """
        raise NotImplementedError()

    def is_valid_pre_condition(self):
        """Returns a boolean specifying if this condition can be used as a 
           PRISM pre-condition."""
        raise NotImplementedError()

    def is_valid_post_condition(self):
        """Returns a boolean specifying if this condition can be used as a 
           PRISM post-condition."""
        raise NotImplementedError()

    def get_state_factors(self):
        """ Returns all state factors used within the condition as a list."""
        raise NotImplementedError()

    def check_partial_sat(self, partial_state):
        """
        This function is only valid for preconditions only, and therefore 
        should only be implemented for conditions that can be preconditions.

        Given a partial state (a State object that may be missing some state 
        factors) could form a complete state that satisfies this condition.
        I.e. this checks if there exists a completion of the 'partial_state' 
        that will satisfy this condition.

        Args:
            partial_state: A partially complete State objective
        Returns:
            If the 'partial_state' could be completed to satisfy this condition
        """
        raise NotImplementedError()

    def _apply_to_state_dict_(self, state_dict):
        """
        Passed an internal representation of a state, a dictionary mapping from 
        StateFactor object names to values, applies this post cond. 

        Subclasses should implement this only if they can be a valid post 
        condition. If the condition can be either a pre/post condition, it 
        should be treated as a post condition for this method. As it does 
        not make sense to apply a pre condition. Subclasses that are only a 
        valid pre condition should NOT implement this method.

        Note: This should NEVER be called directly. It should only be called 
        via the 'apply_post_cond' methods in state.py. 

        We use the suffixed underscore to indicate that this function has 
        side-effects on the parameter input.

        Args:
            state_dict: A dictionary mapping from StateFactor names to 
                        values, representing a state. 
        Returns:
            Nothing. It alters the state dictionary inline.
        """
        raise NotImplementedError()
    
    def range_of_values(self):
        """ Returns the range of values of the feasible state factors
        values within the condition. Returns in the form of a list of
        dictionaries from state factors to lists of ranges. This doesn't
        work for conditions which are only post conditions.
        
        If typed, this list should have a type of 
        List[Dict[StateFactor,List[Object]]]. To break this down, List[Object] 
        is a "range" of possible values for some StateFactor object. And so 
        Dict[StateFactor,List[Object]] is a mapping from StateFactor objects to
        allowable ranges. If a StateFactor (from some State object) is missing 
        from this Dict, then it is assumed that it may take any arbitrary value.
        Finally, the outermost list should be considered a union. 

        As an example, if you have two state factors X and Y, and we only want 
        to allow (0,1) and (1,0) as valid values, then the range_of_values list 
        should be:
        [
            {X: [0], Y: [1]},
            {X: [1], Y: [0]},
        ].
        """
        raise NotImplementedError()
    
    def intersect_two_lists(self, list_1, list_2):
        """ Finds intersection of ranges specified in two lists.

        This function takes two ranges of values specified as lists of
        dictionaries and finds the satisfying intersection of these.

        The lists passed in must be of a specific form. which must have the 
        same structure as what is returned from the 'range_of_values' function. 

        Each list passed into the function specifies a set of allowable states.
        And this function computes a list (with the same 
        List[Dict[StateFactor,List[Object]]] type returned by the 
        'range_of_values' function) representing the intestection of the two 
        sets of states.

        Args:
            list_1: The first list representing satisfying values
            list_2: The second list representing satisfying values 

        Returns:
            intersected: The list of dictionaries representing the
            intersection of these two ranges
        """
        intersected = []

        for i in range(len(list_1)):
            for j in range(len(list_2)):
                current_intersect = {}

                state_vars = set(list_1[i].keys()).union(set(list_2[j].keys()))

                for state_var in list(state_vars):
                    if state_var not in list_1[i]:
                        list_1_values = set(state_var.get_valid_values())
                    else:
                        list_1_values = set(list_1[i][state_var])
                    
                    if state_var not in list_2[j]:
                        list_2_values = set(state_var.get_valid_values())
                    else:
                        list_2_values = set(list_2[j][state_var])
                    

                    intersect = list_1_values.intersection(list_2_values) 

                    if len(intersect) == 0:
                        current_intersect = {}
                        break    
                    else:
                        current_intersect[state_var] = list(intersect)
                
                if len(current_intersect) != 0:
                    intersected.append(current_intersect)

        return intersected



class EqualityCondition(Condition):
    """Equality condition subclass.
    
    Represents a condition that checks if the value of 'state_factor' in a 
    state is equal to 'value'. This condition only depends on the current state.

    Attributes:
        _state_factor: A StateFactor object representing the state feature.
        _value: A value to check equality with.
    """
    def __init__(self, state_factor, value):
        super(EqualityCondition, self).__init__()
        if not state_factor.valid_value(value):
            raise Exception("Trying to set equality condition with an invalid "
                            "value for the state factor.")
        self._state_factor = state_factor
        self._value = value

    def check_cond_sat(self, previous_state, state):
        """See base class description."""
        return state.get_value(self._state_factor.name) == self._value    

    def is_valid_pre_condition(self):
        """See base class description."""
        return True

    def is_valid_post_condition(self):
        """See base class description."""
        return True
    
    def get_state_factors(self):
        """See base class description."""
        return {self._state_factor}

    def range_of_values(self):
        """See base class description."""
        return [{self._state_factor: [self._value]}]
                
    def check_partial_sat(self, partial_state):
        """See base class description."""
        if self._state_factor.name not in partial_state._state_mapping:
            return True
        return self.check_cond_sat(None, partial_state)

    def _apply_to_state_dict_(self, state_dict):
        """See base class description."""
        state_dict[self._state_factor.name] = self._value



class ConjunctionCondition(Condition):
    """A Condition subclass for conjunction conditions.

    A conjunction condition takes a list of other Condition objects, and is 
    only satisfied if all of the given conditions are satisfied.

    Attributes:
        _cond_list: A list of Condition objects making up the condjunctional 
                    condition.
    """
    def __init__(self, *conds):
        super(ConjunctionCondition, self).__init__()
        self._cond_list = []
        for cond in conds:
            self._cond_list.append(cond)

    def add_cond(self, cond):
        """Adds another Condition object to the conjunction condition.
        
        Args:
            cond: A Condition object to ass to the conjunction.
        """
        self._cond_list.append(cond) 

    def check_cond_sat(self, previous_state, state):
        """See base class description."""
        for cond in self._cond_list:
            if not cond.check_cond_sat(previous_state, state):
                return False
        return True

    def is_valid_pre_condition(self):
        """See base class description.
        
        A ConjunctionCondition is only a valid pre-condition if all of the 
        Condition objects in the conjunction are also valid pre-conditions.
        """
        for cond in self._cond_list:
            if not cond.is_valid_pre_condition():
                return False
        return True

    def is_valid_post_condition(self):
        """See base class description.
        
        A ConjunctionCondition is only a valid post-condition if all of the 
        Condition objects in the conjunction are also valid post-conditions.
        """
        for cond in self._cond_list:
            if not cond.is_valid_post_condition():
                return False
        return True
    
    def get_state_factors(self):
        """See base class description."""
        state_factors = set()
        for cond in self._cond_list:
            state_factors.update(cond.get_state_factors())
        return state_factors
        
    def check_partial_sat(self, partial_state):
        """See base class description."""
        for cond in self._cond_list:
            if not cond.check_partial_sat(partial_state):
                return False
        return True

    def _apply_to_state_dict_(self, state_dict):
        """See base class description."""
        for cond in self._cond_list:
            cond._apply_to_state_dict_(state_dict)

    def range_of_values(self):
        """See base class description."""
        ranges = self._cond_list[0].range_of_values()

        for i in range(1, len(self._cond_list)):
            ranges = self.intersect_two_lists(ranges, 
                     self._cond_list[i].range_of_values())
            # We know straight away that there is no intersection
            if ranges is []:
                return []

        return ranges



class DisjunctionCondition(Condition):
    """A Condition subclass for disjunction conditions.

    A disjunction condition takes a list of other Condition objects, and is 
    satisfied if at least one of the given conditions are satisfied.

    Attributes:
        _cond_list: A list of Condition objects making up the disjunctional 
                    condition.
    """
    def __init__(self, *conds):
        super(DisjunctionCondition, self).__init__()
        self._cond_list = []
        for cond in conds:
            self._cond_list.append(cond)

    def add_cond(self, cond):
        """Adds another Condition object to the disjunction condition.
        
        Args:
            cond: A Condition object to ass to the disjunction.
        """
        self._cond_list.append(cond)

    def check_cond_sat(self, previous_state, state):
        """See base class description."""
        for cond in self._cond_list:
            if cond.check_cond_sat(previous_state, state):
                return True
        return False
    
    def is_valid_pre_condition(self):
        """See base class description.
        
        A DisjunctionCondition is only a valid pre-condition if all of the 
        Condition objects in the disjunction are also valid pre-conditions.
        """
        for cond in self._cond_list:
            if not cond.is_valid_pre_condition():
                return False
        return True

    def is_valid_post_condition(self):
        """See base class description.
        
        A DisjunctionCondition can never be a valid post-condition
        """
        return False

    def check_partial_sat(self, partial_state):
        """See base class description."""
        for cond in self._cond_list:
            if cond.check_partial_sat(partial_state):
                return True
        return False

    def get_state_factors(self):
        """See base class description."""
        state_factors = set()
        for cond in self._cond_list:
            state_factors.update(cond.get_state_factors())
        return state_factors

    def range_of_values(self):
        """See base class description."""
        ranges = []
        for cond in self._cond_list:
            ranges += cond.range_of_values()
        return ranges



class CumulativeCondition(Condition):
    """A cumulative addition condition, specifying that a value should increase 
    by a certain value, as a result of a transition.
        
    Cumulative conditions can only be used as post conditions because they 
    represent a relation between a previous state and the next state. This is 
    because when we are checking a post-condition (at the end of a transition), 
    there is an obvious notion of "previous state" and "next state", whereas 
    in a pre-condition there is not.

    Attributes:
        _state_factor: A StateFactor object representing the state feature.
        _value: A value representing the increase required by the variable.
    """

    def __init__(self, state_factor, value):
        super(CumulativeCondition, self).__init__()
        if not isinstance(state_factor, IntegerStateFactor):
            raise Exception("Cumulative conditions may only be applied to "
                            "IntegerStateFactor's.")
        self._state_factor = state_factor
        self._value = value

    def check_cond_sat(self, previous_state, state):
        """See base class description."""
        return (previous_state.get_value(self._state_factor.name) + self._value 
                == state.get_value(self._state_factor.name))

    def is_valid_pre_condition(self):
        """See base class description."""
        return False

    def is_valid_post_condition(self):
        """See base class description."""
        return True

    def _apply_to_state_dict_(self, state_dict):
        """See base class description."""
        new_value = state_dict[self._state_factor.name] + self._value
        if not self._state_factor.valid_value(new_value):
            raise Exception("Trying to set out of range value in applying "
                            "cumulative condition")
        state_dict[self._state_factor.name] = new_value
                
    def check_partial_sat(self, partial_state):
        """See base class description."""
        if self._state_factor.name not in partial_state._state_mapping:
            return True
        val = partial_state[self._state_factor.name]
        prev_val = val - self._value
        return (self._state_factor.valid_value(prev_val))

    def get_state_factors(self):
        """See base class description."""
        return {self._state_factor}

    def range_of_values(self):
        """See base class description."""
        raise Exception('No range of values as cumulative condition ' +
                        'can only be a post-condition.')



class LessThanCondition(Condition):
    """A Condition subclass for representing inequalities.
    Inequalities can only be used as pre-conditions, because as a 
    post-condition it makes the next state ambiguous, and it's not clear what 
    it should be,
    Attributes:
        _state_factor: A StateFactor object representing the state feature.
        _value: A value for which the value of the state factor must be 
                strictly less than for the condition to be satisfied.
    """

    def __init__(self, state_factor, value):
        super(LessThanCondition, self).__init__()
        valid_factor = isinstance(state_factor, IntegerStateFactor)
        if not valid_factor:
            raise Exception("Less Than conditions may only be applied to "
                            "IntegerStateFactor's "
                            "or FloatStateFactor's")
        self._state_factor = state_factor
        self._value = value

    def check_cond_sat(self, previous_state, state):
        """See base class description."""
        return state.get_value(self._state_factor.name) < self._value

    def is_valid_pre_condition(self):
        """See base class description."""
        return True

    def is_valid_post_condition(self):
        """See base class description."""
        return False

    def get_state_factors(self):
        """See base class description."""
        return {self._state_factor}
                
    def check_partial_sat(self, partial_state):
        """See base class description."""
        if self._state_factor.name not in partial_state._state_mapping:
            return True
        return self.check_cond_sat(None, partial_state)

    def range_of_values(self):
        """See base class description."""
        num_values = self._state_factor.num_values()
        vals = []
        for i in range(num_values):
            current_val = self._state_factor.value(i)
            if current_val < self._value:
                vals.append(current_val)

        return [{self._state_factor: vals}]



class GreaterThanCondition(Condition):
    """A Condition subclass for representing inequalities.

    Inequalities can only be used as pre-conditions, because as a 
    post-condition it makes the next state ambiguous, and it's not clear what 
    it should be,

    Attributes:
        _state_factor: A StateFactor object representing the state feature.
        _value: A value for which the value of the state factor must be 
                strictly less than for the condition to be satisfied.
    """

    def __init__(self, state_factor, value):
        super(GreaterThanCondition, self).__init__()
        valid_factor = isinstance(state_factor, IntegerStateFactor)
        if not valid_factor:
            raise Exception("Greater Than conditions may only be applied to "
                            "IntegerStateFactor's "
                            "or FloatStateFactor's")
        self._state_factor = state_factor
        self._value = value

    def check_cond_sat(self, previous_state, state):
        """See base class description."""
        return state.get_value(self._state_factor.name) > self._value

    def is_valid_pre_condition(self):
        """See base class description."""
        return True

    def is_valid_post_condition(self):
        """See base class description."""
        return False

    def get_state_factors(self):
        """See base class description."""
        return {self._state_factor}
                
    def check_partial_sat(self, partial_state):
        """See base class description."""
        if self._state_factor.name not in partial_state._state_mapping:
            return True
        return self.check_cond_sat(None, partial_state)

    def range_of_values(self):
        """See base class description."""
        num_values = self._state_factor.num_values()
        vals = []
        for i in range(num_values):
            current_val = self._state_factor.value(i)
            if current_val < self._value:
                vals.append(current_val)

        return [{self._state_factor: vals}]