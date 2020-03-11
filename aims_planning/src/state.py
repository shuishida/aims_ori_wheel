#!/usr/bin/env python
# -*- coding: utf-8 -*-


import copy

from condition import Condition



def _get_key_for_state_item(state_item):
    """When we wish to sort state items (see below) to provide a hash for the 
    state, we need to provide a sortable key, to sort the items by.

    Args:
        state_item: A tuple between a state factor name and value
    Return:
        A sortable key for the state item, to use in 'sorted'.
    """
    sf_name, value = state_item
    return (sf_name, value)



class State(object):
    """A state is a mapping from state factors to values.

    A state is internally just a dictionary from state factor names
    to values that state variable can take. This class is essentially a wrapper 
    around a dictionary to make it immutable and hashable.

    Attributes:
        _state_mapping: A dictionary mapping state variable names to values
        _hash_val: Cached value for the hash of this function (as the hash 
            function is slow due to sorting etc)
    """
    def __init__(self, state_mapping):
        """Constructor initialises state mapping."""
        self._state_mapping = state_mapping
        self._hash_val = None 
    
    def get_value(self, state_factor_name):
        """Looks up state_factor in the internal dictionary."""
        return self._state_mapping[state_factor_name]

    def __getitem__(self, state_factor_name):
        """Same as self.get_value, but allows indexing notation to be used."""
        return self.get_value(state_factor_name)

    def __repr__(self):
        """Return self as string"""
        return str(self._state_mapping)

    
    def __eq__(self, other):
        """Overrides the == equality operator for State objects.

        Consider the following code example:
        # sf1 and sf2 are valid IntegerStateFactor objects
        sd = {sf1:1, sf2:2}
        s1 = State(sd)
        s2 = State(sd)
        print(s1 == s2)

        Without overriding __eq__ the code above would print "False", but we 
        obviously would like it to print "True"

        Args:
            other: A second instance of 'State' to compare 'self' to.
        """
        for key in self._state_mapping:
            if key not in other._state_mapping:
                return False
            if self._state_mapping[key] != other._state_mapping[key]:
                return False
        return True

    def __ne__(self, other):
        """Overrides the != operator for State objects. It's the negation of 
        ==, but must be overriden in Python2.7 if == is also overrided."""
        return not self.__eq__(other)

    def __hash__(self):
        """Override hash to be consistend with the overriding of __eq__. As 
        objects that are equal must have the same hash.
        
        To provide a deterministic hash, we provide a key function to be able 
        to sort the (state_variable, value) 
        """
        if self._hash_val is None:
            self._hash_val = hash(tuple(sorted(self._state_mapping.items(), 
                                               key=_get_key_for_state_item)))
        
        return self._hash_val

    def apply_post_cond(self, post_cond):
        """Applys a single single condition 'post_cond' (of type Condition) to 
        this State object to return a new State object.

        When applying a conjunction condition, the subconditions are applied in 
        'in-order' traversal order. We give an example, showing that the 
        semantics are similar to 'self.apply_post_conds':
        sv = IntegerStateFactor(...)
        eq = EqualityCondition(sf, 3)
        cumu = CumulativeCondition(sf, 1)
        state = State({sf: 10})
        conj1 = ConjunctionCondition(eq, cumu)
        new_state = state.apply_post_cond(conj1) # returns State({sf: 4})
        conj2 = ConjunctionCondition(cumu, eq)
        new_state = state.apply_post_cond(conj2) # returns State({sf: 3})
        
        Args:
            post_cond: A 'Condition' type object to apply to the state
        Returns:
            A new State object, representing the State after having applied 
            this 
        Raises:
            Exception: if the type of 'post_cond' is not yet supported.
        """
        if not isinstance(post_cond, Condition):
            raise Exception("Cannot apply post_cond that is not a 'Condition' "
                            "type")
        new_state_mapping = copy.copy(self._state_mapping)
        post_cond._apply_to_state_dict_(new_state_mapping)
        return State(new_state_mapping)

    def apply_post_conds(self, post_conds):
        """
        Deterministically applies post conditions 'post_conds'. If the post 
        conditions "overlap", and multiple of them will alter the same state 
        factors, then we assume that the conditions are to be applied one 
        after another.

        Examples of this corner case:
        sf = IntegerStateFactor(...)
        eq = EqualityCondition(sf, 3)
        cum = CumulativeCondition(sf, 1)
        state = State({sf: 10})
        new_state = state.apply_post_conds([eq, cum]) # returns State({sf: 4})
        new_state = state.apply_post_conds([cum, eq]) # returns State({sf: 3})

        Args:
            post_conds: A list of 'Condition' type objects to apply this state.
        Returns:
            A new 'State' object instance that is the result of applying these 
            post conditions 
        Raises:
            Exception: if the type of on of the 'post_conds' is not yet 
                       supported. 
        """
        new_state = self
        for post_cond in post_conds:
            new_state = new_state.apply_post_cond(post_cond)
        return new_state