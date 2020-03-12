#!/usr/bin/env python
# -*- coding: utf-8 -*-


from numbers import Number


class StateFactor(object):
    """A state factor to be used for creating factored states.

    This state factor takes an arbitrary list of python objects as values, as 
    long as they are sortable. Typically we will use strings for these values.

    Attributes:
        name: A name for this state factor in the factored state representation.
        _range: A (sorted) list of values that the state can take as a value.
    """
    def __init__(self, name, values):
        self.name = name
        self._range = sorted(values)

    def __repr__(self):
        """Return self as string"""
        return str(self.name)

    def get_valid_values(self):
        """Returns a list of all valid possible values for this state factor."""
        return set(self._range)

    def valid_value(self, value):
        """Checks if 'value' is a valid assignment to this state factor."""
        return value in self._range

    def value(self, index):
        """Returns the 'index'th value that the state factor can take."""
        if not (0 <= index and index < len(self._range)):
            raise Exception("Index out of range for state factor.")
        return self._range[index]

    def num_values(self):
        """Returns the number of values the state factor can take."""
        return len(self._range)

    def index(self, state_value):
        """Returns the index of 'state_value' in self._range."""
        if not self.valid_value(state_value):
            raise Exception("Value not in state factor's allowed values.")
        return self._range.index(state_value)

    def get_values(self):
        """Returns all values this state factor can take."""
        return self._range

    def __iter__(self):
        """Exposes iterator of underlying value list"""
        return self._range.__iter__()

    def next(self):
        """Exposes next operator for iterator of underlying value list"""
        return self._range.next()


class IntegerStateFactor(StateFactor):
    """A state factor that takes integer values.

    As we have an integer factor, it's range can be defined by a minimum and 
    maximum value. That is the integer can take any integer value in the closed 
    interval [min, max].

    Attributes:
        name: A name for this state factor in the factored state representation.
        _range: A tuple defining the minimum and maximum 
        _min: The minimium value that this state factor can take
        _min: The maximum value that this state factor can take
    """

    def __init__(self, name, min, max):
        super(IntegerStateFactor, self).__init__(name=name, values=(min,max))
        self._min = min
        self._max = max 

    def get_valid_values(self):
        """Returns a list of all valid possible values for this state factor."""
        return set(range(self._min, self._max+1))

    def valid_value(self, value):
        """Checks if 'value' is a valid assignment to this state factor."""
        if not isinstance(value, Number): 
            return False
        return (self._min <= value and value <= self._max)

    def value(self, index):
        """Returns the 'index'th value that the state factor can take."""
        if not (0 <= index and index <= self._max - self._min):
            raise Exception("Index out of range for state factor.")
        return self._min + index

    def num_values(self):
        """Returns number of integers state factor can take."""
        return self._max - self._min + 1

    def index(self, state_value):
        """Returns the index of 'state_value' in self._range."""
        if not self.valid_value(state_value):
            raise Exception("Value not in state factor's allowed values.")
        return state_value - self._min

    def get_values(self):
        """See base class description."""
        return list(range(self._min,self._max+1))

    def __iter__(self):
        """See base class description."""
        return iter(range(self._min,self._max+1))

    def set_min(self, min):
        """Sets minimum value to min"""
        self._min = min

    def set_max(self, max):
        """Sets maximum value to max"""
        self._max = max