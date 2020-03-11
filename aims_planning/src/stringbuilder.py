#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""A wrapper around pythons 'array' object, to provide fast string building 
while maintaining a sane interface.
Authors: Michael Painter
Owner: Michael Painter
"""




import sys
if sys.version_info[0] == 2:
    from cStringIO import StringIO
else:
    from io import StringIO





class StringBuilder(object):
    """Python has a lack of good string building objects. So here's a custom
    StringBuilder object.

    Attributes:
        _arr: An internal array object that we're using for the string building.
        _is_empty: Boolean keeping track of if the array is empty or not.
    """
    def __init__(self):
        self._str = StringIO()
        self._is_empty = True

    def append(self, string, with_line_break = False):
        """Appends a string in the String Builder.
        Args:
            string: The string to be appended
            with_line_break: If True then a '\n' is appended to the end of
            the string
        """
        self._is_empty = (self._is_empty and string == '')
        self._str.write(string)
        if with_line_break:
            self._str.write('\n')

    def is_empty(self):
        """Returns if the string builder is currently empty."""
        return self._is_empty

    def to_string(self):
        """Return's the built string in the string builder."""
        return self._str.getvalue()