#!/usr/bin/env python
# -*- coding: utf-8 -*-

from condition import Condition

class ProbTransition(object):
    """ Probabilistic transitions for non-deterministic Markov Models.

    Defines a conditional transition to be used in SSP. It specifies a 
    pre-condition 'pre_cond' that need to hold for the transition to be 
    "enabled". It also specifies a set of post conditions and 
    the probabilities that they hold after a transition. 

    Attributes:
        action_name: A (human readable) action name corresponding to this 
            transition.
        pre_cond: A Condition object defining a condition that must be 
            satisfied before taking this transition.
        prob_post_conds: A dictionary mapping from Condition objects to 
            probabilities (float). If prob_post_conds[k] == p, then this means 
            that Condition 'k' will be satisfied with probability 'p' after the 
            transition. In reality when this transition is used in, we sample 
            a post cond to update a state with the appropriate probabilities.
    """
    def __init__(self, 
                 action_name=None,
                 pre_cond=None,
                 prob_post_conds=None):
        if action_name is None or pre_cond is None or prob_post_conds is None:
            raise Exception("ProbTransitions must be fully specified.")
        self.action_name=action_name
        self.pre_cond=pre_cond
        self.prob_post_conds=prob_post_conds
        self._prune_post_conds_()


    def _prune_post_conds_(self):
        """ Prunes any prob post conds that would occur with an invalid 
        probability.

        This is useful in more complicated MDPs when it is easy to accidentally 
        create prob post conds that occur with probability zero. 
        """
        new_prob_post_conds = {}
        for post_cond, prob in self.prob_post_conds.items():
            if prob <= 0.0 or prob > 1.0:
                continue
            new_prob_post_conds[post_cond] = prob
        self.prob_post_conds = new_prob_post_conds

