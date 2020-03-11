#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from state import State
from state_factor import IntegerStateFactor
from condition import EqualityCondition
from condition import LessThanCondition
from condition import GreaterThanCondition
from condition import ConjunctionCondition
from condition import DisjunctionCondition
from condition import CumulativeCondition
from transition import ProbTransition
from cost import StateActionCost
from ssp import SSP
from stringbuilder import StringBuilder




class GDST(object):
    """
    An encoding of the Generalized Deep Sea Treasure task.
    Tasks are generated as follows:
    1. the map is geneated column by column
    2. the depth at each column is at least the depth of the last plus a step 
       down, which is in the range [min_vertical_step, max_vertical_step)
    3. all depths are at least a minimum depth (enforced by setting the first 
       depth deterministically to that depth)
    4. The maximum depth is determined by the final column, and is random
    
    Attributes:
        pareto_shape: The shape of the pareto front, in {"linear", "concave", 
                      "convex", "mixed"}
        num_cols: The number of columns in the environment (how wide it is)
        transition_noise: The probability for which we take a random step in 
                          any direction. So the probability of going up 
        depths: A list of length 'num_cols', specifying the depths of the 
                treasure
        steps: A list of length 'num_cols', where steps[col] specifies the 
               minimum number of steps possible to reach the trasure in column 
               'col'
        num_rows: THe number of rows (the max depth) of the environment
        treasure: A list of length 'num_cols' containing the reward values of 
                  the treasure in each column
        max_treasure: The maximum reward for finding treasure
        reward_shift: Todo
        reward_scale: Todo
    """ 

    def __init__(
        self, 
        pareto_shape=None, 
        env_width=10, 
        min_depth=1, 
        min_drop=0, 
        max_drop=3, 
        transition_noise=0.0, 
        min_treasure=1, 
        max_treasure=1000, 
        depths_override=None, 
        treasure_override=None, 
        distance_reward_shift=0.0, 
        treasure_reward_shift=0.0,
        distance_reward_scale=1.0,
        treasure_reward_scale=1.0,
        max_steps=1000):
        """
        Args: (that are not just the attributes)
            min_drop: The minimum drop between treasure (non-strict)
            max_drop: The maximum drop between treasure (strictly less)
            min_treasure: The minimum reward for finding treasure
            depths_override: If not none, overrides the depth values
            treasure_override: If not none, overrides the treasure values
        """
        if depths_override is not None or treasure_override is not None:
            if depths_override is None and env_width != len(depths_override):
                raise Exception("Depth override and environment width inconsistent")
            if treasure_override is None and env_width != len(treasure_override):
                raise Exception("Treasure override and environment width inconsistent")

        if pareto_shape is None:
            pareto_shape = "mixed"
        
        # Things to remember
        self.pareto_shape = pareto_shape.lower()
        self.num_cols = env_width
        self.transition_noise = transition_noise
        self.max_treasure = max_treasure
        self.distance_reward_shift = distance_reward_shift
        self.treasure_reward_shift = treasure_reward_shift
        self.distance_reward_scale = distance_reward_scale
        self.treasure_reward_scale = treasure_reward_scale
        self.max_steps = max_steps

        # Compute depths and steps to reach the treasure
        if depths_override is not None:
            self.depths = depths_override
            self.steps = []
            for col in range(self.num_cols):
                self.steps.append(col + self.depths[col])
        else:
            self.depths = [min_depth]
            self.steps = [min_depth]
            drop_range = max_drop - min_drop
            for col in range(1, self.num_cols):
                drop = np.random.randint(drop_range) + min_drop
                next_depth = self.depths[-1] + drop
                self.depths.append(next_depth)
                self.steps.append(col + next_depth) # col steps right, next_depth down

        # Remember max depth
        self.num_rows = self.depths[-1] + 1

        # Compute the rewards
        if treasure_override is not None:
            self.treasure = treasure_override
            self.max_treasure = self.treasure[-1]
        else:
            self.treasure = [min_treasure] # (first reward is min treasure)
            steps_range = self.steps[-1] - self.steps[0]
            treasure_range = max_treasure - min_treasure

            for col in range(1, self.num_cols-1):
                linear_reward_ratio = (self.steps[col] - self.steps[0]) / steps_range
                adjust_factor = linear_reward_ratio * (1.0 - linear_reward_ratio)
                treasure = int(linear_reward_ratio * treasure_range + min_treasure)

                if self.pareto_shape == "convex":
                    treasure += int(adjust_factor * treasure_range)
                elif self.pareto_shape == "concave":
                    treasure -= int(adjust_factor * treasure_range)
                elif self.pareto_shape == "mixed":
                    rand = np.random.rand()*2-1
                    treasure += int(adjust_factor * treasure_range * rand)
                self.treasure.append(treasure)

            self.treasure.append(max_treasure) # (last reward is max treasure)

    
    def pretty_print(self):
        """
        Returns a pretty string describing the environment
        """
        sb = StringBuilder()
        cell_width = int(np.floor(np.log10(self.max_treasure))) + 1
        row_delimiter = '+' + (('-'*cell_width) + '+') * self.num_cols + '\n'
        for row in range(self.num_rows):
            sb.append(row_delimiter)
            for col in range(self.num_cols):
                sb.append('|')
                if row < self.depths[col]:
                    sb.append(' '*cell_width)
                elif row > self.depths[col]:
                    sb.append(' ' + 'x'*(cell_width-2) + ' ')
                else:
                    sb.append(str(self.treasure[col]).zfill(cell_width))
            sb.append('|\n')
        sb.append(row_delimiter)
        sb.append("Transition Noise: {tn}\n\n".format(tn=self.transition_noise))
        return sb.to_string()


    def to_ssp(self, add_time=False):
        """
        Returns an MDP for the deep sea treasure environment
        """
        # Three state variable, where we are, and the time
        xloc_sf = IntegerStateFactor(name='x', min=-2, max=self.num_cols)
        yloc_sf = IntegerStateFactor(name='y', min=-1, max=self.num_rows)
        if add_time:
            time_sf = IntegerStateFactor(name='t', min=0, max=self.max_steps)

        # Time progression post cond
        if add_time:
            time_prog_post_cond = CumulativeCondition(time_sf, value=1)

        # Conditions for being "in" a column (not at the top or bottom)
        in_col_conditions = []
        not_at_top_cond = GreaterThanCondition(yloc_sf, value=0)
        for col in range(self.num_cols):
            x_cond = EqualityCondition(xloc_sf, value=col)
            not_at_bottom_cond = LessThanCondition(yloc_sf, value=self.depths[col])
            col_cond = ConjunctionCondition(x_cond, not_at_top_cond, not_at_bottom_cond)
            in_col_conditions.append(col_cond)

        # Condition for being able to move in any direction
        free_space_cond = DisjunctionCondition(*(in_col_conditions[1:-1]))
        
        # Condition for being in the corners
        x_cond = EqualityCondition(xloc_sf, value=0)
        y_cond = EqualityCondition(yloc_sf, value=0)
        top_left_cond = ConjunctionCondition(x_cond, y_cond)
        
        x_cond = EqualityCondition(xloc_sf, value=self.num_cols-1)
        y_cond = EqualityCondition(yloc_sf, value=0)
        top_right_cond = ConjunctionCondition(x_cond, y_cond)

        # Condition for being in the top row
        x_gr_cond = GreaterThanCondition(xloc_sf, value=0)
        x_le_cond = LessThanCondition(xloc_sf, value=self.num_cols-1)
        y_cond = EqualityCondition(yloc_sf, value=0)
        top_row_cond = ConjunctionCondition(x_gr_cond, x_le_cond, y_cond)

        # Condition for being in any one of the treasure locations
        treasure_loc_conditions = []
        for col in range(self.num_cols):
            x_cond = EqualityCondition(xloc_sf, value=col)
            y_cond = EqualityCondition(yloc_sf, value=self.depths[col])
            at_treasure_loc_cond = ConjunctionCondition(x_cond, y_cond)
            treasure_loc_conditions.append(at_treasure_loc_cond)
        
        any_treasure_loc_cond = DisjunctionCondition(*treasure_loc_conditions)

        # Define the post conditions for moving
        left_cum_cond = CumulativeCondition(xloc_sf, value=-1)
        right_cum_cond = CumulativeCondition(xloc_sf, value=1)
        up_cum_cond = CumulativeCondition(yloc_sf, value=-1)
        down_cum_cond = CumulativeCondition(yloc_sf, value=1)

        same_cum_condx = CumulativeCondition(xloc_sf, value=0)
        same_cum_condy = CumulativeCondition(yloc_sf, value=0)
            
        if add_time:
            left_cum_cond = ConjunctionCondition(
                left_cum_cond,
                time_prog_post_cond)
            right_cum_cond = ConjunctionCondition(
                right_cum_cond,
                time_prog_post_cond)
            up_cum_cond = ConjunctionCondition(
                up_cum_cond,
                time_prog_post_cond)
            down_cum_cond = ConjunctionCondition(
                down_cum_cond,
                time_prog_post_cond)

            same_cum_condx = ConjunctionCondition(
                same_cum_condx,
                time_prog_post_cond)
            same_cum_condy = ConjunctionCondition(
                same_cum_condy,
                time_prog_post_cond)
            

        # Post cond for finishing
        x_cond = EqualityCondition(xloc_sf, value=-1)
        y_cond = EqualityCondition(yloc_sf, value=-1)
        finished_post_cond = ConjunctionCondition(x_cond, y_cond)

        transitions = []
        
        # add transitions for free space (move in all directions)
        can_move_any_cond = free_space_cond
        if add_time:
            can_move_any_cond = ConjunctionCondition(
                can_move_any_cond, 
                LessThanCondition(time_sf, value=self.max_steps))
        self._add_transitions(transitions=transitions, 
                              pre_cond=can_move_any_cond, 
                              move_left_result_cond=left_cum_cond, 
                              move_right_result_cond=right_cum_cond, 
                              move_up_result_cond=up_cum_cond, 
                              move_down_result_cond=down_cum_cond)
        
        # add transitions for left col (cant move left)
        cant_move_left_cond = in_col_conditions[0]
        if add_time:
            cant_move_left_cond = ConjunctionCondition(
                cant_move_left_cond, 
                LessThanCondition(time_sf, value=self.max_steps))
        self._add_transitions(transitions=transitions, 
                              pre_cond=cant_move_left_cond, 
                              move_left_result_cond=same_cum_condx, 
                              move_right_result_cond=right_cum_cond, 
                              move_up_result_cond=up_cum_cond, 
                              move_down_result_cond=down_cum_cond)
        
        # add transitions for right col (cant move right)
        cant_move_right_cond = in_col_conditions[-1]
        if add_time:
            cant_move_right_cond = ConjunctionCondition(
                cant_move_right_cond, 
                LessThanCondition(time_sf, value=self.max_steps))
        self._add_transitions(transitions=transitions, 
                              pre_cond=cant_move_right_cond, 
                              move_left_result_cond=left_cum_cond, 
                              move_right_result_cond=same_cum_condx, 
                              move_up_result_cond=up_cum_cond, 
                              move_down_result_cond=down_cum_cond)
        
        # add transitions for top row (cant move up)
        cant_move_up_cond = top_row_cond
        if add_time:
            cant_move_up_cond = ConjunctionCondition(
                cant_move_up_cond, 
                LessThanCondition(time_sf, value=self.max_steps))
        self._add_transitions(transitions=transitions, 
                              pre_cond=cant_move_up_cond, 
                              move_left_result_cond=left_cum_cond, 
                              move_right_result_cond=right_cum_cond, 
                              move_up_result_cond=same_cum_condy, 
                              move_down_result_cond=down_cum_cond)
        
        # add transitions for top left cornder (cant move up or left)
        cant_move_up_or_left_cond = top_left_cond
        if add_time:
            cant_move_up_or_left_cond = ConjunctionCondition(
                cant_move_up_or_left_cond, 
                LessThanCondition(time_sf, value=self.max_steps))
        self._add_transitions(transitions=transitions, 
                              pre_cond=cant_move_up_or_left_cond, 
                              move_left_result_cond=same_cum_condx, 
                              move_right_result_cond=right_cum_cond, 
                              move_up_result_cond=same_cum_condy, 
                              move_down_result_cond=down_cum_cond)
        
        # add transitions for top row (cant move up)
        cant_move_up_or_right_cond = top_right_cond
        if add_time:
            cant_move_up_or_right_cond = ConjunctionCondition(
                cant_move_up_or_right_cond, 
                LessThanCondition(time_sf, value=self.max_steps))
        self._add_transitions(transitions=transitions, 
                              pre_cond=cant_move_up_or_right_cond, 
                              move_left_result_cond=left_cum_cond, 
                              move_right_result_cond=same_cum_condx, 
                              move_up_result_cond=same_cum_condy, 
                              move_down_result_cond=down_cum_cond)

        # Add a transition for collecting a treasure
        collect_treasure_trans = ProbTransition(
            action_name="collect_bounty",
            pre_cond=any_treasure_loc_cond,
            prob_post_conds={finished_post_cond: 1.0},
        )
        transitions.append(collect_treasure_trans)

        # Add a reward for the treasure
        sar_tuples = []
        for col in range(self.num_cols):
            pre_cond = EqualityCondition(xloc_sf, value=col)
            action = "collect_bounty"
            reward_value = -self.treasure[col] * self.treasure_reward_scale
            sar_tuples.append((pre_cond, action, reward_value))

        treasure_cost = StateActionCost(
            sac_tuples=sar_tuples)

        # Add a reward for number of steps taken + if we have a positive reward 
        # for reaching the treasure, add that
        sar_tuples = [(None,  "left", self.distance_reward_scale),
                      (None, "right", self.distance_reward_scale),
                      (None,    "up", self.distance_reward_scale),
                      (None,  "down", self.distance_reward_scale),]
        distance_cost = StateActionCost(
            sac_tuples=sar_tuples)

        # Make the initial state distributions
        if add_time:
            init_state = State({'x': 0, 'y': 0, 't': 0})
        else: 
            init_state = State({'x': 0, 'y': 0})
        init_state_distr = {init_state: 1.0}
                                            
        # Make and return the MDP
        state_factors = {'x': xloc_sf, 'y': yloc_sf}
        if add_time:
            state_factors['t'] = time_sf
        return SSP(state_factors=state_factors,
                   initial_state_probs=init_state_distr,
                   transitions=transitions,
                   costs=[distance_cost, treasure_cost],                   
                   bypass_sanity_checks=True)


    def _add_transitions(self,
                         transitions, 
                         pre_cond, 
                         move_left_result_cond, 
                         move_right_result_cond, 
                         move_up_result_cond, 
                         move_down_result_cond):
        """
        Helper that makes transitions. 
        Args:
            transitions: A list of transitions we're appending to
            pre_cond: A pre_condition specifying the current location and time
            move_left_result_cond: A post condition specifying what happens if 
                                    the agent tries to move left
            move_right_result_cond: A post condition specifying what happens if 
                                    the agent tries to move right
            move_up_result_cond: A post condition specifying what happens if 
                                    the agent tries to move up
            move_down_result_cond: A post condition specifying what happens if 
                                    the agent tries to move down
        """
        left_prob_post_conds = {
            move_left_result_cond: 1.0 - self.transition_noise * 0.75,
            move_right_result_cond: self.transition_noise * 0.25,
            move_up_result_cond: self.transition_noise * 0.25,
            move_down_result_cond: self.transition_noise * 0.25,
        }
        left_trans = ProbTransition(action_name="left", 
                                    pre_cond=pre_cond, 
                                    prob_post_conds=left_prob_post_conds)

        right_prob_post_conds = {
            move_left_result_cond: self.transition_noise * 0.25,
            move_right_result_cond: 1.0 - self.transition_noise * 0.75,
            move_up_result_cond: self.transition_noise * 0.25,
            move_down_result_cond: self.transition_noise * 0.25,
        }
        right_trans = ProbTransition(action_name="right", 
                                    pre_cond=pre_cond, 
                                    prob_post_conds=right_prob_post_conds)

        up_prob_post_conds = {
            move_left_result_cond: self.transition_noise * 0.25,
            move_right_result_cond: self.transition_noise * 0.25,
            move_up_result_cond: 1.0 - self.transition_noise * 0.75,
            move_down_result_cond: self.transition_noise * 0.25,
        }
        up_trans = ProbTransition(action_name="up", 
                                    pre_cond=pre_cond, 
                                    prob_post_conds=up_prob_post_conds)

        down_prob_post_conds = {
            move_left_result_cond: self.transition_noise * 0.25,
            move_right_result_cond: self.transition_noise * 0.25,
            move_up_result_cond: self.transition_noise * 0.25,
            move_down_result_cond: 1.0 - self.transition_noise * 0.75,
        }
        down_trans = ProbTransition(action_name="down", 
                                    pre_cond=pre_cond, 
                                    prob_post_conds=down_prob_post_conds)

        transitions.append(left_trans)
        transitions.append(right_trans)
        transitions.append(up_trans)
        transitions.append(down_trans)