#!/usr/bin/env python

import numpy as np

def get_removal_time(name):
    """ defines the removal time for each size of rubble pile """

    if 'small' in name:
        return 10
    elif 'medium' in name:
        return 20
    elif 'large' in name:
        return 45

def get_door_prob():
    """ returns the probability that any given door is closed """
    return 0.5

def get_time_lim():
    """ returns the maximum time before sim times out """
    return 5000

def get_num_targets():
    """ returns the known number of targets """
    return 2

def get_starting_loc():
    """ returns the location where the robot starts and ends each episode. """

    return {'min_x': -1., 'max_x':1., 'min_y':-1., 'max_y':1.}

def get_points():
    """ defines the points for viewing each target, and for each second over
    the time limit. """
    return {'mission_complete':500, 'per_second':-1}

def get_rubble_locations():
    """ possible locations to spawn doors into """

    locs = [
            {'x':1.55, 'y':2.15, 'angle':-0.3, 'loc':1, 'probs':{None:0.3, 'small':0.4, 'medium':0., 'large':0.3}},
            {'x':-1.5, 'y':1.75, 'angle':0.2, 'loc':2, 'probs':{None:0.3, 'small':0.4, 'medium':0., 'large':0.3}},
            {'x':-0.9, 'y':3.7, 'angle':1.67, 'loc':3, 'probs':{None:0.3, 'small':0.4, 'medium':0., 'large':0.3}},
            {'x':1., 'y':3.1, 'angle':1.47, 'loc':4, 'probs':{None:0.3, 'small':0.4, 'medium':0., 'large':0.3}}
            ]
    return locs

def get_robot_location():
    """ defines the initial pose of the robot when sim is reset """

    return {'x':0., 'y':-2., 'angle':1.57}

def get_cylinder_loc():
    """ returns location for cylinder """

    x = 0 + np.random.uniform(-0.1, 0.1)
    y = 1.95 + np.random.uniform(-0.05, 0.05)
    return {'x': x, 'y': y}

def get_target_locations():
    locs = [
            {'x':1.4, 'y':1., 'angle':1.85, 'model':'person1'},
            {'x':-1.4, 'y':0.75, 'angle':3.14, 'model':'person2'},
            {'x':-1.6, 'y':3.6, 'angle':3.14, 'model':'person3'},
            {'x':3.2, 'y':3.1, 'angle':3.14, 'model':'person4'}
            ]
    return locs

def sample_rubble(all_rubble):
    """ Returns a new list with each item in original list included with
    given probability. """

    samples = []
    for rubble in all_rubble:
        size = np.random.choice(
                    rubble['probs'].keys(), p = rubble['probs'].values())
        if size is not None:
            rubble['size'] = size
            samples.append(rubble)
    return samples

def get_new_config(seed):
    """ Randomly sample a new configuration of doors and targets """

    np.random.seed(seed)

    # random door locations
    all_rubble = get_rubble_locations()
    rubble_sample = sample_rubble(all_rubble)

    # cylinder location
    cylinder_loc = get_cylinder_loc()

    # random target locations
    num_targs = get_num_targets()
    all_targets = get_target_locations()
    target_sample = np.random.choice(all_targets, num_targs, replace = False)

    # get robot starting location
    robot_loc = get_robot_location()

    return rubble_sample, target_sample, robot_loc, cylinder_loc
