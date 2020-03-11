#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

from state import State
from state_factor import StateFactor
from state_factor import IntegerStateFactor
from condition import EqualityCondition
from condition import ConjunctionCondition
from condition import CumulativeCondition
from condition import DisjunctionCondition
from condition import GreaterThanCondition
from condition import LessThanCondition
from transition import ProbTransition
from cost import StateActionCost
from ssp import SSP

from strands_navigation_msgs.srv import GetTaggedNodes
from strands_navigation_msgs.msg import TopologicalMap
import rospy



# Constants including probabilities and costs that appear in the ssp
NUM_PEOPLE = 2
TIME_HORIZON = 5000

RUBBLE_PILE_CLEAR_PROB = 0.3
LARGE_RUBBLE_PILE_PROB = 0.3

EDGE_TIME_COST_TO_LEN_RATIO = 100.0
LARGE_RUBBLE_PILE_CLEARING_TIME_COST = 20
SMALL_RUBBLE_PILE_CLEARING_TIME_COST = 5
CHECK_FOR_PERSON_TIME_COST = 5



def get_rubble_clearing_time_cost(large=False):
    """
    Exposes rubble clearing cost for policy executor
    """
    if large:
        return LARGE_RUBBLE_PILE_CLEARING_TIME_COST 
    return SMALL_RUBBLE_PILE_CLEARING_TIME_COST

def get_check_for_person_time_cost():
    """
    Exposes cost for checking for a person for policy executor
    """
    return CHECK_FOR_PERSON_TIME_COST

def get_length_time_cost():
    """
    Exposes 
    """
    return EDGE_TIME_COST_TO_LEN_RATIO



class N(object):
    """
    A basic node object, used to abstract away from topological nodes which are 
    'overkill' for constructing this SSP
    """
    def __init__(self, x, y, q, name):
        self.x = x
        self.y = y
        self.q = q
        self.name = name

class E(object):
    """
    A basic edge object, used to abstract away from topological edges which are 
    also overkill for constructing this SSP
    """
    def __init__(self, n1, n2):
        self.n1 = n1
        self.n2 = n2

    def length(self):
        """
        Returns the length of this edge 
        """
        return math.sqrt(float(self.n1.x - self.n2.x) ** 2
                         +float(self.n1.y - self.n2.y) ** 2)



def read_in_topo_map():
    """ Function reads in topological map from ROS topic.

    Returns:
        node_to_loc_sv: Dictionary from N objects to values for the 'location' 
            state factor
        name_to_loc_sv: Dictionary from node names to values for the 'location' 
            state factor
        topological_edges: A list of E objects
    """
    # Read topological map from topic
    rospy.loginfo('Waiting for topological map...')
    ros_topo_map = rospy.wait_for_message('/topological_map',
                                          TopologicalMap)

    if not isinstance(ros_topo_map, TopologicalMap):
        rospy.logerr("Topological Map Not Received...")
        exit()

    rospy.loginfo("Topological Map Received")

    # Convert topological map into our simple form
    node_names = {}
    edge_names = []

    for node in ros_topo_map.nodes:
        node_names[node.name] = N(node.pose.position.x,
                                  node.pose.position.y,
                                  node.pose.orientation,
                                  node.name)
        for edge in node.edges:
            edge_names.append((node.name, edge.node))

    # map from descriptive tags for rooms to node names
    tags = ['start', 'outside_room_1', 'room_1',
            'outside_room_2', 'room_2', 'outside_room_3',
            'room_3', 'outside_room_4', 'room_4']
    node_tags = {}
    rospy.wait_for_service('/topological_map_manager/get_tagged_nodes')
    try:
        get_tags = \
            rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', \
                GetTaggedNodes)

        for tag in tags:
            resp = get_tags(tag)
            if resp.nodes == []:
                raise Exception('No node for tag!')
            else:
                node_tags[tag] = resp.nodes[0]
    except rospy.ServiceException:
        rospy.logerr("Couldn't use get tagged nodes service...")
        exit()

    # Now create node_to_loc_sv and name_to_loc_sv
    node_to_loc_sv = {}
    name_to_loc_sv = {}
    for i in range(len(tags)):
        node_to_loc_sv[node_names[node_tags[tags[i]]]] = i
        name_to_loc_sv[node_tags[tags[i]]] = i

    # Create a list of E objects extracted from the topological map
    bi_edges = [(node_tags['start'], node_tags['outside_room_1']),
                (node_tags['start'], node_tags['outside_room_2']),
                (node_tags['outside_room_1'], node_tags['outside_room_2']),
                (node_tags['outside_room_1'], node_tags['outside_room_3']),
                (node_tags['outside_room_1'], node_tags['outside_room_4']),
                (node_tags['outside_room_2'], node_tags['outside_room_3']),
                (node_tags['outside_room_2'], node_tags['outside_room_4']),
                (node_tags['outside_room_3'], node_tags['outside_room_4']),
                (node_tags['outside_room_1'], node_tags['room_1']),
                (node_tags['outside_room_2'], node_tags['room_2']),
                (node_tags['outside_room_3'], node_tags['room_3']),
                (node_tags['outside_room_4'], node_tags['room_4']),
                (node_tags['room_3'], node_tags['room_4'])]

    topological_edges = []
    rospy.loginfo(edge_names)
    for edge in bi_edges:
        if edge not in edge_names:
            rospy.loginfo(edge)
            raise Exception("Bad edge given")

        topological_edges.append(E(node_names[edge[0]],node_names[edge[1]]))

    return node_to_loc_sv, name_to_loc_sv, topological_edges



def build_spoof_topo_map():
    """ Builds a topological map that closely resembles the true topological 
    map, without having to listen to any ROS topics.

    Returns:
        node_to_loc_sv: Dictionary from N objects to values for the 'location' 
            state factor
        name_to_loc_sv: Dictionary from node names to values for the 'location' 
            state factor
        topological_edges: A list of E objects
    """
    # Make N objects
    start_node = N(3.7, 0.9, 0, 'WayPoint9')
    outside_room_1_node = N(4.1, 2.5, 0, 'WayPoint4')
    room_1_node = N(5.9, 2.3, 0, 'WayPoint8')
    outside_room_2_node = N(3.3, 2.4, 0, 'WayPoint2')
    room_2_node = N(1.3, 2.0, 0, 'WayPoint7')
    outside_room_3_node = N(3.1, 2.9, 0, 'WayPoint1')
    room_3_node = N(1.3, 4.9, 0, 'WayPoint5')
    outside_room_4_node = N(4.3, 2.7, 0, 'WayPoint3')
    room_4_node = N(5.5, 4.7, 0, 'WayPoint6')

    # Make node_to_loc_sv and name_to_loc_sv
    node_to_loc_sv = {
        start_node: 0,
        outside_room_1_node: 1,
        room_1_node: 2,
        outside_room_2_node: 3,
        room_2_node: 4,
        outside_room_3_node: 5,
        room_3_node: 6,
        outside_room_4_node: 7,
        room_4_node: 8,
    }

    name_to_loc_sv = {}
    for n in node_to_loc_sv:
        name_to_loc_sv[n.name] = node_to_loc_sv[n]

    # Create edge objects and put them into a list
    start_to_room_1_edge = E(start_node, outside_room_1_node)
    start_to_room_2_edge = E(start_node, outside_room_2_node)

    room_1_to_room_2_edge = E(outside_room_1_node, outside_room_2_node)
    room_1_to_room_3_edge = E(outside_room_1_node, outside_room_3_node)
    room_1_to_room_4_edge = E(outside_room_1_node, outside_room_4_node)
    room_2_to_room_3_edge = E(outside_room_2_node, outside_room_3_node)
    room_2_to_room_4_edge = E(outside_room_2_node, outside_room_4_node)
    room_3_to_room_4_edge = E(outside_room_3_node, outside_room_4_node)

    room_1_doorway_edge = E(outside_room_1_node, room_1_node)
    room_2_doorway_edge = E(outside_room_2_node, room_2_node)
    room_3_doorway_edge = E(outside_room_3_node, room_3_node)
    room_4_doorway_edge = E(outside_room_4_node, room_4_node)

    tunnel_edge = E(room_3_node, room_4_node)

    topological_edges = [
        start_to_room_1_edge,
        start_to_room_2_edge,
        room_1_to_room_2_edge,
        room_1_to_room_3_edge,
        room_1_to_room_4_edge,
        room_2_to_room_3_edge,
        room_2_to_room_4_edge,
        room_3_to_room_4_edge,
        room_1_doorway_edge,
        room_2_doorway_edge,
        room_3_doorway_edge,
        room_4_doorway_edge,
        tunnel_edge,
    ]

    return node_to_loc_sv, name_to_loc_sv, topological_edges

def make_search_and_rescue_state_factors(topological_edges):
    """
    Creates the state factors needed to make the search and rescue SSP

    Returns:
        A dictionary mapping from state factor names to StateFactor objects
    """
    max_cost = max([EDGE_TIME_COST_TO_LEN_RATIO*e.length() for e in topological_edges])
    max_cost = int(math.ceil(max(max_cost, LARGE_RUBBLE_PILE_CLEARING_TIME_COST)))

    # State factors
    # for room i, location 2i is inside and 2i-1 is outside the door
    # -1 is used for the home location when finished
    loc_factor = IntegerStateFactor(name='location', min=-1, max=8)
    person_counter_factor = IntegerStateFactor(name='num_people_found', min=0, max=4)
    time_factor = IntegerStateFactor(name='time', min=0, max=TIME_HORIZON+2*max_cost)
    number_rooms_searched_factor = IntegerStateFactor(name='num_rooms_searched', min=0, max=4)

    valid_person_states = {'unknown', 'missing', 'found'}
    room_1_person_factor = StateFactor(name='room_1_person', values=valid_person_states)
    room_2_person_factor = StateFactor(name='room_2_person', values=valid_person_states)
    room_3_person_factor = StateFactor(name='room_3_person', values=valid_person_states)
    room_4_person_factor = StateFactor(name='room_4_person', values=valid_person_states)

    rubble_states = {'unknown', 'small_pile', 'large_pile', 'cleared'}
    room_1_rubble_factor = StateFactor(name='room_1_rubble', values=rubble_states)
    room_2_rubble_factor = StateFactor(name='room_2_rubble', values=rubble_states)
    room_3_rubble_factor = StateFactor(name='room_3_rubble', values=rubble_states)
    room_4_rubble_factor = StateFactor(name='room_4_rubble', values=rubble_states)

    state_factor_list = [
        loc_factor,
        person_counter_factor,
        time_factor,
        number_rooms_searched_factor,
        room_1_person_factor,
        room_2_person_factor,
        room_3_person_factor,
        room_4_person_factor,
        room_1_rubble_factor,
        room_2_rubble_factor,
        room_3_rubble_factor,
        room_4_rubble_factor,
    ]

    return {sf.name: sf for sf in state_factor_list}



def make_non_doorway_traversal_transitions(edges, state_factors, node_to_loc_sv):
    """
    Make ProbTransition objects for traversing edges in 'edges'. We assume that 
    there is no restrictions on being able to traverse these edges, and that 
    they are bidirectional.

    Args:
        edges: A list of E objects to make traversal transitions for
        state_factors: A dictionary mapping from state factor names to state 
            factor objects
        node_to_loc_sv: A dictionary mapping from N objects to values for the 
            'location' state factor.

    Returns:
        A list of ProbTransition objects for traversal around the map
    """
    transitions = []

    for edge in edges:
        forward_precond = EqualityCondition(state_factors['location'], node_to_loc_sv[edge.n1])
        forward_action_name = edge.n2.name
        forward_prob_postconds = {
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[edge.n2]),
                CumulativeCondition(state_factors['time'], math.ceil(edge.length()*EDGE_TIME_COST_TO_LEN_RATIO))):
                    1.0
        }

        backward_precond = EqualityCondition(state_factors['location'], node_to_loc_sv[edge.n2])
        backward_action_name = edge.n1.name
        backward_prob_postconds = {
            EqualityCondition(state_factors['location'], node_to_loc_sv[edge.n1]): 
                1.0
        }
        backward_prob_postconds = {
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[edge.n1]),
                CumulativeCondition(state_factors['time'], math.ceil(edge.length()*EDGE_TIME_COST_TO_LEN_RATIO))):
                    1.0
        }

        forward_transition = ProbTransition(
            pre_cond=forward_precond,
            action_name=forward_action_name,
            prob_post_conds=forward_prob_postconds)
        backward_transition = ProbTransition(
            pre_cond=backward_precond,
            action_name=backward_action_name,
            prob_post_conds=backward_prob_postconds)

        transitions.append(forward_transition)
        transitions.append(backward_transition)

    return transitions



def make_rubble_transitions(room_info, state_factors, node_to_loc_sv):
    """
    Makes transitions related to observing and clearing rubble in doorways

    Args:
        room_info: A list of tuples (doorway_edge, person_factor, rubble_factor) 
            where 'doorway_edge' is an edge that may be blocked by rubble, 
            'person_factor' corresponds to a state factor 'room_i_person' for 
            some i and similarly 'rubble_factor' corresponds to a state factor 
            'room_i_rubble'
        state_factors: A dictionary mapping from state factor names to state 
            factor objects
        node_to_loc_sv: A dictionary mapping from N objects to values for the 
            'location' state factor.

    Returns:
        A list of ProbTransition objects for transitions related to observing 
        and clearing rubble
    """
    transitions = []

    for doorway_edge, _, rubble_factor in room_info:
        # observing what rubble there is
        observe_rubble_precond = \
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n1]),
                EqualityCondition(rubble_factor, 'unknown'),
            )
        observe_rubble_action_name = 'check_for_rubble'
        observe_rubble_prob_postconds = {
            EqualityCondition(rubble_factor, 'cleared'): 
                RUBBLE_PILE_CLEAR_PROB,
            EqualityCondition(rubble_factor, 'small_pile'): 
                (1.0 - RUBBLE_PILE_CLEAR_PROB - LARGE_RUBBLE_PILE_PROB),
            EqualityCondition(rubble_factor, 'large_pile'): 
                LARGE_RUBBLE_PILE_PROB,
        }
        observe_rubble_transition = ProbTransition(
            pre_cond=observe_rubble_precond,
            action_name=observe_rubble_action_name,
            prob_post_conds=observe_rubble_prob_postconds)
        transitions.append(observe_rubble_transition)

        # clearing large rubble
        clear_rubble_precond = \
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n1]),
                EqualityCondition(rubble_factor, 'large_pile'),
            )
        clear_rubble_action_name = 'clear_rubble'
        clear_rubble_prob_postconds = {
            ConjunctionCondition(
                EqualityCondition(rubble_factor, 'cleared'),
                CumulativeCondition(state_factors['time'], LARGE_RUBBLE_PILE_CLEARING_TIME_COST)):
                    1.0,
        }
        clear_rubble_transition = ProbTransition(
            pre_cond=clear_rubble_precond,
            action_name=clear_rubble_action_name,
            prob_post_conds=clear_rubble_prob_postconds)
        transitions.append(clear_rubble_transition)

        # clearing small rubble
        clear_rubble_precond = \
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n1]),
                EqualityCondition(rubble_factor, 'small_pile'),
            )
        clear_rubble_action_name = 'clear_rubble'
        clear_rubble_prob_postconds = {
            ConjunctionCondition(
                EqualityCondition(rubble_factor, 'cleared'),
                CumulativeCondition(state_factors['time'], SMALL_RUBBLE_PILE_CLEARING_TIME_COST)):
                    1.0,
        }
        clear_rubble_transition = ProbTransition(
            pre_cond=clear_rubble_precond,
            action_name=clear_rubble_action_name,
            prob_post_conds=clear_rubble_prob_postconds)
        transitions.append(clear_rubble_transition)

    return transitions



def make_doorway_transitions(room_info, state_factors, node_to_loc_sv):
    """
    Makes transitions related to traversing through doorways. We need to model 
    that the robot cannot traverse through a doorway without knowing that it 
    is clear

    Args:
        room_info: A list of tuples (doorway_edge, person_factor, rubble_factor) 
            where 'doorway_edge' is an edge that may be blocked by rubble, 
            'person_factor' corresponds to a state factor 'room_i_person' for 
            some i and similarly 'rubble_factor' corresponds to a state factor 
            'room_i_rubble'
        state_factors: A dictionary mapping from state factor names to state 
            factor objects
        node_to_loc_sv: A dictionary mapping from N objects to values for the 
            'location' state factor.

    Returns:
        A list of ProbTransition objects for transitions related to traversing 
        doorways
    """
    transitions = []

    for doorway_edge, person_factor, rubble_factor in room_info:
        time_cost = math.ceil(doorway_edge.length()*EDGE_TIME_COST_TO_LEN_RATIO)
        # moving into room
        forward_precond = \
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n1]),
                EqualityCondition(rubble_factor, 'cleared'),
            )
        forward_action_name = doorway_edge.n2.name
        forward_prob_postconds = {
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n2]),
                CumulativeCondition(state_factors['time'], time_cost)):
                    1.0
        }
        forward_transition = ProbTransition(
            pre_cond=forward_precond,
            action_name=forward_action_name,
            prob_post_conds=forward_prob_postconds)
        transitions.append(forward_transition)

        # moving out of room
        backward_precond = \
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n2]),
                EqualityCondition(rubble_factor, 'cleared'),
            )
        backward_action_name = doorway_edge.n1.name
        backward_prob_postconds = {
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n1]),
                CumulativeCondition(state_factors['time'], time_cost)):
                    1.0
        }
        backward_transition = ProbTransition(
            pre_cond=backward_precond,
            action_name=backward_action_name,
            prob_post_conds=backward_prob_postconds)
        transitions.append(backward_transition)

    return transitions



def make_check_for_person_transitions(room_info, state_factors, node_to_loc_sv):
    """
    Makes transitions related to searching rooms for people. 

    Args:
        room_info: A list of tuples (doorway_edge, person_factor, rubble_factor) 
            where 'doorway_edge' is an edge that may be blocked by rubble, 
            'person_factor' corresponds to a state factor 'room_i_person' for 
            some i and similarly 'rubble_factor' corresponds to a state factor 
            'room_i_rubble'
        state_factors: A dictionary mapping from state factor names to state 
            factor objects
        node_to_loc_sv: A dictionary mapping from N objects to values for the 
            'location' state factor.

    Returns:
        A list of ProbTransition objects for transitions related to searching 
        for people
    """
    
    """
    YOUR CODE HERE
    You should create ProbTransition objects to update each of the 
    'person_factor's, and the 'num_people_found' and 'num_rooms_searched' 
    factors for when each of the rooms are searched.

    The action name for these ProbTransition objects must be "check_for_person"
    """
    search_person_action_name = "check_for_person"
    transitions = []
    for doorway_edge, person_factor, rubble_factor in room_info:
        
        time_cost = CHECK_FOR_PERSON_TIME_COST
        
        for num_people_found in state_factors['num_people_found']:
            
            for num_rooms_searched in state_factors['num_rooms_searched']:
                
                if num_people_found <= num_rooms_searched:
                
                    unknown_precond = 
                        ConjunctionCondition(
                            EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n2]),
                            EqualityCondition(person_factor, 'unknown')
                        )
                    
                    missing_precond = 
                        ConjunctionCondition(
                            EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n2]),
                            EqualityCondition(person_factor, 'missing')
                        )
                    
                    found_precond = 
                        ConjunctionCondition(
                            EqualityCondition(state_factors['location'], node_to_loc_sv[doorway_edge.n2]),
                            EqualityCondition(person_factor, 'found')
                        )
                    
                    person_found_prob = (NUM_PEOPLE - num_people_found) / (4.0 - num_rooms_searched)
                    person_found_prob = min(person_found_prob, 1.0)
                    person_found_prob = max(0.0, person_found_prob)
                    
                    unknown_postconds = {
                        ConjunctionCondition(
                            CumulativeCondition(state_factors['num_people_found'], 1.0),
                            CumulativeCondition(state_factors['num_rooms_searched'], 1.0),
                            EqualityCondition(person_factor, 'found'),
                            CumulativeCondition(state_factors['time'], time_cost)):
                                person_found_prob,
                        ConjunctionCondition(
                            CumulativeCondition(state_factors['num_rooms_searched'], 1.0),
                            EqualityCondition(person_factor, 'missing'),
                            CumulativeCondition(state_factors['time'], time_cost)):
                                1.0 - person_found_prob
                    }
                    
                    missing_postconds = {
                        ConjunctionCondition(
                            CumulativeCondition(state_factors['time'], time_cost)):
                                1.0
                    }
                    
                    found_postconds = {
                        ConjunctionCondition(
                            CumulativeCondition(state_factors['time'], time_cost)):
                                1.0
                    }
                    
                    unknown_transition = ProbTransition(
                        pre_cond=unknown_precond,
                        action_name=search_person_action_name,
                        prob_post_conds=unknown_postconds)
                    transitions.append(unknown_transition)
                    
                    missing_transition = ProbTransition(
                        pre_cond=missing_precond,
                        action_name=search_person_action_name,
                        prob_post_conds=missing_postconds)
                    transitions.append(missing_transition)
                    
                    found_transition = ProbTransition(
                        pre_cond=found_precond,
                        action_name=search_person_action_name,
                        prob_post_conds=found_postconds)
                    transitions.append(found_transition)
        
        '''
        if person_factor == 'unkown':
            person_found_prob = (NUM_PEOPLE - state_factors['num_people_found']) / (4.0 - state_factors['num_rooms_searched'])
        elif person_factor == 'missing':
            person_found_prob = 0.0
        elif person_factor == 'found':
            person_found_prob = 1.0
        
        search_person_prob_postconds = {
            ConjunctionCondition(
                CumulativeCondition(state_factors['num_people_found'], 1.0),
                CumulativeCondition(state_factors['num_rooms_searched'], 1.0),
                EqualityCondition(person_factor, 'found'),
            
                CumulativeCondition(state_factors['time'], time_cost)):
                    person_found_prob,
            
            ConjunctionCondition(
                CumulativeCondition(state_factors['num_rooms_searched'], 1.0),
                EqualityCondition(person_factor, 'missing'),
            
                CumulativeCondition(state_factors['time'], time_cost)):
                    1.0 - person_found_prob
        } # Stay in room. Person found or not. Time increased.
        
        search_person_transition = ProbTransition(
            pre_cond=search_person_precond,
            action_name=search_person_action_name,
            prob_post_conds=search_person_prob_postconds)
        transitions.append(search_person_transition)
        '''
    
    return transitions



def make_sink_state_transitions(start_node, state_factors, node_to_loc_sv):
    """
    Makes transitions related to traversing to sink state. 

    Args:
        start_node: The N object corresponding to the start location
        state_factors: A dictionary mapping from state factor names to state 
            factor objects
        node_to_loc_sv: A dictionary mapping from N objects to values for the 
            'location' state factor.

    Returns:
        A list of ProbTransition objects for transitions related to traversing 
        doorways
    """
    # Add a finish action to when completed task
    finish_precond = \
        ConjunctionCondition(
            EqualityCondition(state_factors['location'], node_to_loc_sv[start_node]),
            EqualityCondition(state_factors['num_people_found'], NUM_PEOPLE),
        )
    finish_action_name = 'finish'
    finish_prob_postconds = {EqualityCondition(state_factors['location'], -1): 1.0}

    finish_transition = ProbTransition(
        pre_cond=finish_precond,
        action_name=finish_action_name,
        prob_post_conds=finish_prob_postconds)

    # Also add a run_out_of_time action when have run out of time
    no_time_precond = ConjunctionCondition(
        GreaterThanCondition(state_factors['time'], TIME_HORIZON-1),
        GreaterThanCondition(state_factors['location'], -1),
    )
    no_time_action_name = 'run_out_of_time'
    no_time_prob_postconds = {EqualityCondition(state_factors['location'], -1): 1.0}

    no_time_transition = ProbTransition(
        pre_cond=no_time_precond,
        action_name=no_time_action_name,
        prob_post_conds=no_time_prob_postconds)

    return[finish_transition, no_time_transition]





def add_traversal_costs(cost, topological_edges, state_factors, node_to_loc_sv):
    """
    Add costs associated with travesing edges to 'cost'

    Args:
        cost: The StateActionCost object for the SSP
        topological_edges: The list of all E edge objects
        state_factors: A dictionary mapping from state factor names to state 
            factor objects
        node_to_loc_sv: A dictionary mapping from N objects to values for the 
            'location' state factor.
    Returns:
        Nothing
    """
    for edge in topological_edges:
        pre_cond = EqualityCondition(state_factors['location'], node_to_loc_sv[edge.n1])
        action_name = edge.n2.name
        cost_value = EDGE_TIME_COST_TO_LEN_RATIO * edge.length()
        cost.append(pre_cond=pre_cond,
                    action_name=action_name,
                    cost_value=cost_value)
        pre_cond = EqualityCondition(state_factors['location'], node_to_loc_sv[edge.n2])
        action_name = edge.n1.name
        cost_value = EDGE_TIME_COST_TO_LEN_RATIO * edge.length()
        cost.append(pre_cond=pre_cond,
                    action_name=action_name,
                    cost_value=cost_value)



def add_rubble_clear_costs(cost, room_info, state_factors, node_to_loc_sv):
    """
    Add costs associated with clearing rubble to 'cost'

    Args:
        cost: The StateActionCost object for the SSP
        room_info: A list of tuples (doorway_edge, person_factor, rubble_factor) 
            where 'doorway_edge' is an edge that may be blocked by rubble, 
            'person_factor' corresponds to a state factor 'room_i_person' for 
            some i and similarly 'rubble_factor' corresponds to a state factor 
            'room_i_rubble'
        state_factors: A dictionary mapping from state factor names to state 
            factor objects
        node_to_loc_sv: A dictionary mapping from N objects to values for the 
            'location' state factor.

    Returns:
        Nothing
    """
    for edge, _, rubble_factor in room_info:
        pre_cond = \
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[edge.n1]),
                EqualityCondition(rubble_factor, 'small_pile'),
            )
        action_name = 'clear_rubble'
        cost_value = SMALL_RUBBLE_PILE_CLEARING_TIME_COST
        cost.append(pre_cond=pre_cond,
                    action_name=action_name,
                    cost_value=cost_value)
        pre_cond = \
            ConjunctionCondition(
                EqualityCondition(state_factors['location'], node_to_loc_sv[edge.n1]),
                EqualityCondition(rubble_factor, 'large_pile'),
            )
        action_name = 'clear_rubble'
        cost_value = LARGE_RUBBLE_PILE_CLEARING_TIME_COST
        cost.append(pre_cond=pre_cond,
                    action_name=action_name,
                    cost_value=cost_value)



def add_not_finishing_cost(cost, topological_edges, state_factors):
    """
    Adds costs associated with not finishing, so that any path not reaching the 
    goal state will recieve the same total cost and is considered equally as 
    bad

    Args:
        cost: The StateActionCost object for the SSP
        topological_edges: The complete list of E object
        state_factors: A dictionary mapping from state factor names to state 
            factor objects
            
    Returns:
        Nothing
    """
    max_cost = max([EDGE_TIME_COST_TO_LEN_RATIO*e.length() for e in topological_edges])
    max_cost = int(math.ceil(max(max_cost, LARGE_RUBBLE_PILE_CLEARING_TIME_COST)))
    max_time_val = TIME_HORIZON+2*max_cost

    for t in range(TIME_HORIZON, max_time_val):
        pre_cond = EqualityCondition(state_factors['time'], t)
        action_name = 'run_out_of_time'
        cost_value = max_time_val - t
        cost.append(pre_cond=pre_cond,
                    action_name=action_name,
                    cost_value=cost_value)



def make_search_and_rescue_ssp(use_real_map=True):
    """
    Returns a SSP that encapsulates a search and rescue mission
    """
    # Read in topologcial map from ROS topic/spoof function
    if use_real_map:
        node_to_loc_sv, _, topological_edges = read_in_topo_map()
    else:
        node_to_loc_sv, _, topological_edges = build_spoof_topo_map()

    # Give sensible names to each node for building the SSP
    # Rooms are labelled in a CLOCKWISE order, with rooms 1 and 2 being closest 
    # to the robots starting position. If this is confusing try draw it out :) 
    loc_sv_to_node = {node_to_loc_sv[k]: k for k in list(node_to_loc_sv.keys())}

    start_node = loc_sv_to_node[0]
    outside_room_1_node = loc_sv_to_node[1]
    room_1_node = loc_sv_to_node[2]
    outside_room_2_node = loc_sv_to_node[3]
    room_2_node = loc_sv_to_node[4]
    outside_room_3_node = loc_sv_to_node[5]
    room_3_node = loc_sv_to_node[6]
    outside_room_4_node = loc_sv_to_node[7]
    room_4_node = loc_sv_to_node[8]

    # Similarly give sensible names to each of the edges
    start_to_room_1_edge = topological_edges[0]
    start_to_room_2_edge = topological_edges[1]

    room_1_to_room_2_edge = topological_edges[2]
    room_1_to_room_3_edge = topological_edges[3]
    room_1_to_room_4_edge = topological_edges[4]
    room_2_to_room_3_edge = topological_edges[5]
    room_2_to_room_4_edge = topological_edges[6]
    room_3_to_room_4_edge = topological_edges[7]

    room_1_doorway_edge = topological_edges[8]
    room_2_doorway_edge = topological_edges[9]
    room_3_doorway_edge = topological_edges[10]
    room_4_doorway_edge = topological_edges[11]

    tunnel_edge = topological_edges[12]


    # Build state factors
    state_factors = make_search_and_rescue_state_factors(topological_edges)


    # Create the initial state
    initial_state_values = {
        'location': 0,
        'time': 0,
        'num_people_found': 0,
        'num_rooms_searched': 0,
        'room_1_person': 'unknown',
        'room_2_person': 'unknown',
        'room_3_person': 'unknown',
        'room_4_person': 'unknown',
        'room_1_rubble': 'unknown',
        'room_2_rubble': 'unknown',
        'room_3_rubble': 'unknown',
        'room_4_rubble': 'unknown',
    }

    initial_state = State(initial_state_values)


    # Defining transitions - traversal excluding doorways
    transitions = []

    non_doorway_edges = [
        start_to_room_1_edge,
        start_to_room_2_edge,
        room_1_to_room_2_edge,
        room_1_to_room_3_edge,
        room_1_to_room_4_edge,
        room_2_to_room_3_edge,
        room_2_to_room_4_edge,
        room_3_to_room_4_edge,
        tunnel_edge,
    ]

    non_doorway_traversal_transitions = \
        make_non_doorway_traversal_transitions(non_doorway_edges, state_factors, node_to_loc_sv)
    transitions.extend(non_doorway_traversal_transitions)


    # Defining transitions - rubble observations
    room_info = [
        (room_1_doorway_edge, state_factors['room_1_person'], state_factors['room_1_rubble']),
        (room_2_doorway_edge, state_factors['room_2_person'], state_factors['room_2_rubble']),
        (room_3_doorway_edge, state_factors['room_3_person'], state_factors['room_3_rubble']),
        (room_4_doorway_edge, state_factors['room_4_person'], state_factors['room_4_rubble']),
    ]

    rubble_transitions = make_rubble_transitions(room_info, state_factors, node_to_loc_sv)
    transitions.extend(rubble_transitions)


    # Defining transitions - doorway traversal + checking for people
    doorway_transitions = make_doorway_transitions(room_info, state_factors, node_to_loc_sv)
    transitions.extend(doorway_transitions)
    people_transitions = make_check_for_person_transitions(room_info, state_factors, node_to_loc_sv)
    transitions.extend(people_transitions)


    # Defining transitions - enforcing the time horizon
    # To make all states over the time horizon sink states, we need to add the
    # pre condition that the time is below the horizon. This means any state
    # with time having surpassed the time horizon will have zero enabled actions
    for transition in transitions:
        transition.pre_cond = ConjunctionCondition(
            transition.pre_cond,
            LessThanCondition(state_factors['time'], TIME_HORIZON),
        )


    # Defining transitions - sink state
    sink_transitions = make_sink_state_transitions(start_node, state_factors, node_to_loc_sv) 
    transitions.extend(sink_transitions)


    # Defining costs - travel cost
    cost = StateActionCost()
    add_traversal_costs(cost, topological_edges, state_factors, node_to_loc_sv)
    add_rubble_clear_costs(cost, room_info, state_factors, node_to_loc_sv)
    add_not_finishing_cost(cost, topological_edges, state_factors)


    # Finally, make and return the SSP object
    return SSP(state_factors=state_factors,
               initial_state_probs={initial_state: 1.0},
               transitions=transitions,
               costs=[cost],
               index_factors_names=['location','num_people_found'],
               bypass_sanity_checks=True)
