#!/usr/bin/env python
from build_search_and_rescue_ssp import build_spoof_topo_map
import numpy as np


node_to_loc_sv, name_to_loc_sv, topological_edges = build_spoof_topo_map()


def rollout_policy(state, action):
    if 'check_rubble' in action:
        return 'check_rubble'
    # print(state, action)
    return np.random.choice(action)
