#!/usr/bin/env python
# -*- coding: utf-8 -*-
from build_search_and_rescue_ssp import make_search_and_rescue_ssp
from mcts import mcts
import numpy as np
from rollout_policy import rollout_policy


ssp = make_search_and_rescue_ssp(False)


dn = mcts(ssp, ssp.get_initial_state(), max_iters=50000, rollout_policy=rollout_policy)
print("Value at root decision node: {v}".format(v=dn.value))

print("Most sampled path:")
print(dn.print_most_selected_path(delimiter="\n->"))
