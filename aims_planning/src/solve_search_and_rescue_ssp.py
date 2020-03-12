#!/usr/bin/env python
# -*- coding: utf-8 -*-
from build_search_and_rescue_ssp import make_search_and_rescue_ssp
from mcts import mcts

ssp = make_search_and_rescue_ssp(False)
dn = mcts(ssp, ssp.get_initial_state(), max_iters=50000)
print("Value at root decision node: {v}".format(v=dn.value))

print("Most sampled path:")
print(dn.print_most_selected_path(delimiter="\n->"))