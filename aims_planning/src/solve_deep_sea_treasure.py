
from deep_sea_treasure import GDST
from ssp import SSP
from mcts import mcts 

import numpy as np

np.random.seed(60415)
gdst = GDST(env_width=5, transition_noise=0.02)
ssp = gdst.to_ssp()

print("GDST environment:")
print(gdst.pretty_print())

dn = mcts(ssp, ssp.get_initial_state(), max_iters=100000)

print("Value at root decision node: {v}".format(v=dn.value))

print("Most sampled path:")
print(dn.print_most_selected_path(delimiter="\n->"))