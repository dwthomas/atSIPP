#!/usr/bin/env python
import numpy as np
import subprocess
from progress.bar import Bar

base_command = "./build/atsipp -x 0 -X 19 -y 0 -Y 8 -m example/halls.map -s {algorithm} -o 0.5 -u 10000 -b {lookahead} --minDuration 100 --maxDuration 800 --seed {seed} > {outfile}"
base_command = "./build/atsipp -x 139 -X 273 -y 450 -Y 443 -m example/random512-25-0.map -s {algorithm} -o 0.25 -u 10000 -b {lookahead} --minDuration 500 --maxDuration 1000 --seed {seed} > {outfile}"
algorithms = ["rtasipp", "hybrid", "plrtosipp", "plrtosipphonly"]
lookaheads = np.logspace(0, 12, 64, base = 2)
lookaheads = set(np.floor(lookaheads).astype(int))

def mkname(lookahead, alg, seed):
    return "results/random512-25-0_{}_{}_{}.out".format(alg, lookahead, seed)

bar = Bar('experiments', max=32*len(algorithms)*len(lookaheads))
for alg in algorithms:
    for lookahead in lookaheads:
        for seed in range(32):
            command = base_command.format(algorithm=alg, lookahead=lookahead,seed = seed, outfile=mkname(lookahead, alg, seed))
            #subprocess.run(command, shell = True, timeout = 300)
            print(command)
            bar.next()
bar.finish()
