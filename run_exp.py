#!/usr/bin/env python
import numpy as np
import subprocess
from progress.bar import Bar

base_commands = ["./build/atsipp -x 139 -X 273 -y 450 -Y 443 -m example/random512-25-0.map -s {algorithm} -o 0.25 -u 10000 -b {lookahead} --minDuration 500 --maxDuration 1000 --seed {seed} > {outfile}",
                 "./build/atsipp -x 184 -y 57 -X 18 -Y 72 -m example/den520d.map -s {algorithm} -o 0.5 -u 10000 -b {lookahead} --minDuration 500 --maxDuration 1000 --seed {seed} > {outfile}",
                 "./build/atsipp -x 2 -y 2 -X 158 -Y 60 -m example/warehouse-10-20-10-2-1.map -s {algorithm} -o 0.1 -u 10000 -b {lookahead} --minDuration 500 --maxDuration 1000 --seed {seed} > {outfile}",
                 "./build/atsipp -x 24 -X 24 -y 0 -Y 48 -m example/cup49.map -s {algorithm} -o 0.5 -u 10000 -b {lookahead} --minDuration 10 --maxDuration 100 --seed {seed} > {outfile}"
                 ]
algorithms = ["rtasipp", "hybrid", "plrtosipp", "plrtosipphonly"]
lookaheads = np.logspace(0, 12, 32, base = 2)
lookaheads = set(np.floor(lookaheads).astype(int))
maps = ["random512-25-0.map", "den520d.map", "warehouse-10-20-10-2-1.map", "cup49.map"]

def mkname(lookahead, alg, seed, m):
    return "final_results/" + m + "_{}_{}_{}.out".format(alg, lookahead, seed)

bar = Bar('experiments', max=32*len(algorithms)*len(lookaheads)*len(maps))
for m in maps:
    base_command = ""
    for c in base_commands:
        if m in c:
            base_command = c
            break
    
    for alg in algorithms:
        for lookahead in lookaheads:
            for seed in range(32):
                command = base_command.format(algorithm=alg, lookahead=lookahead,seed = seed, outfile=mkname(lookahead, alg, seed, m))
                #subprocess.run(command, shell = True, timeout = 300)
                print(command)
                #break
                bar.next()
bar.finish()
