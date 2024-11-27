#!/usr/bin/env python
import numpy as np
import subprocess
from progress.bar import Bar

base_command = "./build/grid2d -x {source_x} -y {source_y} -X {dest_x} -Y {dest_y} -m {map} -s {algorithm} -o 0.25 -u 500 --atlimit 10 --minDuration 1 --maxDuration 5 --seed {seed} > {outfile}"
algorithms = [
    "sipp",
#    "asipp",
#   "repeat"
]
seeds = list(range(1))
replicates = list(range(1))
maps = {
    "maps/den520/den520d.map": "maps/den520/den520d.map.scen",
    }

def mkname(x, y, X, Y, alg, seed, m, i, r):
    return "experiment_results/" + m.split("/")[-1].split(".")[0] + "_{}_{}_{}_{}_{}_{}_{}_{}.out".format(x, y, X, Y, alg, seed, i, r)

i = 0
instances = []
for m in maps:
    with open(maps[m]) as f:
        for line in f:
            for rep in replicates:
                try:
                    x, y, X, Y = line.strip().split()[4:8]
                    instances.append((x, y, X, Y, m, i, rep))
                except:
                    pass
            i+=1
#print(line)

bar = Bar('experiments', max=len(instances)*len(algorithms)*len(seeds))
for m in maps:
    for alg in algorithms:
        for instance in instances:
            for seed in seeds:
                command = base_command.format(source_x = instance[0], source_y = instance[1], dest_x = instance[2], dest_y = instance[3], map = instance[4], algorithm=alg,seed = seed, outfile=mkname(instance[0], instance[1], instance[2], instance[3], alg, seed, m, instance[5], instance[6]))
                #subprocess.run(command, shell = True, timeout = 300)
                print(command)
                #break
                bar.next()
bar.finish()
