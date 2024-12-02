#!/usr/bin/env python
import numpy as np
import subprocess
from progress.bar import Bar

base_command = "./build/grid2d -x {source_x} -y {source_y} -X {dest_x} -Y {dest_y} -m {map} -s {algorithm} -o {occupancy} -u 500 --atlimit 10 --minDuration 1 --maxDuration 5 --seed {seed} > {outfile}"
algorithms = [
    "sipp",
#    "asipp",
#   "repeat"
]
occupancy = list(map(str, np.linspace(0, 1, 11)))
seeds = list(range(1))
replicates = list(range(1))
maps = {
    "maps/den520/den520d.map": "maps/den520/den520d.map.scen",
    "maps/random/random512-10-0.map": "maps/random/random512-10-0.map.scen"
}

def mkname(x, y, X, Y, oc, alg, seed, m, i, r):
    return "experiment_results_occupancy/" + m.split("/")[-1].split(".")[0] + "_{}_{}_{}_{}_{}_{}_{}_{}_{}.out".format(x, y, X, Y, oc,alg, seed, i, r)

i = 0
instances = []
print("occupancy")
for occ in occupancy:
    print(occ, end = " ")
for m in maps:
    with open(maps[m]) as f:
        for occ in occupancy:
            for line in f:
                for rep in replicates:
                    print("occ: {}".format(occ) + occ)
                    x, y, X, Y = line.strip().split()[4:8]
                    instances.append((x, y, X, Y, occ, m, i, rep))
                i+=1
#print(line)
print(instances)

bar = Bar('experiments', max=len(instances)*len(algorithms)*len(seeds)*len(maps))
for m in maps:
    for alg in algorithms:
        for instance in instances:
            for seed in seeds:
                print(instance)
                command = base_command.format(source_x = instance[0], source_y = instance[1], dest_x = instance[2], dest_y = instance[3], occupancy=instance[4], map = instance[5], algorithm=alg,seed = seed, outfile=mkname(instance[0], instance[1], instance[2], instance[3], instance[4], alg, seed, m, instance[6], instance[7]))
                #subprocess.run(command, shell = True, timeout = 300)
                #print(command)
                #break
                bar.next()
bar.finish()
