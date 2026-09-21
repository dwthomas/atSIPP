#!/usr/bin/env python
import numpy as np
import subprocess
from progress.bar import Bar

base_command = "LD_PRELOAD=/home/aifs2/devin/miniforge3/lib/libstdc++.so ./build/grid2d -x {source_x} -y {source_y} -X {dest_x} -Y {dest_y} -m {map} -s {algorithm} -o {occupancy} -u 1000 --atlimit 100 --minDuration {minD} --maxDuration {maxD} --seed {seed} --test_query_time > {outfile}"
algorithms = [
    #'augmentedsipp'
    # "sipp",
    #  "asipp",
    "repeat"
]

#durs = [(5, 50), (10, 100), (50, 500)]
durs = [(10, 100),]
#occupancy = list(map(str, 0.05 + np.linspace(0, 1, 10)))[0:-1]
occupancy = [0.35,]
seeds = list(range(1))
replicates = list(range(1))
maps = {
    "maps/den520/den520d.map": "maps/den520/den520d.map.scen",
    "maps/random/random512-10-0.map": "maps/random/random512-10-0.map.scen",
    "maps/rooms/64room_006.map" : "maps/rooms/64room_006.map.scen"
}

def mkname(x, y, X, Y, oc, alg, seed, m, i, r, minD, maxD):
    return "experiment_results_jair_repeat_qtime/" + m.split("/")[-1].split(".")[0] + "_{}_{}_{}_{}_{}_{}_{}_{}_{}_{}_{}.out".format(x, y, X, Y, oc, alg, seed, i, r, minD, maxD)

i = 0
instances = []
for m in maps:
    for occ in occupancy:
        for minD, maxD in durs:
            with open(maps[m]) as f:
                for line in f:
                    if "version" in line:
                        continue
                    for rep in replicates:
                        x, y, X, Y = line.strip().split()[4:8]
                        instances.append((x, y, X, Y, occ, m, i, rep, minD, maxD))
                    i+=1
#print(line)

bar = Bar('experiments', max=len(instances)*len(algorithms)*len(seeds))
for alg in algorithms:
    for instance in instances:
        for seed in seeds:
            command = base_command.format(source_x = instance[0], source_y = instance[1], dest_x = instance[2], dest_y = instance[3], occupancy=instance[4], map = instance[5], algorithm=alg, seed = seed, minD=instance[8], maxD=instance[9], outfile=mkname(instance[0], instance[1], instance[2], instance[3], str(int(100*float(instance[4]))), alg, seed, instance[5], instance[6], instance[7], instance[8], instance[9]))
            #subprocess.run(command, shell = True, timeout = 300)
            print(command)
            #break
            bar.next()
bar.finish()
