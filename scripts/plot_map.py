#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt
try:
    map_path = sys.argv[1]
    start_loc = (int(sys.argv[2]), int(sys.argv[3]))
    end_loc = (int(sys.argv[4]), int(sys.argv[5]))
    out_path = sys.argv[6]
except:
    print("Usage: plot_map.py map start_x start_y goal_x goal_y out")
    sys.exit()

tonum = {
    ".":(255,255,255),
    "G":(255,255,255),
    "S":(255,255,255),
    "@":(0,0,0),
    "O":(0,0,0),
    "T":(0,0,0),
    "W":(0,0,0)
}

map_array = []
i = 0
with open(map_path) as m:
    for line in m.readlines():
        if i < 4:
            i += 1
            continue
        map_array.append(list(map(lambda x: tonum[x], list(line.strip()))))
map_array = np.asarray(map_array, dtype = int)
map_array[start_loc[1],start_loc[0]] = (0,128,255)
map_array[end_loc[1],end_loc[0]] = (17,102,0)

plt.imshow(map_array, origin='upper', interpolation="none")
for  i in range(map_array.shape[0]):
    plt.hlines(i-.5, -.5,  map_array.shape[1]-.5, "k")
for  i in range(map_array.shape[1]):
    plt.vlines(i-.5, -.5,  map_array.shape[0]-.5, "k")
plt.ylim(map_array.shape[0]-.5, 0-.5)
plt.xlim(0-.5, map_array.shape[1]-.5)
plt.tick_params(
    axis='x',          # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    bottom=False,      # ticks along the bottom edge are off
    top=False,         # ticks along the top edge are off
    labelbottom=False) # labels along the bottom edge are off
plt.tick_params(
    axis='y',          # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    left=False,      # ticks along the bottom edge are off
    right=False,         # ticks along the top edge are off
    labelleft=False) # labels along the bottom edge are off
plt.tight_layout()
plt.gcf().set_dpi(1200)
plt.savefig(out_path)