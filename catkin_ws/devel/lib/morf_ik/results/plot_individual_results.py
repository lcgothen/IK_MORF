import numpy as np
import matplotlib.pyplot as plt
import csv
import os.path
from os import path

filepath = "./devel/lib/morf_ik/results/5div/batch_02_10_01_50000_02_09/"
filename = filepath + "results.data"

reader = csv.reader(open(filename), delimiter="\t")
data = list(reader)

colours = []
colourmap = np.array(['darkgreen', 'darkred'])

dist = []
i=0
for row in data:
    dist.append(float(row[2]))
    i+=1
    if float(row[2]) > 0.02:
        colours.append(1)
    else:
        colours.append(0)



plt.title("Distance to Button Centre")
plt.scatter(range(i), dist, color = colourmap[colours])
plt.savefig(filepath + "distance_indiv.png")
plt.close()
