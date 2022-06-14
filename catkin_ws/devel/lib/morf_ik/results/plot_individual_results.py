import numpy as np
import matplotlib.pyplot as plt
import csv
import os.path
from os import path

filepath = "./devel/lib/morf_ik/results/1div/batch_01_05_01_50000_03_09/"
filename = filepath + "results.data"
# filename_fail = filepath + "failures.data"

reader = csv.reader(open(filename), delimiter="\t")
data = list(reader)
# reader = csv.reader(open(filename_fail), delimiter="\t")
# data_fail = list(reader)

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

# dist_fail = []
# i_fail=0
# for row in data_fail:
#     dist_fail.append(float(row[2]))
#     i_fail+=1


plt.title("Distance to Button Centre")
plt.scatter(range(i), dist, color = colourmap[colours])
# plt.scatter(range(i_fail), dist_fail, color = 'darkred')
plt.savefig(filepath + "distance_indiv.png")
plt.close()

# filename_nn = filepath + "nn/failures.data"

# dist_nn = 0
# quant_dist = 0
# quant_dur = 0

# reader = csv.reader(open(filename_nn), delimiter="\t")
# data_nn = list(reader)

# for row in data_nn:
#     if row[1]=='distance':
#         dist_nn += float(row[2])
#         quant_dist+=1
#     else:
#         quant_dur+=1

# if quant_dist>0:
#     dist = dist_nn/quant_dist
# else:
#     dist=0

# dist_name = 'Distance \n avg: ' + str(dist)

# plt.title("Failures with NN")
# plt.bar(['Duration', dist_name], [quant_dur, quant_dist])
# plt.savefig(filepath + "failures_nn.png")
# plt.close()