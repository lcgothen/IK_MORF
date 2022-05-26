import numpy as np
import matplotlib.pyplot as plt
import csv
import os.path
from os import path

filepath = "./devel/lib/morf_ik/results/4div_babbling/"
filename_eqs = filepath + "../eqs/successes.data"
filename_nn = filepath + "batch_01_05_01_50000_03_09/successes.data"

reader = csv.reader(open(filename_eqs), delimiter="\t")
data_eqs = list(reader)
reader = csv.reader(open(filename_nn), delimiter="\t")
data_nn = list(reader)

dur_eqs = 0
dist_eqs = 0
dur_nn = 0
dist_nn = 0
perc = [0, 0, 0]

quant_eqs = 0
quant_nn = 0

for row in data_eqs:
    dur_eqs += float(row[1])
    dist_eqs += float(row[2])
    quant_eqs+=1

for row in data_nn:
    dur_nn += float(row[1])
    dist_nn += float(row[2])
    perc[0] += float(row[3])
    perc[1] += float(row[4])
    perc[2] += float(row[5])
    quant_nn+=1

dur = [dur_eqs/quant_eqs, dur_nn/quant_nn]
dist = [dist_eqs/quant_eqs, dist_nn/quant_nn]

perc = [perc[0]/quant_nn, perc[1]/quant_nn, perc[2]/quant_nn]

plt.title("Average Calculations Duration (ns)")
plt.bar(['IK equations', 'Neural networks'], dur)
plt.savefig(filepath + "duration.png")
plt.close()

plt.title("Average Distance to Button Centre")
plt.bar(['IK equations', 'Neural networks'], dist)
plt.savefig(filepath + "distance.png")
plt.close()

plt.title("Neural Networks Deviation from Equations (%)")
plt.bar(['th1', 'th2', 'th3'], perc)
plt.savefig(filepath + "perc.png")
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