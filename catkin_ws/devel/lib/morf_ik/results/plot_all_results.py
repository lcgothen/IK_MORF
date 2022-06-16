import numpy as np
import matplotlib.pyplot as plt
import csv
import os.path
from os import path

filepath = "./devel/lib/morf_ik/results/"
filename_eqs = filepath + "eqs/successes.data"
names = ["IK\n equations", \
        "4 div.\n equations", \
        "5 div.\n equations", \
        "4 div.\n simulation", \
        "5 div.\n simulation"]
filename_nn = [filepath + "4div/batch_01_05_01_50000_02_09/results.data", filepath + "5div/batch_02_10_01_50000_02_09/results.data", \
                filepath + "4div_babbling/batch_02_10_01_50000_02_09/results.data", filepath + "5div_babbling/batch_01_10_01_50000_02_09/results.data"]

n = len(names)

reader = csv.reader(open(filename_eqs), delimiter="\t")
data_eqs = list(reader)

data_nn = []
for i in range(n-1):
    reader = csv.reader(open(filename_nn[i]), delimiter="\t")
    data_nn.append(list(reader))


dur_eqs = 0
dist_eqs = 0
quant_eqs = 0

for row in data_eqs:
    dur_eqs += float(row[1])
    dist_eqs += float(row[2])
    quant_eqs+=1

s = (3,n-1)

dur_nn = np.zeros(n-1)
dist_nn = np.zeros(n-1)
perc = np.zeros(s)
perc_tot = np.zeros(n-1)
quant_nn = np.zeros(n-1)

for i in range(n-1):
    for row in data_nn[i]:
        dur_nn[i] += float(row[1])
        dist_nn[i] += float(row[2])
        perc[0][i] += float(row[3])
        perc[1][i] += float(row[4])
        perc[2][i] += float(row[5])
        quant_nn[i]+=1

dur = []
dist = []

dur.append(dur_eqs/quant_eqs)
dist.append(dist_eqs/quant_eqs)
for i in range(n-1):
    dur.append(dur_nn[i]/quant_nn[i])
    dist.append(dist_nn[i]/quant_nn[i])
    perc[0][i] = perc[0][i]/quant_nn[i]
    perc[1][i] = perc[1][i]/quant_nn[i]
    perc[2][i] = perc[2][i]/quant_nn[i]
    perc_tot[i] = (perc[0][i]+perc[1][i]+perc[2][i])/3


# size = (12,7)

# plt.figure(figsize=size)
plt.figure()
ax = plt.axes()
ax.tick_params(axis='both', which='major', labelsize=12)
ax.tick_params(axis='both', which='minor', labelsize=12)
# plt.title("Average Calculations Duration (ns)")
plt.bar(names, dur, color=['mediumaquamarine', 'skyblue', 'skyblue', 'deepskyblue', 'deepskyblue'])
plt.savefig(filepath + "duration.png")
plt.close()

plt.figure()
ax = plt.axes()
ax.tick_params(axis='both', which='major', labelsize=12)
ax.tick_params(axis='both', which='minor', labelsize=12)
# plt.title("Average Distance to Button Centre (m)")
plt.bar(names, dist, color=['mediumaquamarine', 'skyblue', 'skyblue', 'deepskyblue', 'deepskyblue'])
plt.savefig(filepath + "distance.png")
plt.close()



plt.figure()
ax = plt.axes()
ax.tick_params(axis='both', which='major', labelsize=12)
ax.tick_params(axis='both', which='minor', labelsize=12)
# plt.title("Neural Networks Deviation from Equations (rad)")
plt.bar(names[1:n], perc_tot, color=['skyblue', 'skyblue', 'deepskyblue', 'deepskyblue'])
plt.savefig(filepath + "perc.png")
plt.close()
