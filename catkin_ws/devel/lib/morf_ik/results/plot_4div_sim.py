import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import csv
import os.path
from os import path

filepath = "./devel/lib/morf_ik/results/4div_babbling/"

filename_nn = [filepath + "batch_01_05_01_50000_02_09/results.data", filepath + "batch_01_10_01_50000_02_09/results.data", \
                filepath + "batch_02_10_01_50000_02_09/results.data"]

n = len(filename_nn)


data_nn = []
for i in range(n):
    reader = csv.reader(open(filename_nn[i]), delimiter="\t")
    data_nn.append(list(reader))


s = (3,n)

dur_nn = np.zeros(n)
dist_nn = np.zeros(n)
perc = np.zeros(s)
perc_tot = np.zeros(n)
quant_nn = np.zeros(n)

for i in range(n):
    for row in data_nn[i]:
        dur_nn[i] += float(row[1])
        dist_nn[i] += float(row[2])
        perc[0][i] += float(row[3])
        perc[1][i] += float(row[4])
        perc[2][i] += float(row[5])
        quant_nn[i]+=1

dur = []
dist = []

for i in range(n):
    dur.append(dur_nn[i]/quant_nn[i])
    dist.append(dist_nn[i]/quant_nn[i])
    perc[0][i] = perc[0][i]/quant_nn[i]
    perc[1][i] = perc[1][i]/quant_nn[i]
    perc[2][i] = perc[2][i]/quant_nn[i]
    perc_tot[i] = (perc[0][i]+perc[1][i]+perc[2][i])/3

y_labels = [1, 1, 2]
x_labels = [5, 10, 10]

y = [1, 1, 2]
x = [1, 2, 2]
z = np.zeros(3)
dx = np.ones(3)
dy = np.ones(3)         


# size = (12,7)

plt.figure()
ax = plt.axes(projection='3d')
ax.set_xticklabels(x_labels)
ax.set_yticklabels(y_labels)
ax.set_xticks(x)
ax.set_yticks(y)
ax.set_ylabel('no. of hidden layers')
ax.set_xlabel('no. of neurons')
plt.title("Average Calculations Duration (ns)")
ax.bar3d(x, y, z, dx, dy, dur, color=['skyblue', 'deepskyblue', 'steelblue'])
# plt.show()
plt.savefig(filepath + "duration_4div_sim.png")
plt.close()

plt.figure()
ax = plt.axes(projection='3d')
ax.set_xticklabels(x_labels)
ax.set_yticklabels(y_labels)
ax.set_xticks(x)
ax.set_yticks(y)
ax.set_ylabel('no. of hidden layers')
ax.set_xlabel('no. of neurons')
plt.title("Average Distance to Button Centre (m)")
ax.bar3d(x, y, z, dx, dy, dist, color=['skyblue', 'deepskyblue', 'steelblue'])
plt.savefig(filepath + "distance_4div_sim.png")
plt.close()



plt.figure()
ax = plt.axes(projection='3d')
ax.set_xticklabels(x_labels)
ax.set_yticklabels(y_labels)
ax.set_xticks(x)
ax.set_yticks(y)
ax.set_ylabel('no. of hidden layers')
ax.set_xlabel('no. of neurons')
plt.title("Neural Networks Deviation from Equations (m)")
ax.bar3d(x, y, z, dx, dy, perc_tot, color=['skyblue', 'deepskyblue', 'steelblue'])
plt.savefig(filepath + "perc_4div_sim.png")
plt.close()
