import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import csv
import os.path
from os import path

filepath = "./devel/lib/morf_ik/results/"
filename_eqs = filepath + "eqs/successes.data"
# names = ["4 div eq:\n 1 hidden\n 5 neurons\n 0.02 error", "4 div eq:\n 1 hidden\n 5 neurons\n 0.03 error", "4 div eq:\n 1 hidden\n 10 neurons\n 0.02 error", \
#         "4 div eq:\n 2 hidden\n 10 neurons\n 0.02 error"]

# names = ["4 div sim:\n 1 hidden\n 5 neurons\n 0.02 error", \
#         "4 div sim:\n 1 hidden\n 5 neurons\n 0.03 error", "4 div sim:\n 2 hidden\n 10 neurons\n 0.02 error"]
names = ["5 div eq:\n 1 hidden\n 3 neurons\n 0.02 error", "5 div eq:\n 1 hidden\n 5 neurons\n 0.02 error"]

# filename_nn = [filepath + "4div/batch_01_05_01_50000_02_09/results.data", filepath + "4div/batch_01_10_01_50000_02_09/results.data", \
#                 filepath + "4div/batch_02_10_01_50000_02_09/results.data"]

# filename_nn = [filepath + "4div_babbling/batch_01_05_01_50000_02_09/results.data", \
#                 filepath + "4div_babbling/batch_01_05_01_50000_03_09/results.data", filepath + "4div_babbling/batch_02_10_01_50000_02_09/results.data"]
filename_nn = [filepath + "5div/batch_01_03_01_50000_02_09/results.data", \
                filepath + "5div/batch_01_05_01_50000_02_09/results.data"]
# filename_nn = [filepath + "5div_babbling/batch_01_03_01_50000_02_09/results.data"]

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

x_labels = [1, 1]
y_labels = [3, 5]

x = [1, 1]
y = [1, 2]
z = np.zeros(n)
dx = np.ones(n)
dy = np.ones(n)         


# size = (12,7)

plt.figure()
ax = plt.axes(projection='3d')
ax.set_xticklabels(x_labels)
ax.set_yticklabels(y_labels)
ax.set_xticks(x)
ax.set_yticks(y)
ax.set_xlabel('no. of hidden layers')
ax.set_ylabel('no. of neurons')
plt.title("Average Calculations Duration (ns)")
ax.bar3d(x, y, z, dx, dy, dur)
# plt.show()
plt.savefig(filepath + "duration_5div_eqs.png")
plt.close()

plt.figure()
ax = plt.axes(projection='3d')
ax.set_xticklabels(x_labels)
ax.set_yticklabels(y_labels)
ax.set_xticks(x)
ax.set_yticks(y)
ax.set_xlabel('no. of hidden layers')
ax.set_ylabel('no. of neurons')
plt.title("Average Distance to Button Centre (m)")
ax.bar3d(x, y, z, dx, dy, dist)
plt.savefig(filepath + "distance_5div_eqs.png")
plt.close()



plt.figure()
ax = plt.axes(projection='3d')
ax.set_xticklabels(x_labels)
ax.set_yticklabels(y_labels)
ax.set_xticks(x)
ax.set_yticks(y)
ax.set_xlabel('no. of hidden layers')
ax.set_ylabel('no. of neurons')
plt.title("Neural Networks Deviation from Equations (m)")
ax.bar3d(x, y, z, dx, dy, perc_tot)
plt.savefig(filepath + "perc_5div_eqs.png")
plt.close()
