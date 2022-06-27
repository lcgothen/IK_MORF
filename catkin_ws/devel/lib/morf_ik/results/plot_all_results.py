import numpy as np
import matplotlib.pyplot as plt
import csv
import os.path
from os import path
import statistics
from scipy import stats
from statsmodels.multivariate.manova import MANOVA
import pandas as pd

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


dur_eqs = []
dist_eqs = []
dur = []
dist = []

for row in data_eqs:
    dur_eqs.append(float(row[1]))
    dist_eqs.append(float(row[2]))

dur.append(statistics.mean(dur_eqs))
dist.append(statistics.mean(dist_eqs))

dur_nn = []
dev_nn = []
dist_nn = []
aux_dev = []
aux_dur = []
aux_dist = []

duration = []
deviation = []
configuration = []

for i in range(n-1):
    for row in data_nn[i]:
        aux_dev.append((float(row[3])+float(row[4])+float(row[5]))/3)
        aux_dur.append(float(row[1]))
        aux_dist.append(float(row[2]))

        duration.append(float(row[1]))
        deviation.append((float(row[3])+float(row[4])+float(row[5]))/3)
        configuration.append(names[i+1])
        
    dev_nn.append(aux_dev)
    dur_nn.append(aux_dur)
    dist_nn.append(aux_dist)

    aux_dev = []
    aux_dur = []

dev = []

for i in range(n-1):
    dur.append(statistics.mean(dur_nn[i]))
    dist.append(statistics.mean(dist_nn[i]))
    dev.append(statistics.mean(dev_nn[i]))

data = pd.DataFrame({'configuration' : configuration, 'duration' : duration, 'deviation' : deviation})

print("Duration")
print(stats.ttest_ind(data[data['configuration'] == '5 div.\n equations']['duration'], data[data['configuration'] == '4 div.\n equations']['duration'], equal_var=False, alternative='less')) 

print("Deviation")
print(stats.ttest_ind(data[data['configuration'] == '4 div.\n equations']['deviation'], data[data['configuration'] == '5 div.\n equations']['deviation'], equal_var=False, alternative='less')) 


# size = (12,7)

# plt.figure(figsize=size)
# plt.figure()
# ax = plt.axes()
# ax.tick_params(axis='both', which='major', labelsize=12)
# ax.tick_params(axis='both', which='minor', labelsize=12)
# # plt.title("Average Calculations Duration (ns)")
# plt.bar(names, dur, color=['mediumaquamarine', 'skyblue', 'skyblue', 'deepskyblue', 'deepskyblue'])
# plt.savefig(filepath + "duration.png")
# plt.close()

# plt.figure()
# ax = plt.axes()
# ax.tick_params(axis='both', which='major', labelsize=12)
# ax.tick_params(axis='both', which='minor', labelsize=12)
# # plt.title("Average Distance to Button Centre (m)")
# plt.bar(names, dist, color=['mediumaquamarine', 'skyblue', 'skyblue', 'deepskyblue', 'deepskyblue'])
# plt.savefig(filepath + "distance.png")
# plt.close()

# plt.figure()
# ax = plt.axes()
# ax.tick_params(axis='both', which='major', labelsize=12)
# ax.tick_params(axis='both', which='minor', labelsize=12)
# # plt.title("Neural Networks Deviation from Equations (rad)")
# plt.bar(names[1:n], dev, color=['skyblue', 'skyblue', 'deepskyblue', 'deepskyblue'])
# plt.savefig(filepath + "deviation.png")
# plt.close()


# plt.figure()
# ax = plt.axes()
# ax.tick_params(axis='both', which='major', labelsize=12)
# ax.tick_params(axis='both', which='minor', labelsize=12)
# # plt.title("Neural Networks Deviation from Equations (rad)")
# plt.bar(names[1:3], dev[0:2], color=['skyblue', 'deepskyblue'])
# plt.savefig(filepath + "dev_nn_eqs.png")
# plt.close()

# plt.figure()
# ax = plt.axes()
# ax.tick_params(axis='both', which='major', labelsize=12)
# ax.tick_params(axis='both', which='minor', labelsize=12)
# # plt.title("Neural Networks Deviation from Equations (rad)")
# plt.bar(names[1:3], dur[0:2], color=['skyblue', 'deepskyblue'])
# plt.savefig(filepath + "dur_nn_eqs.png")
# plt.close()