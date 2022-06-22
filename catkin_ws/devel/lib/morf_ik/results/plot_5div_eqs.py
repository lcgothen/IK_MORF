import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import csv
import os.path
from os import path
import statistics
from scipy import stats
from statsmodels.multivariate.manova import MANOVA
import pandas as pd

filepath = "./devel/lib/morf_ik/results/5div/"

filename_nn = [filepath + "batch_01_05_01_50000_02_09/results.data", filepath + "batch_01_10_01_50000_02_09/results.data", \
                filepath + "batch_02_10_01_50000_02_09/results.data"]

names = ['1 layer, 5 neurons', '1 layer, 10 neurons', '2 layers, 10 neurons']

n = len(filename_nn)


data_nn = []
for i in range(n):
    reader = csv.reader(open(filename_nn[i]), delimiter="\t")
    data_nn.append(list(reader))


dur_vals = []
dev_vals = []
aux_dev = []
aux_dur = []

duration = []
deviation = []
configuration = []

for i in range(n):
    for row in data_nn[i]:
        aux_dev.append((float(row[3])+float(row[4])+float(row[5]))/3)
        aux_dur.append(float(row[1]))

        duration.append(float(row[1]))
        deviation.append((float(row[3])+float(row[4])+float(row[5]))/3)
        configuration.append(names[i])
        
    dev_vals.append(aux_dev)
    dur_vals.append(aux_dur)

    aux_dev = []
    aux_dur = []

s = (2,n)
dev = np.zeros(s)
dur = np.zeros(s)

for i in range(n):
    dev[0][i] = statistics.mean(dev_vals[i])
    dev[1][i] = statistics.variance(dev_vals[i], dev[0][i])
    dur[0][i] = statistics.mean(dur_vals[i])
    dur[1][i] = statistics.variance(dur_vals[i], dur[0][i])

print(stats.f_oneway(dev_vals[2], dev_vals[1], dev_vals[0]))
data = pd.DataFrame({'configuration' : configuration, 'duration' : duration, 'deviation' : deviation})
# print(data)
manova_result = MANOVA.from_formula('duration + deviation ~ configuration', data)
# print(manova_result.mv_test(hypotheses=None))

print("Duration")
print(stats.ttest_ind(data[data['configuration'] == '1 layer, 5 neurons']['duration'], data[data['configuration'] == '1 layer, 10 neurons']['duration'], equal_var=False)) 
print(stats.ttest_ind(data[data['configuration'] == '1 layer, 5 neurons']['duration'], data[data['configuration'] == '2 layers, 10 neurons']['duration'], equal_var=False)) 
print(stats.ttest_ind(data[data['configuration'] == '1 layer, 10 neurons']['duration'], data[data['configuration'] == '2 layers, 10 neurons']['duration'], equal_var=False)) 

print("Deviation")
print(stats.ttest_ind(data[data['configuration'] == '1 layer, 5 neurons']['deviation'], data[data['configuration'] == '1 layer, 10 neurons']['deviation'], equal_var=False)) 
print(stats.ttest_ind(data[data['configuration'] == '1 layer, 5 neurons']['deviation'], data[data['configuration'] == '2 layers, 10 neurons']['deviation'], equal_var=False)) 
print(stats.ttest_ind(data[data['configuration'] == '1 layer, 10 neurons']['deviation'], data[data['configuration'] == '2 layers, 10 neurons']['deviation'], equal_var=False)) 


y_labels = [1, 1, 2]
x_labels = [5, 10, 10]

y = [1, 1, 2]
x = [1, 2, 2]
z = np.zeros(n)
dx = np.ones(n)
dy = np.ones(n)          


# size = (12,7)

# plt.figure()
# ax = plt.axes(projection='3d')
# ax.set_xticklabels(x_labels)
# ax.set_yticklabels(y_labels)
# ax.set_xticks(x)
# ax.set_yticks(y)
# ax.set_ylabel('no. of hidden layers')
# ax.set_xlabel('no. of neurons')
# plt.title("Calculations Duration (ns) - Mean")
# ax.bar3d(x, y, z, dx, dy, dur[0], color=['skyblue', 'deepskyblue', 'steelblue'])
# plt.savefig(filepath + "dur_5div_eqs.png")
# plt.close()


# plt.figure()
# ax = plt.axes(projection='3d')
# ax.set_xticklabels(x_labels)
# ax.set_yticklabels(y_labels)
# ax.set_xticks(x)
# ax.set_yticks(y)
# ax.set_ylabel('no. of hidden layers')
# ax.set_xlabel('no. of neurons')
# plt.title("Neural Networks Deviation from Equations (rad) - Mean")
# ax.bar3d(x, y, z, dx, dy, dev[0], color=['skyblue', 'deepskyblue', 'steelblue'])
# plt.savefig(filepath + "dev_5div_eqs.png")
# plt.close()

# plt.figure()
# ax = plt.axes(projection='3d')
# ax.set_xticklabels(x_labels)
# ax.set_yticklabels(y_labels)
# ax.set_xticks(x)
# ax.set_yticks(y)
# ax.set_ylabel('no. of hidden layers')
# ax.set_xlabel('no. of neurons')
# plt.title("Calculations Duration (ns) - Variance")
# ax.bar3d(x, y, z, dx, dy, dur[1], color=['skyblue', 'deepskyblue', 'steelblue'])
# plt.savefig(filepath + "Vdur_5div_eqs.png")
# plt.close()


# plt.figure()
# ax = plt.axes(projection='3d')
# ax.set_xticklabels(x_labels)
# ax.set_yticklabels(y_labels)
# ax.set_xticks(x)
# ax.set_yticks(y)
# ax.set_ylabel('no. of hidden layers')
# ax.set_xlabel('no. of neurons')
# plt.title("Neural Networks Deviation from Equations (rad) - Variance")
# ax.bar3d(x, y, z, dx, dy, dev[1], color=['skyblue', 'deepskyblue', 'steelblue'])
# plt.savefig(filepath + "Vdev_5div_eqs.png")
# plt.close()