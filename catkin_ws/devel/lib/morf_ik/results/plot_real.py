import numpy as np
import matplotlib.pyplot as plt
import csv
import os.path
from os import path

fails = []
# filenames = ["./devel/lib/morf_ik/results/3div_real/batch_01_05_01_50000_02_09/failures.data", \
#             "./devel/lib/morf_ik/results/3div_real/batch_01_10_01_50000_02_09/failures.data", \
#             "./devel/lib/morf_ik/results/3div_real/batch_02_10_01_50000_02_09/failures.data"]

filenames = ["./devel/lib/morf_ik/results/3div_real/batch_01_05_01_50000_02_09/failures.data", \
            "./devel/lib/morf_ik/results/3div_real/batch_01_10_01_50000_02_09/failures.data", \
            "./devel/lib/morf_ik/results/3div_real/batch_02_10_01_50000_02_09/failures.data", \
            "./devel/lib/morf_ik/results/4div_real/batch_01_05_01_50000_02_09/failures.data", \
            "./devel/lib/morf_ik/results/4div_real/batch_01_10_01_50000_02_09/failures.data", \
            "./devel/lib/morf_ik/results/4div_real/batch_02_10_01_50000_02_09/failures.data"]


filepath = "./devel/lib/morf_ik/results/3div_real/"

n = len(filenames)

for i in range(n):
    reader = csv.reader(open(filenames[i]), delimiter="\t")
    data = list(reader)

    fails.append(len(data)/30.0*100)

# size = (5,4)
# plt.figure(figsize=size)
# ax = plt.axes()
# ax.set_xlabel('no. of neurons')
# ax.set_ylabel('percentage of failures')
# plt.bar(['5', '20', '40', '20', '40'], fails, color=['skyblue', 'deepskyblue', 'steelblue'])
# plt.savefig("./devel/lib/morf_ik/results/1div/fails.png")
# plt.close()

# y_labels = [1, 1, 2]
# x_labels = [5, 10, 10]
x_labels = ["1 layer\n 5 neurons", "1 layer\n 10 neurons", "     2 layers\n       10 neurons", "", "", ""]
y_labels = [3, "", "", 4, "", ""]

# y = [1, 1, 2]
# x = [1, 2, 2]

x = [1, 2, 3, 1, 2, 3]
y = [2, 2, 2, 1, 1, 1]
z = np.zeros(n)
dx = np.ones(n)
dy = np.ones(n)    

plt.rcParams.update({'font.size': 12})

size = (6,6)
plt.figure(figsize=size)
ax = plt.axes(projection='3d')
ax.set_xticklabels(x_labels)
ax.set_yticklabels(y_labels)
ax.set_xticks(x)
ax.set_yticks(y)
ax.set_xlabel('configuration', labelpad=20)
ax.set_ylabel('no. of divisions', labelpad=7)
plt.title("Percentage of failures")
ax.bar3d(x, y, z, dx, dy, fails, color=['orange', 'wheat', 'tan', 'chocolate', 'sandybrown', 'peru'])
plt.savefig(filepath+"fails.png")
plt.close()