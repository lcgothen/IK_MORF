import numpy as np
import matplotlib.pyplot as plt
import csv
import os.path
from os import path

fails = []
filenames = ["./devel/lib/morf_ik/results/1div/batch_01_05_01_50000_03_09/failures.data", \
            "./devel/lib/morf_ik/results/1div/batch_01_20_01_50000_03_09/failures.data", \
            "./devel/lib/morf_ik/results/1div/batch_01_40_01_50000_03_09/failures.data", \
            "./devel/lib/morf_ik/results/1div/batch_02_20_01_50000_03_09/failures.data", \
            "./devel/lib/morf_ik/results/1div/batch_02_40_01_50000_03_09/failures.data"]



for i in range(len(filenames)):
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

y_labels = [1, 1, 1, 2, 2]
x_labels = [5, 20, 40, 20, 40]

y = [1, 1, 1, 2, 2]
x = [1, 2, 3, 2, 3]
z = np.zeros(5)
dx = np.ones(5)
dy = np.ones(5)  

plt.figure()
ax = plt.axes(projection='3d')
ax.set_xticklabels(x_labels)
ax.set_yticklabels(y_labels)
ax.set_xticks(x)
ax.set_yticks(y)
ax.set_ylabel('no. of hidden layers')
ax.set_xlabel('no. of neurons')
plt.title("Percentage of failures")
ax.bar3d(x, y, z, dx, dy, fails, color=['orange', 'tan', 'chocolate', 'sandybrown', 'peru'])
plt.savefig("./devel/lib/morf_ik/results/1div/fails.png")
plt.close()