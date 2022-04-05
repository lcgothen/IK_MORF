import numpy as np
import matplotlib.pyplot as plt
import csv

reader = csv.reader(open("./catkin_ws/devel/lib/morf_ik/results/batch_01_100_03_5000.dat"), delimiter="\t")
data = list(reader)

x = []
err_train = []
err_vali = []

for row in data:
    x.append(float(row[0]))
    err_train.append(float(row[1]))
    err_vali.append(float(row[2]))

plt.plot(err_vali, label = "validation error")
plt.plot(err_train, label = "training error")
plt.legend()
plt.savefig('./catkin_ws/devel/lib/morf_ik/results/batch_01_100_03_5000.png')
# plt.show()