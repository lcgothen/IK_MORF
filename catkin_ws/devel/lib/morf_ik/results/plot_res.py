import numpy as np
import matplotlib.pyplot as plt
import csv
import os.path
from os import path

filepath = "./catkin_ws/devel/lib/morf_ik/results/batch_01_05_01_50000_03/"

div=4

for j in range(div):
    for k in range(div):
        for l in range(div):
            filename = filepath + str(j) + str(k) + str(l) + ".dat"

            if path.exists(filename):
                reader = csv.reader(open(filename), delimiter="\t")
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
                plt.savefig(filepath + "graphs/" + str(j) + str(k) + str(l)  + ".png")
                plt.close()
                # plt.show()