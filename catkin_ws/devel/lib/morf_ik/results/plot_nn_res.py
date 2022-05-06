import numpy as np
import matplotlib.pyplot as plt
import csv
import os.path
from os import path

dur = []
dist = []
x = []

for i in range(4,6):
    filepath = "./devel/lib/morf_ik/results/"
    filename_nn = filepath + str(i) + "div/nn/successes.data"

    reader = csv.reader(open(filename_nn), delimiter="\t")
    data_nn = list(reader)

    dur_nn = 0
    dist_nn = 0

    quant_nn = 0

    for row in data_nn:
        dur_nn += float(row[1])
        dist_nn += float(row[2])
        quant_nn+=1

    x.append(str(i)+" divisions")
    dur.append(dur_nn/quant_nn)
    dist.append(dist_nn/quant_nn)

plt.title("Average Calculations Duration (ns)")
plt.bar(x, dur)
plt.savefig(filepath + "duration_nn.png")
plt.close()

plt.title("Average Distance to Button Centre")
plt.bar(x, dist)
plt.savefig(filepath + "distance_nn.png")
plt.close()

x=[]
fails=[]

for i in range(4,6):
    filename_nn = filepath + str(i) + "div/nn/failures.data"

    reader = csv.reader(open(filename_nn), delimiter="\t")
    data_nn = list(reader)

    x.append(str(i)+" divisions")
    fails.append(len(data_nn))

plt.title("Failures with NN")
plt.bar(x, fails)
plt.savefig(filepath + "failures_nn.png")
plt.close()