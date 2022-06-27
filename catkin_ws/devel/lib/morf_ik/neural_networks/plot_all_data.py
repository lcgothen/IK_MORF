import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import csv
import os.path
from os import path
from matplotlib import cm
import matplotlib.tri as mtri 


filepath = "./devel/lib/morf_ik/neural_networks/data_4div_direct/"

div=4

x = []
y = []
z = []
c = []

for j in range(div):
    for k in range(div):
        for l in range(div):
            filename = filepath + "train" + str(j) + str(k) + str(l) + ".data"

            if path.exists(filename):
                reader = csv.reader(open(filename), delimiter=" ")
                data = list(reader)

                i=0

                for row in data:
                    if i % 2 != 0:
                        x.append(float(row[0]))
                        y.append(float(row[1]))
                        z.append(float(row[2]))
                        c.append(float(row[2])*1.5)
                        
                    i+=1

# reader = csv.reader(open(filename), delimiter=" ")
# data = list(reader)

# for row in data:
#     x.append(float(row[0]))
#     y.append(float(row[1]))
#     z.append(float(row[2]))



fig = plt.figure()
ax = plt.axes(projection='3d')
# ax.scatter(x, y, z, c=c, vmin=-0.05, vmax=0.20, cmap=cm.coolwarm, s=0.1)
plt.tricontourf(x, y, z, 20, vmin=0, vmax=0.15, cmap=cm.coolwarm)
# ax.plot_surface(xx, yy, zz)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
# plt.show()
plt.savefig(filepath + "all_data.png")
plt.close()

# filename = filepath + "train212.data"

# if path.exists(filename):
#     reader = csv.reader(open(filename), delimiter=" ")
#     data = list(reader)

#     x = []
#     y = []
#     z = []

#     i=0

#     for row in data:
#         if i % 2 != 0:
#             x.append(float(row[0]))
#             y.append(float(row[1]))
#             z.append(float(row[2]))
            
#         i+=1

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.scatter(x, y, z)
# plt.savefig(filepath + "graphs/212.png")
# plt.show()