import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import csv
import os.path
from os import path

filepath = "./devel/lib/morf_ik/babbling_data/5div/"

div=5

x = []
y = []
z = []

for j in range(div):
    for k in range(div):
        for l in range(div*2):
            filename = filepath + str(j) + str(k) + str(l) + ".data"
            # filename = filepath + "train" + str(j) + str(k) + str(l) + ".data"

            if path.exists(filename):
                reader = csv.reader(open(filename), delimiter=" ")
                data = list(reader)

                i=0

                for row in data:
                    if i % 2 != 0:
                        x.append(float(row[0]))
                        y.append(float(row[1]))
                        z.append(float(row[2]))
                        
                    i+=1

# reader = csv.reader(open(filename), delimiter=" ")
# data = list(reader)

# for row in data:
#     x.append(float(row[0]))
#     y.append(float(row[1]))
#     z.append(float(row[2]))

# xx, yy = np.meshgrid(np.linspace(-0.001, 0.0002), np.linspace(-0.05, 0.25))
# zz = xx*yy*0-0.05


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter(x, y, z)
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