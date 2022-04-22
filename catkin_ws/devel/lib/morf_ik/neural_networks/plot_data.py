import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import csv
import os.path
from os import path

filepath = "./catkin_ws/devel/lib/morf_ik/data_5div_bigger/"

div=5

for j in range(div):
    for k in range(div):
        for l in range(div):
            filename = filepath + "train" + str(j) + str(k) + str(l) + ".data"

            if path.exists(filename):
                reader = csv.reader(open(filename), delimiter=" ")
                data = list(reader)

                x = []
                y = []
                z = []

                i=0

                for row in data:
                    if i % 2 != 0:
                        x.append(float(row[0]))
                        y.append(float(row[1]))
                        z.append(float(row[2]))
                        
                    i+=1

                fig = plt.figure()
                ax = plt.axes(projection='3d')
                ax.scatter(x, y, z)
                ax.set_xlabel('x')
                ax.set_ylabel('y')
                ax.set_zlabel('z')
                plt.savefig(filepath + "graphs/" + str(j) + str(k) + str(l)  + ".png")
                plt.close()
                # plt.show()

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