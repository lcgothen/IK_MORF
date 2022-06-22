import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import csv
import os.path
from os import path

filepath = "./devel/lib/morf_ik/real_data/"
filename = "./devel/lib/morf_ik/real_data/input.data"


x_length = 0.247507-0.00106799
y_length = 0.18289+0.0837749
z_length = 0.156892+0.18068

x_start = 0.00106799 
y_start = -0.0837749 
z_start = -0.18068 


div=4
divZ=2*div

x = []
y = []
z = []

x_aux = []
y_aux = []
z_aux = []


reader = csv.reader(open(filename), delimiter=" ")
data = list(reader)

for row in data:
    x.append(float(row[0]))
    y.append(float(row[1]))
    z.append(float(row[2]))


sx1 = (div+1, div+1, 10)
sy1 = (div+1, div+1, 10)
sz1 = (div+1, 10)

sx2 = (divZ+1, 10)
sy2 = (divZ+1, div+1, 10)
sz2 = (divZ+1, div+1, 10)

sx3 = (divZ+1, div+1, 10)
sy3 = (divZ+1, 10)
sz3 = (divZ+1, div+1, 10)


x1 = np.zeros(sx1)
y1 = np.zeros(sy1)
z1 = np.zeros(sz1)

for i in range(div+1):
    for j in range(div+1):
        z1[i] = np.linspace(z_start, z_start+z_length, 10)
        for k in range(10):
            x1[i][j][k] = x_start+i*x_length/div
            y1[i][j][k] = y_start+j*y_length/div

x2 = np.zeros(sx2)
y2 = np.zeros(sy2)
z2 = np.zeros(sz2)

for i in range(divZ+1):
    for j in range(div+1):
        x2[i] = np.linspace(x_start, x_start+x_length, 10)
        for k in range(10):
            z2[i][j][k] = z_start+i*z_length/divZ
            y2[i][j][k] = y_start+j*y_length/div

x3 = np.zeros(sx3)
y3 = np.zeros(sy3)
z3 = np.zeros(sz3)

for i in range(divZ+1):
    for j in range(div+1):
        y3[i] = np.linspace(y_start, y_start+y_length, 10)
        for k in range(10):
            z3[i][j][k] = z_start+i*z_length/divZ
            x3[i][j][k] = x_start+j*x_length/div


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter(x, y, z, alpha=0.01)
ax.scatter(x_aux, y_aux, z_aux, 'g', alpha=0.01)
# ax.plot_surface(xx, yy, zz)

for i in range(div+1):
    for j in range(div+1):
        ax.plot(x1[i][j], y1[i][j], z1[i], 'k', linewidth=0.8)

for i in range(divZ+1):
    for j in range(div+1):
        ax.plot(x2[i], y2[i][j], z2[i][j], 'k', linewidth=0.8)

for i in range(divZ+1):
    for j in range(div+1):
        ax.plot(x3[i][j], y3[i], z3[i][j], 'k', linewidth=0.8)


ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
# plt.show()
plt.savefig(filepath + "all_data_cube.png")
plt.close()