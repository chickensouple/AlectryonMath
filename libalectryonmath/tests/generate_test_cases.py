import numpy as np
import transforms3d
from array import array

N = 9

# creating unit (x, y, z) distributed across a quarter sphere
# creating a half sphere
z = np.linspace(0, 0.9, N-1)
z = np.insert(z, N-2, 0.985) # creating close to identity transforms
# z = np.linspace(0, 0.9, N)

radii = np.sqrt(1 - np.square(z))
theta = np.linspace(0, 2*np.pi, 4*N)

z = np.tile(z, [4*N, 1]).T.reshape(4*N*N)
radii = np.tile(radii, [4*N, 1]).T.reshape(4*N*N)
theta = np.tile(theta, [N])
x = np.sin(theta) * radii
y = np.cos(theta) * radii


# double half sphere for full sphere
x = np.tile(x, 2)
y = np.tile(y, 2)
z = np.concatenate((z, -z))

x = np.concatenate((x, np.array([0, 0])))
y = np.concatenate((y, np.array([0, 0])))
z = np.concatenate((z, np.array([1., -1])))

# # plotting
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(x, y, z)
# plt.show()


# create angles of rotation for each axis of rotation
angles = np.linspace(0.1, np.pi, N)


num_pts = len(x)
num_angles = len(angles)
x = np.tile(x, [num_angles])
y = np.tile(y, [num_angles])
z = np.tile(z, [num_angles])
angles = np.tile(angles, [num_pts, 1]).T.reshape(num_pts*num_angles)

axis_angle = []
for (x_val, y_val, z_val, angle) in zip(x, y, z, angles):
    aa4 = np.array([angle, x_val, y_val, z_val])
    axis_angle.append(aa4)

axis_angle.append(np.array([0, 1, 0, 0.]))

# axis angle representation of all rotations
aa4_list = np.array(axis_angle)
aa3_list = aa4_list[:, 1:] * np.array([aa4_list[:, 0]]).T

num_rots = aa4_list.shape[0]

# generating rotation matrices
rot_list = []
quat_list = []
eulerZYX_list = []
eulerXYZ_list = []
for i in range(num_rots):
    ax = aa4_list[i, 1:]
    angle = aa4_list[i, 0]
    rot = transforms3d.axangles.axangle2mat(ax, angle)
    rot_list.append(rot)
    quat = transforms3d.quaternions.mat2quat(rot)
    quat_list.append(quat)
    eulerZYX = transforms3d.euler.quat2euler(quat, 'rzyx')
    eulerZYX_list.append(eulerZYX)
    eulerXYZ = transforms3d.euler.quat2euler(quat, 'rxyz')
    eulerXYZ_list.append(eulerXYZ)

rot_list = np.array(rot_list)
# rot_list = np.reshape(rot_list, [num_rots, 9]) # rots are in row major
rot_list = np.reshape(rot_list.T, [9, num_rots]).T # rots are in column major
quat_list = np.array(quat_list)
eulerZYX_list = np.array(eulerZYX_list)
eulerXYZ_list = np.array(eulerXYZ_list)



data_list = [rot_list, quat_list, aa3_list, aa4_list, eulerZYX_list, eulerXYZ_list]


print('Created ' + str(num_rots) + ' rotations')

for idx in range(len(data_list)):
    data_list[idx] = np.reshape(data_list[idx], (-1)).tolist()

# flatten data list
data_list = [item for sublist in data_list for item in sublist]
print("Bytes Len: " + str(len(data_list) * 8))

output_file = open('rot_data.data', 'wb')
float_array = array('d', data_list)
float_array.tofile(output_file)
output_file.close()
print('Done')



