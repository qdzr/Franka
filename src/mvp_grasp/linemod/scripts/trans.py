from geometry_msgs.msg import Pose  
from tf import transformations as tft
import numpy as np
def quaternion_to_list(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
pose = Pose() 
ready_orientation = [0.0445198873672, 0.0222246313684, 0.662938278256, 0.747018664166]
pose.orientation.x = ready_orientation[0]
pose.orientation.y = ready_orientation[1]
pose.orientation.z = ready_orientation[2]
pose.orientation.w = ready_orientation[3]
camera_rot = tft.quaternion_matrix([-0.00191370172097, 0.000179181800004, 0.705426149517, 0.708780821705])
print(camera_rot.shape)
print(type(camera_rot))
print(camera_rot)
a = np.pi / 2
b = np.pi / 2
y_rot = np.array([[np.cos(a), 0, -np.sin(a), 0], [0, 1, 0, 0], [np.sin(a), 0, np.cos(a), 0], [0, 0, 0, 1]])
print(y_rot)
rot = np.dot(camera_rot, y_rot)
print('rot:')
print(rot)
x_rot = np.array([[1, 0, 0, 0], [0, np.cos(b), -np.sin(b), 0], [0, np.sin(b), np.cos(b), 0], [0, 0, 0, 1]])
print('x_rot: ')
print(x_rot)
matrix = np.dot(rot, x_rot)
quaternion = tft.quaternion_from_matrix(matrix)
print("quaternion: ")
print(quaternion)

q = [0.923904587446, -0.382622456213, -0.00035127511102, 0.000495887519594]
elur = tft.euler_from_quaternion(q)
print('elur: ', elur)
