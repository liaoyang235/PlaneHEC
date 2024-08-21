import numpy as np
import cv2 
import csv
import os
from PIL import Image

def deproject(intrinsics,pixel,depth,reverse=[False,False]):
    # 焦距 fx = intrinsics[0][0][0]
    # 焦距 fy = intrinsics[0][1][1]
    # 主点（像素中心） ppx = intrinsics[0][0][2]
    # 主点（像素中心） ppy = intrinsics[0][1][2]
    # 令世界坐标为pw，像素坐标是px
    # pwx = (pxx - ppx) / fx * depth
    # pwy = (pxy - ppy) / fy * depth
    # pwz = depth
    # 另外注意一下传入的pixel的x，y坐标顺序
    p = [(pixel[0]-intrinsics[0][0][2])/intrinsics[0][0][0],(pixel[1]-intrinsics[0][1][2])/intrinsics[0][1][1]]
    if reverse[0]:
        p[0] = -p[0]
    if reverse[1]:
        p[1] = -p[1]
    return [depth*p[0],depth*p[1],depth]

# 实际上是对去投影的逆运算
def project(intrinsics,pw,reverse=[False,False]):
    # pxx = pwx * fx / pwz + ppx
    # pxy = pwy * fy / pwz + ppy
    u = intrinsics[0][0][0]*pw[0]/pw[2]+intrinsics[0][0][2]
    v = intrinsics[0][1][1]*pw[1]/pw[2]+intrinsics[0][1][2]
    if reverse[0]:
        u = -u
    if reverse[1]:
        v = -v
    return [u,v]

def quaternion2matrix(q):
    #q:list
    w,x,y,z = q
    return np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
             [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
             [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])

# 这是两组手工测量的值，用画图软件找标定板左上角角点的像素位置。
# 其中pg1和pg2是分别在标定板处于不同位置时进行拍照获得的，在depth和rgb中分别对应前6张图和后6张图
pg1 = [[807,280],[820,301],[637,503],[550,393],[641,112],[414,421]]
pg2 = [[750,354],[702,321],[995,262],[909,142],[611,301],[517,177]]

# Matlab计算出的X
X = np.asarray([[-0.7049,0.7160,0.0288,0.0217],[-0.6993,-0.7111,0.0245,0.1012],[0.0418,-0.0033,0.9965,0.0110],[0,0,0,1.0000]])

# 已经获知的相机内参
intrinsics = [[[921.13,0,649.02], [0,921.43,360.41], [0,0,1]]]
As = []
depths = []
path = 'depth'

# As.csv记录了每张图片拍摄时flange的位姿，格式同之前的
with open('As.csv','r') as f:
    lines = f.readlines()
    for line in lines:
        line = line.split(',')
        R = list(quaternion2matrix((float(line[6]),float(line[3]),float(line[4]),float(line[5]))))
        T = np.zeros((4,4))
        T[0:3,0:3] = R
        T[0][3] = float(line[0])
        T[1][3] = float(line[1])
        T[2][3] = float(line[2])
        T[3][3] = 1
        As.append(T)

for f in os.listdir(path):
    depths.append(np.array(Image.open(os.path.join(path, f))))

#对第一组数据进行重投影误差计算
error1 = 0
cnt1 = 0
for i in range(len(pg1)):
    d = depths[i][pg1[i][1]][pg1[i][0]]
    if d==0 or d>2000:
        continue
    # 计算角点在拍摄第一张图的相机坐标系下的三维坐标
    pc1 = deproject(intrinsics,[pg1[i][0],pg1[i][1]],d/1000)
    pc1.append(1)
    for j in range(i+1,len(pg1)):
        # 根据公式计算把坐标转换到第二张图的相机坐标系下
        pc2 = np.dot(np.dot(np.dot(np.dot(X,np.linalg.inv(As[j])),As[i]),np.linalg.inv(X)),np.array(pc1).T)
        # 进行重投影
        px = project(intrinsics,pc2)
        #计算误差
        e = np.array(px)-np.array(pg1[j])
        error1 = error1+np.power(np.linalg.norm(e),2)
        cnt1 = cnt1+1

ave_e = np.power(error1/cnt1,0.5)
print(ave_e)

# 同理对第二组进行处理
error2 = 0
cnt2 = 0
for i in range(len(pg2)):
    d = depths[i+6][pg2[i][1]][pg2[i][0]]
    if d==0 or d>2000:
        continue
    pc1 = deproject(intrinsics,[pg2[i][0],pg2[i][1]],d/1000)
    pc1.append(1)
    for j in range(i+1,len(pg2)):
        pc2 = np.dot(np.dot(np.dot(np.dot(X,np.linalg.inv(As[j+6])),As[i+6]),np.linalg.inv(X)),np.array(pc1).T)
        px = project(intrinsics,pc2)
        e = np.array(px)-np.array(pg2[j])
        error2 = error2+np.power(np.linalg.norm(e),2)
        cnt2 = cnt2+1

ave_e = np.power(error2/cnt2,0.5)
print(ave_e)
