import pyransac3d as pyrsc
from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
from time import time
import os
from RANSAC import visualize,from_ply,RANSAC,write_fly
import argparse
import random
import math
from internal_support import split_array
import matplotlib.pyplot as plt


root = "E:\\基坑点云\\Rhino\\steel_column\\"

LOG_DIR = root
LOG_FOUT = open(os.path.join(LOG_DIR, 'log_RANSAC_steel.txt'), 'w')


def log_string(out_str):
    LOG_FOUT.write(out_str+'\n')
    LOG_FOUT.flush()
    print(out_str)

def plot(points,a,b):
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1, 1)
    ax1.scatter(points[:,0], points[:,1], s=1)
    plt.gca().set_aspect(1)#轴比例相等
    x = np.linspace(np.min(points[:,0])-0.2,np.max(points[:,0])+0.2)
    for i in range(len(a)):
        y = a[i]*x+b[i]
        plt.plot(x,y,linewidth=1)
    X,Y = cross_points(a,b)
    ax1.scatter(X, Y, s=20, marker = '^',color = 'red')
    center_x = np.mean(X)
    center_y = np.mean(Y)
    log_string("X："+str(X)+" Y:"+str(Y)+"Center point (%f,%f)"%(center_x,center_y))

    plt.show()


def slice(points,N,theta):
    zmax = np.max(points[:,2])
    zmin = np.min(points[:,2])
    length = (zmax-zmin)/(N+1) #one slice length
    Loop = []
    Z = []
    for i in range(N):
        single_loop = []
        z_slice = zmin + (i+1)*length
        for j in range(points.shape[0]):
            if (points[j,2]>=(z_slice-theta/2) and points[j,2]<=(z_slice+theta/2)):
                single_loop.append(points[j,:])
        if len(single_loop)>= 250:
            Loop.append(np.array(single_loop))
            Z.append(z_slice)
    return np.array(Loop),Z

def fit_line_by_ransac(point_list, sigma, iters = 1000):
    # 使用RANSAC算法拟合直线
    # 迭代最大次数 iters = 1000
    # 数据和模型之间可接受的差值 sigma
    # 希望的得到正确模型的概率P = 0.99
    a_array = np.zeros(4)
    b_array = np.zeros(4)
    for j in range(4):
        n_total = 0#内点数目
        for i in range(iters):
            # 随机选两个点去求解模型
            sample_index = random.sample(range(len(point_list)), 2)
            x_1 = point_list[sample_index[0]][0]
            y_1 = point_list[sample_index[0]][1]
            x_2 = point_list[sample_index[1]][0]
            y_2 = point_list[sample_index[1]][1]
            if x_2 == x_1:
                continue
                
            # y = ax + b 求解出a，b
            a = (y_2 - y_1) / (x_2 - x_1)
            b = y_1 - a * x_1

            # 算出内点数目,返回内点
            total_inlier = 0
            inlier = []
            for index in range(len(point_list)):
                y_estimate = a * point_list[index][0] + b
                if abs(y_estimate - point_list[index][1]) < sigma:
                    total_inlier += 1
                    inlier.append(index)

            # 判断当前的模型是否比之前估算的模型好
            if total_inlier > n_total:
                # iters = math.log(1 - P) / math.log(1 - pow(total_inlier/len(point_list), 2))
                n_total = total_inlier
                final_inlier = inlier
                a_array[j] = a
                b_array[j] = b
        point_list = split_array(point_list, final_inlier)
    return a_array, b_array

def cross_points(a,b):
    X = []
    Y = []
    N = range(1,4)
    for j in range(3):
        for i in range(N[j]):
            delta = a[i]-a[i+4-N[j]]
            if abs(delta)>0.2:
                x = (-b[i]+b[i+4-N[j]])/delta
                X.append(x)
                y = a[i]*x+b[i]
                Y.append(y)
    return(X,Y)
    # for i in range(3):
    #     delta = a[i]-a[i+1]
    #     if abs(delta)>0.2:
    #         x = (b[i]-b[i+1])/delta
    #         X.append(x)
    #         Y.append(a[i]*x+b[i])
    # for i in range(2):
    #     delta = a[i]-a[i+2]
    #     if abs(delta)>0.2:
    #         x = (b[i]-b[i+2])/delta
    #         X.append(x)
    #         Y.append(a[i]*x+b[i])
    # for i in range(1):
    #     delta = a[i]-a[i+3]
    #     if abs(delta)>0.2:
    #         x = (b[i]-b[i+3])/delta
    #         X.append(x)
    #         Y.append(a[i]*x+b[i])
    # return


if __name__ == '__main__':
    filename = root + "steel_column_1.ply"
    points = from_ply(filename)
    Loop, Z = slice(points,9,0.03)
    for i in range(Loop.shape[0]):
        # print(Loop[i].shape)
        a, b= fit_line_by_ransac(Loop[i][:,0:2],sigma = 0.01)
        # print(a,b)
        log_string("Slice_%i: "%i+"Z:%f"%Z[i]+"a: "+str(a)+" b: "+str(b))
        plot(Loop[i],a,b)
