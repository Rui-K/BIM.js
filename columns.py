import pyransac3d as pyrsc
from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
from time import time
import os
from RANSAC import visualize,from_ply,RANSAC,write_fly
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--thresh', type=float, default=0.05, help='threshold [default:0.05]')
parser.add_argument('--maxIteration', type=int, default=1000, help='maxIteration [default:1000]')
FLAGS = parser.parse_args()
thresh = FLAGS.thresh
maxIteration = FLAGS.maxIteration

root = "E:\\基坑点云\\Rhino\\column\\"

LOG_DIR = root
LOG_FOUT = open(os.path.join(LOG_DIR, 'log_RANSAC_column_locate.txt'), 'w')
LOG_FOUT.write(str(FLAGS)+'\n')

def log_string(out_str):
    LOG_FOUT.write(out_str+'\n')
    LOG_FOUT.flush()
    print(out_str)

def slice(points,N,theta):
    """points as input data, N as how many circles return, theta as thickness of circle"""
    zmax = np.max(points[:,2])
    zmin = np.min(points[:,2])
    length = (zmax-zmin)/(N+1) #one slice length
    circles = []
    vis = []
    for i in range(N):
        single_circle = []
        z_slice = zmin + (i+1)*length
        for j in range(points.shape[0]):
            if (points[j,2]>=(z_slice-theta/2) and points[j,2]<=(z_slice+theta/2)):
                single_circle.append(points[j,:].tolist())
                vis.append(points[j,:])
        circles.append(single_circle)
    return np.array(vis),circles

def locate(root):
    for i in range(6):
        filename = os.path.join(root, "column_%i.ply"%i)
        column = from_ply(filename)
        _, circles = slice(column,3,0.03)
        avg_center, avg_radius = fit(circles)
        log_string("Column_%i avg_center: "%i+str(avg_center)+" avg_radius: "+str(avg_radius))

def fit(circles):
    Center = np.zeros((3))
    Radius = 0
    for i in range(len(circles)):
        center,axis,radius,inlier = RANSAC(np.array(circles[i]),type='circle',thresh=thresh, maxIteration=maxIteration)
        write_fly(root+"circle_%i.ply"%i,np.array(circles[i])[inlier,:])
        # visualize(np.array(circles[i])[inlier_,:],with_color=True)
        # log_string("------Circle %i------"%i)
        # log_string("center: "+str(center))
        # log_string("axis: "+str(axis))
        # log_string("radius: %f"%radius)
        Center += center
        Radius += radius
    Center = Center/(len(circles))
    Radius = Radius/(len(circles))
    return Center, Radius

if __name__ == '__main__':
    # filename = root+"column.ply"
    # points = from_ply(filename, with_color=True)
    s = time()
    locate(root)
    # vis, circles = slice(points,8,0.05)
    # visualize(vis,with_color=True)
    # write_fly(root+"circles.ply",vis)
    # fit(circles)

    e = time()

    log_string("time cost:%f"%(e-s))
    LOG_FOUT.close()

