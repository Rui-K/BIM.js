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

root = "E:\\基坑点云\\Rhino\\internal_support\\"

LOG_DIR = root
LOG_FOUT = open(os.path.join(LOG_DIR, 'log_RANSAC_line.txt'), 'w')
LOG_FOUT.write(str(FLAGS)+'\n')
def log_string(out_str):
    LOG_FOUT.write(out_str+'\n')
    LOG_FOUT.flush()
    print(out_str)

def extract_plane(points,outname):
    # visualize(points,True)
    best_eq, best_inliers = RANSAC(points, 'plane',thresh = thresh, maxIteration=maxIteration)
    visualize(points[best_inliers,:],True)
    write_fly(os.path.join(root,outname),points[best_inliers,:])

def split_array(array,index):
    """input array and the index of element do NOT need, return remain array"""
    remain_index = []
    for i in range(array.shape[0]):
        if i not in index:
            remain_index.append(i)
    return array[remain_index,:]

def to_points(array):
    points = []
    for i in range(array.shape[0]):
        for point in array[i]:
            points.append(point.tolist())
    return np.array(points)
    

def extract_line(points):
    NUM = points.shape[0]
    remain = points
    lines = []
    i = 0
    remain_percent = len(remain)/NUM
    while(remain_percent>0.3):
        a, b, in_liers = RANSAC(remain,'line',thresh = thresh, maxIteration=maxIteration)
        if len(in_liers)<100:
            print("Too less points, skip")
            continue
        log_string("Line %i: "%i + "A: "+str(a)+" B: "+str(b)+" remain percent: "+str(remain_percent))
        lines.append(remain[in_liers,:])
        remain = split_array(remain, in_liers)
        remain_percent = len(remain)/NUM
        i += 1
    return to_points(np.array(lines))

if __name__ == '__main__':
    filename = os.path.join(root,"boundary.ply")
    points = from_ply(filename)
    lines = extract_line(points)
    # visualize(lines)
    write_fly(root+"lines.ply",lines)
    LOG_FOUT.close()