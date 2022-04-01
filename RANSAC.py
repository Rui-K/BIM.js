import pyransac3d as pyrsc
from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
from time import time
import os
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--thresh', type=float, default=0.05, help='threshold [default:0.05]')
parser.add_argument('--maxIteration', type=int, default=1000, help='maxIteration [default:1000]')
FLAGS = parser.parse_args()
thresh = FLAGS.thresh
maxIteration = FLAGS.maxIteration

LOG_DIR = "C:\\Users\\kr971\\OneDrive\\硕士毕业设计\\code"
LOG_FOUT = open(os.path.join(LOG_DIR, 'log_RANSAC.txt'), 'w')
LOG_FOUT.write(str(FLAGS)+'\n')

def log_string(out_str):
    LOG_FOUT.write(out_str+'\n')
    LOG_FOUT.flush()
    print(out_str)

def visualize(points, with_color=False):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:,0:3].reshape(-1, 3))
    if with_color:
    # 颜色归一化处理
        pcd.colors = o3d.utility.Vector3dVector((points[:,3:6]/255).reshape(-1, 3))
    o3d.visualization.draw_geometries([pcd])


def from_ply(root, with_color=False):
    """from root import ply file and return numpy array Nx3 and with color"""
    ply_file = PlyData.read(root)
    vertex = ply_file['vertex']
    if with_color:
        points = np.stack((vertex['x'], vertex['y'], vertex['z'], 
                            vertex['red'], vertex['green'], vertex['blue']), axis = -1).astype(np.float32)
    else:
        points = np.stack((vertex['x'], vertex['y'],
                        vertex['z']), axis = -1).astype(np.float32)
    
    return points
def write_fly(path, points):
    
    color = [(points[i,3], points[i,4], points[i,5]) for i in range(points.shape[0])]
    points = [(points[i,0], points[i,1], points[i,2]) for i in range(points.shape[0])]
    points = PlyElement.describe(np.array(points,dtype=[('x', 'f4'), ('y', 'f4'),('z', 'f4')]), 'vertex')
    color = PlyElement.describe(np.array(color,dtype=[('red', 'u1'), ('green', 'u1'),('blue', 'u1')]),'color')
    PlyData([points,color]).write(path)


def RANSAC(points, type='plane',thresh=0.05, maxIteration=1000):
    """input numpy points, shape type want to fit, return equation parameters and inliers
        type includes 'plane','cylinder','cuboid'
    """
    data_xyz = points[:,0:3]
    if type=='plane':
        plane = pyrsc.Plane()
        best_eq, best_inliers = plane.fit(data_xyz, thresh=thresh, maxIteration=maxIteration)    
        return best_eq, best_inliers
    elif type=='cylinder':
        cylinder = pyrsc.Cylinder()
        center,axis,radius,inlier = cylinder.fit(data_xyz, thresh=thresh, maxIteration=maxIteration)
        return center,axis,radius,inlier
    elif type=='cuboid':
        cuboid = pyrsc.Cuboid()
        best_eq, best_inliers = cuboid.fit(data_xyz, thresh=thresh, maxIteration=maxIteration)
        return best_eq, best_inliers
    

if __name__ =='__main__':
    root = "E:\\基坑点云\\Rhino"
    for i in range(1,3):
        for j in range(1,4):
            filename = os.path.join(root, "wall_%i_%i.ply"%(i,j))
            points = from_ply(filename, with_color=True)
            best_eq, best_inliers = RANSAC(points, 'plane',thresh = thresh, maxIteration=maxIteration)   
            log_string("Normal Vector of wall_%i_%i.ply"%(i,j)+": (%f,%f,%f)"%(best_eq[0],best_eq[1],best_eq[2]))
            write_fly(os.path.join(root,"wall_%i_%i_extract.ply"%(i,j)),points[best_inliers,:])
            # visualize(points[best_inliers,:], with_color=True)

    LOG_FOUT.close()
