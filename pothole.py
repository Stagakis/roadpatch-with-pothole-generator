from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
from random import randrange, uniform, seed
from datetime import datetime
import glob

seed(datetime.now())

def readPointCloud(filename):
    with open(filename, 'rb') as f:
        plydata = PlyData.read(f)

    vertices = plydata.elements[0].data
    vertices = [ [vert[0], vert[1], vert[2]] for vert in vertices ]  
    return np.asarray(vertices, dtype=np.float32)

def savePointCloud(filename, xyz, colors=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    if(colors is not None):
        pcd.colors = colors
    o3d.io.write_point_cloud(filename, pcd)

def downSamplePointCloud(xyz, mvoxel_size):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    downpcd = pcd.voxel_down_sample(voxel_size=mvoxel_size)
    return np.asarray(downpcd.points)

def boundingBoxXYplane(xyz):
    min_x, max_x = min(xyz[:,0]), max(xyz[:,0])
    min_y, max_y = min(xyz[:,1]), max(xyz[:,1])
    width = max_x - min_x
    height = max_y - min_y
    return width, height, min_x, max_x, min_y, max_y

def isPointWithinRange(point, xyz, threshold):
    for p in xyz:
        dist = np.linalg.norm(point[:2]-p[:2])
        if (dist <= threshold):
            return True
    return False

def checkOverlap(min_x, max_x, min_y, max_y, point, xyz):
    i = point[0]
    j = point[1]
    if ( (i<min_x or i>max_x) or (j < min_y or j > max_y)):
        return True
    return not isPointWithinRange(point, xyz, 2)


potholes = glob.glob("../rethinking_road_reconstruction_pothole_detection/dataset/model2/gt/*.ply")

for pothole_filename in potholes:
    print("Reading filename: " + pothole_filename)
    pothole = readPointCloud(pothole_filename)
    road = readPointCloud('road.ply')

    #Find bounding box of road
    width, height, road_min_x, road_max_x, road_min_y, road_max_y = boundingBoxXYplane(road)
    z = road[0,2]

    #Shift top of hole to align with road and add a little bit of randomness
    print("Processing pothole file...")
    pothole *= [1, 1, -1] #Flip pothole in the z axis.
    pothole = downSamplePointCloud(pothole, 2) #Just too many points, the program takes forever
    pothole = pothole / uniform(1.2,2.0) #The hole is too big
    max_z = max(pothole[:,2])
    shift_z = z - max_z - 1
    pothole += [0, 0, shift_z]
    pothole += [ randrange(-int(width/4),int(width/4)),  randrange(-int(height/4),int(height/4)), 0 ]
    savePointCloud("intermediate_files/pothole.ply", pothole)

    #Find bounding box of pothole
    pothole_width, pothole_height, min_x, max_x, min_y, max_y = boundingBoxXYplane(pothole)

    #Generate dense road pointcloud
    print("Processing road file...")
    step = 1
    dense_road = [ [i, j, z] 
    for i in range(int(-width/2),int(width/2),step) 
    for j in range(int(-height/2), int(height/2), step) 
    if ( checkOverlap(min_x, max_x, min_y, max_y, [i,j,z], pothole) )]
    dense_road = np.asarray(dense_road, dtype=np.float32)
    road = downSamplePointCloud(dense_road, 2) #to have homogeneous density

    savePointCloud("intermediate_files/dense_road.ply", dense_road)



    print("Finalizing...")    
    label_pothole = np.ones(pothole.shape[0])
    color_pothole = [ [1, 0, 0] for i in range(pothole.shape[0])]
    label_road = np.zeros(road.shape[0])
    color_road = [ [0, 0, 1] for i in range(pothole.shape[0])]

    final_xyz = np.concatenate( (pothole, road), axis=0)    
    final_labels = np.concatenate( (label_pothole, label_road), axis=0)    
    final_colors = np.concatenate( (color_pothole, color_road), axis=0)

    print("Saving to disk...") 
    np.savetxt("intermediate_files/label.txt", final_labels, fmt="%1.1d")
    np.savetxt("completed_potholes/" + pothole_filename.split('/')[-1][:-4] + "_label.txt", final_labels, fmt="%1.1d")

    savePointCloud("intermediate_files/final.ply", final_xyz)
    savePointCloud("completed_potholes/" + pothole_filename.split('/')[-1], final_xyz)


    savePointCloud("intermediate_files/colored_final.ply", final_xyz, final_colors)
    savePointCloud("completed_potholes/colored_" + pothole_filename.split('/')[-1], final_xyz, final_colors)
