from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
from random import randrange, uniform
import glob

def readPointCloud(filename):
    with open(filename, 'rb') as f:
        plydata = PlyData.read(f)

    vertices = plydata.elements[0].data
    vertices = [ [vert[0], vert[1], vert[2]] for vert in vertices ]  
    return np.asarray(vertices, dtype=np.float32)

def savePointCloud(filename, xyz):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    o3d.io.write_point_cloud(filename, pcd)

def downSamplePointCloud(xyz):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    downpcd = pcd.voxel_down_sample(voxel_size=2)
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
    return not isPointWithinRange(point, xyz, 5)


potholes = glob.glob("../rethinking_road_reconstruction_pothole_detection/dataset/model1/gt/*.ply")

for pothole_filename in potholes:
    pothole = readPointCloud(pothole_filename)
    road = readPointCloud('road.ply')

    #Find bounding box of road
    width, height, road_min_x, road_max_x, road_min_y, road_max_y = boundingBoxXYplane(road)
    z = road[0,2]

    #Shift top of hole to align with road and add a little bit of randomness
    pothole = downSamplePointCloud(pothole) #too many points
    pothole *= [1, 1, -1] #Flip pothole in the z axis
    pothole = pothole / uniform(1.2,2.0) #The hole is too big
    max_z = max(pothole[:,2])
    shift_z = z - max_z - 1
    pothole += [0, 0, shift_z]
    pothole += [ randrange(-int(width/3),int(width/3)),  randrange(-int(height/3),int(height/3)), 0 ]
    savePointCloud("intermediate_files/pothole.ply", pothole)

    #Find bounding box of pothole
    pothole_width, pothole_height, min_x, max_x, min_y, max_y = boundingBoxXYplane(pothole)

    #Generate a denser road pointcloud
    #print("Width: ",width)
    #print("Height: ",height)

    step = 2
    #dense_road = [ [i, j, z] for i in range(int(-width/2),int(width/2),step) for j in range(int(-height/2), int(height/2), step) if ( (i<min_x or i>max_x) or (j < min_y or j > max_y))]
    dense_road = [ [i, j, z] 
    for i in range(int(-width/2),int(width/2),step) 
    for j in range(int(-height/2), int(height/2), step) 
    if ( checkOverlap(min_x, max_x, min_y, max_y, [i,j,z], pothole) )]

    dense_road = np.asarray(dense_road, dtype=np.float32)
    savePointCloud("intermediate_files/dense_road.ply", dense_road)
    #print(dense_road)

    final_xyz = np.concatenate( (pothole, dense_road), axis=0)
    savePointCloud("intermediate_files/final.ply", final_xyz)
    savePointCloud("completed_potholes/" + pothole_filename.split('/')[-1], final_xyz)
