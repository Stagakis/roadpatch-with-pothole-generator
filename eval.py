from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
import glob
from sklearn.metrics import confusion_matrix


def readPointCloud(filename):
    if(filename[len(filename)-4:] == ".ply"):
        with open(filename, 'rb') as f:
            plydata = PlyData.read(f)
        vertices = plydata.elements[0].data

        pos = [ [vert[0], vert[1], vert[2]] for vert in vertices ]
        colors = [ [vert[3], vert[4], vert[5]] for vert in vertices ]

        return np.asarray(pos, dtype=np.float32), np.asarray(colors, dtype=np.uint8)
    else:
        pos = []
        col = []
        with open(filename, 'r') as f:
            lines = f.readlines()
            for line in lines:
                line_parts = line.replace('\n', '').split(' ')[1:]
                pos.append([float(line_parts[0]), float(line_parts[1]), float(line_parts[2])])
                col.append([float(line_parts[3]), float(line_parts[4]), float(line_parts[5])])


        pos = np.asarray(pos)
        pos.reshape(-1,3)

        col = np.asarray(col)*255
        col = np.asarray(col, dtype=np.uint8)
        col.reshape(-1,3)
        return pos, col



ground_truth_file_name_template = "colored_model_*.ply"
files = glob.glob("completed_potholes/"+ground_truth_file_name_template)
for file in files:
    print(file)
    gt_pcl, gt_col = readPointCloud(file)
    res_pcl, res_col = readPointCloud(file[:-4] + "_eigen_binary.obj")


    #Calculation of accuracy

    gt = ["hole" if col.tolist() == [255,0,0] else "road" for col in gt_col]
    res = ["hole" if col.tolist() == [255,0,0] else "road" for col in res_col ]
    conf = confusion_matrix(gt, res, normalize='true')
    print(conf)
    #confusion_matrix(gt_col, res_col)

    #gt = gt_col.tolist()
    #res = res_col.tolist()
    #tp = np.sum([gt[i] == res[i] == [255,0,0] for i in range(len(gt))])/np.sum([col == [255,0,0] for col in gt])
    #tp = np.sum([gt[i] == res[i] == [255,0,0] for i in range(len(gt))])/np.sum([col == [255,0,0] for col in gt])
    #print("True Positives: ",tp)

