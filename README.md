# roadpatch-with-pothole-generator
This is an automated script to ease the workload of creating potholes for the purpose of inserting them into a Carla map

## Dependencies

The code uses the following packages
- numpy
- open3d
- sklearn

## How to use

The ```pothole.py``` script is responsible for reading the pointcloud of the pothole and adding a flat plane around it to simulate
a road patch. For this to happen, it needs to be supplied with the folder of the .ply files from this repo as its input [https://github.com/ruirangerfan/rethinking_road_reconstruction_pothole_detection/tree/main/dataset/model1].

The script afterwards outputs the result in completed potholes and also populates the intermediate_files with the last processed pointcloud (for debug purposes).

The ``` eval.py ``` script, in turn reads the groundtruth pointcloud (the version with the road patch generated from ```pothole.py```)
and the pointcloud generated from this repo [https://github.com/Stagakis/saliency-from-pointcloud] and outputs a confusion matrix for the point classification (pothole and road classes).
