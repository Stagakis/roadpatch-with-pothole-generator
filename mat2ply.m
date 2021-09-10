mat_propath = '/home/stagakis/Desktop/road_with_pothole_generation/stereo_pothole_datasets/dataset1/ptcloud/';
image_propath = '/home/stagakis/Desktop/road_with_pothole_generation/stereo_pothole_datasets/dataset1/label/';

mat_files = dir(strcat(mat_propath,'*.mat'));
image_files = dir(strcat(image_propath,'*.png'));

focal_length = -0.0304996; %30.4996 Mm
camera_intrinsics = [focal_length 0 0 0; 0 focal_length 0 0; 0 0 1 0];
camera_etxrinsics = [1 0 0 0; 0 1 0 0 ; 0 0 1 0; 0 0 0 1];
camera_matrix = camera_intrinsics * camera_etxrinsics;
for k = 1:size(mat_files)
    k
    
    mat_fullpath = strcat(mat_files(k).folder, '/', mat_files(k).name);
    mat = load(mat_fullpath);
    
    image_fullpath = strcat(image_files(k).folder, '/', image_files(k).name);
    image_label = imread(image_fullpath);
    
    mat.xyzPoints(:,:,3) = -mat.xyzPoints(:,:,3);
    mat.xyzPoints(:,:,2) = -mat.xyzPoints(:,:,2);
    ptCloud = pointCloud(mat.xyzPoints);    
    
    
    for i = 1:size(image_label,1)
        for j = 1:size(image_label,2)
           if(image_label(i,j,:) == [0 0 0])
               image_label(i,j,:) = [0 0 255];
           else
               image_label(i,j,:) = [255 0 0];
           end
            
        end
    end
    
    ptCloud.Color = image_label;    
    
    pcwrite(ptCloud, strcat(mat_fullpath(1:size(mat_fullpath,2)-4),".ply"));
    
    %break;
end
    