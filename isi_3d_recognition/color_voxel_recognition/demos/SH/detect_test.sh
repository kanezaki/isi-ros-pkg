#!/bin/bash

# USAGE
# e.g. ./SH/detect_test.sh phone1 0.5 0.2

demos_path=`rospack find color_voxel_recognition`/demos
model_name=$1
similarity_th=$2
rank_num=1 #10
exist_voxel_num_threshold=100
r_dim=20
pca=$demos_path/models/$model_name/pca_result
#size=(`cat $demos_path/models/$model_name/size.txt`)
size1=$3
distance_th=3
display_config_file=$demos_path/display_config

#rosrun color_voxel_recognition detect_object $rank_num $exist_voxel_num_threshold $pca $r_dim $size1 $size1 $size1 $similarity_th $distance_th /input:=/camera/rgb/points
roslaunch color_voxel_recognition detect.launch /demos_path:=$demos_path /rank_num:=$rank_num /exist_voxel_num_threshold:=$exist_voxel_num_threshold /pca:=$pca /r_dim:=$r_dim /size1:=$size1 /size2:=$size1 /size3:=$size1 /similarity_th:=$similarity_th /distance_th:=$distance_th /input:=/camera/rgb/points /display_config:=$display_config_file
