#!/bin/bash

# USAGE
# ./SH/learn_env.sh

demos_path=`rospack find color_voxel_recognition`/demos
dir_name=$demos_path/scene_`date '+%Y%m%d%H%M%S'`
display_config_file=$demos_path/display_config

# save point clouds of scene (= environment)
roslaunch color_voxel_recognition save_data_and_view.launch /dir_name:=$dir_name /input:=/camera/rgb/points /display_config:=$display_config_file

if [ $(ls $dir_name/Points | wc -l) -eq 0 ]
then
    rm -rf $dir_name
else
# prepare directory
    rm -f $demos_path/scene 2>/dev/null
    ln -s $dir_name $demos_path/scene
    mkdir -p $demos_path/scene/Features
    
# calculate threshold for RGB binarize
    rosrun color_voxel_recognition calc_scene_auto_threshold $demos_path $(ls $demos_path/scene/Points | wc -l)
    
# extract C3_HLAC features
    rosrun color_voxel_recognition extract_c3_hlac_scene $demos_path $(ls $demos_path/scene/Points | wc -l)
    
# get projection axis for feature compression
    rosrun color_voxel_recognition pca_scene $demos_path $(ls $demos_path/scene/Features | wc -l)
fi
