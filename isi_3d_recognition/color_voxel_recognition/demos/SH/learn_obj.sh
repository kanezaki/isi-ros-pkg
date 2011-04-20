#!/bin/bash

# USAGE
# e.g. ./SH/learn_obj.sh phone1

limit_depth=0.1
demos_path=`rospack find color_voxel_recognition`/demos
display_config_file=$demos_path/display_config

mkdir -p $demos_path/models_online
rm $demos_path/models
ln -s $demos_path/models_online $demos_path/models
cp $demos_path/scene/pca_result $demos_path/models/compress_axis

# delete object's data if there are
if [ $(ls -d $demos_path/models/$1 2>/dev/null | wc -l ) = 1 ]
then
    rm -I -r $demos_path/models/$1
fi

# save point clouds of the target object
roslaunch color_voxel_recognition save_data_and_view.launch /dir_name:=$demos_path/models/$1 /limit_depth:=$limit_depth /input:=/camera/rgb/points /display_config:=$display_config_file

if [ $(ls $demos_path/models/$1/Points | wc -l) -eq 0 ]
then
    rm -rf $demos_path/models/$1
else
# prepare directory
    mkdir -p $demos_path/models/$1/Features
    
# extract C3_HLAC features
    rosrun color_voxel_recognition extract_c3_hlac_models $demos_path $1 $(ls $demos_path/models/$1/Points | wc -l)
    
# get projection axis of the target object's subspace
    rosrun color_voxel_recognition pca_models $demos_path $1 $(ls $demos_path/models/$1/Features | wc -l)
fi