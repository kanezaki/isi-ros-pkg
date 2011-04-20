cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/param.cpp src/pca.cpp src/search.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(calc_scene_auto_threshold test/calc_scene_auto_threshold.cpp)
rosbuild_add_executable(extract_c3_hlac_scene test/extract_c3_hlac_scene.cpp)
rosbuild_add_executable(extract_c3_hlac_models test/extract_c3_hlac_models.cpp)
rosbuild_add_executable(pca_scene test/pca_scene.cpp)
rosbuild_add_executable(pca_models test/pca_models.cpp)
rosbuild_add_executable(variance_show test/variance_show.cpp)
rosbuild_add_executable(save_data test/save_data.cpp)
rosbuild_add_executable(detect_object test/detect_object.cpp)
##### rosbuild_add_executable(get_voxel_model test/get_voxel_model.cpp)
##### rosbuild_add_executable(get_voxel_scene test/get_voxel_scene.cpp)
target_link_libraries(calc_scene_auto_threshold ${PROJECT_NAME})
target_link_libraries(extract_c3_hlac_scene ${PROJECT_NAME})
target_link_libraries(extract_c3_hlac_models ${PROJECT_NAME})
target_link_libraries(pca_scene ${PROJECT_NAME})
target_link_libraries(pca_models ${PROJECT_NAME})
target_link_libraries(variance_show ${PROJECT_NAME})
target_link_libraries(detect_object ${PROJECT_NAME})
##### target_link_libraries(get_voxel_model ${PROJECT_NAME})
##### target_link_libraries(get_voxel_scene ${PROJECT_NAME})
