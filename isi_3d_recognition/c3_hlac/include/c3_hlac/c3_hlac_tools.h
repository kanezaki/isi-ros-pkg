/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Asako Kanezaki <kanezaki@isi.imi.i.u-tokyo.ac.jp>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The University of Tokyo nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef C3_HLAC_TOOLS_H_
#define C3_HLAC_TOOLS_H_

#include <sys/time.h>
#include "c3_hlac/c3_hlac.h"
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>

//-----------
//* time
double t1,t2;
double my_clock();

//-----------
//* read
void readFeature(const char *name, std::vector< std::vector<float> > &feature );
void readFeature(const char *name, std::vector<float> &feature );

//-----------
//* write
bool ifZeroVec( const std::vector<float> vec );
void writeFeature(const char *name, const std::vector< std::vector<float> > feature, bool remove_0_flg = true );
void writeFeature(const char *name, const std::vector<float> feature, bool remove_0_flg = true );

//-----------
//* voxelize
template <typename T>
void getVoxelGrid( pcl::VoxelGrid<T> &grid, pcl::PointCloud<T> input_cloud, pcl::PointCloud<T>& output_cloud, const float voxel_size );

//------------------------
//* extract - C3HLAC -
//*    color_threshold_r : threshold for R value
//*    color_threshold_g : threshold for G value
//*    color_threshold_b : threshold for B value
//*    voxel_size : the length of voxel side (unit: meter)
//*    subdivision_size : the number of voxels in each subdivision's side
//*                       set 0 if you want to extract one C3_HLAC vector from the whole region
//*    offset_x : the number of overlapping voxels in neighboring subdivisions in x-axis
//*    offset_y : the number of overlapping voxels in neighboring subdivisions in y-axis
//*    offset_z : the number of overlapping voxels in neighboring subdivisions in z-axis

template <typename PointT>
Eigen::Vector3i extractC3HLACSignature981(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector< std::vector<float> > &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0 );

template <typename PointT>
void extractC3HLACSignature981(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector<float> &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size );

template <typename PointT>
Eigen::Vector3i extractC3HLACSignature117(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector< std::vector<float> > &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0 );

template <typename PointT>
void extractC3HLACSignature117(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector<float> &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size );

#include <c3_hlac/c3_hlac_tools.hpp>

#endif  //#ifndef C3_HLAC_TOOLS_H_
