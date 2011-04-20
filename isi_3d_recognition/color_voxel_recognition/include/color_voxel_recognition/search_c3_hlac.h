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

#ifndef COLOR_VOXEL_RECOGNITION_SEARCH_C3_HLAC_H_
#define COLOR_VOXEL_RECOGNITION_SEARCH_C3_HLAC_H_
#include <c3_hlac/c3_hlac_tools.h>
#include "color_voxel_recognition/search.h"

////////////
// single //
////////////

class SearchC3HLAC : public SearchObj {
public:
  SearchC3HLAC(){}
  ~SearchC3HLAC(){}
  void setC3HLAC( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGB> grid, pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

//////////

void SearchC3HLAC::setC3HLAC( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGB> grid, pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  std::vector< std::vector<float> > c3hlac;
  Eigen::Vector3i subdiv_b_ = extractC3HLACSignature981( grid, cloud_downsampled, c3hlac, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );

  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ )
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // the number of occupied voxels before adding up
  setData( subdiv_b_, c3hlac );
}

///////////
// multi //
///////////

class SearchC3HLACMulti : public SearchObjMulti {
public:
  SearchC3HLACMulti(){}
  ~SearchC3HLACMulti(){}
  void setC3HLAC( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGB> grid, pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

//////////

void SearchC3HLACMulti::setC3HLAC( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGB> grid, pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  std::vector< std::vector<float> > c3hlac;
  Eigen::Vector3i subdiv_b_ = extractC3HLACSignature981( grid, cloud_downsampled, c3hlac, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );

  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ )
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // the number of occupied voxels before adding up
  setData( subdiv_b_, c3hlac );
}

#endif
