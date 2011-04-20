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

#include <c3_hlac_core/c3_hlac_core.h>
#include "c3_hlac/c3_hlac.h"

const float NORMALIZE_0 = 1/255.0;    // value for normalizing 0th-order C3_HLAC (without RGB binalize)
const float NORMALIZE_1 = 1/65025.0;  // value for normalizing 1st-order C3_HLAC (without RGB binalize)
const float NORMALIZE_0_BIN = 1;    // value for normalizing 0th-order C3_HLAC (with RGB binalize)
const float NORMALIZE_1_BIN = 1;    // value for normalizing 1st-order C3_HLAC (with RGB binalize)
const float NORMALIZE_117_0 = 1/255.0;    // value for normalizing 0th-order C3_HLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_117_1 = 1/845325.0; // value for normalizing 1st-order C3_HLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_117_0_BIN = 1;    // value for normalizing 0th-order C3_HLAC (with RGB binalize) - rotation-invariant -
const float NORMALIZE_117_1_BIN = 1/13.0; // value for normalizing 1st-order C3_HLAC (with RGB binalize) - rotation-invariant -

//*********************************************************************************
//* calculate a feature vector when the target voxel data is rotated by 90 degrees.
void pcl::rotateFeature90( std::vector<float> &output, const std::vector<float> &input, RotateMode mode ){
  int dim = input.size();

  switch( dim ){
  case DIM_C3HLAC_981_BIN_1_3+DIM_C3HLAC_981_1_3:
    {
      if( output.size() != (std::vector<float>::size_type)dim )
	output.resize( dim );
      
      std::vector<float> tmp_input(DIM_C3HLAC_981_1_3);
      std::vector<float> tmp_output;
      for(int i=0;i<DIM_C3HLAC_981_1_3;i++)
	tmp_input[i] = input[i];
      rotateFeature90( tmp_output, tmp_input, mode );
      for(int i=0;i<DIM_C3HLAC_981_1_3;i++)
	output[i] = tmp_output[i];
      
      tmp_input.resize( DIM_C3HLAC_981_BIN_1_3 );
      for(int i=0;i<DIM_C3HLAC_981_BIN_1_3;i++)
	tmp_input[i] = input[i+DIM_C3HLAC_981_1_3];
      rotateFeature90( tmp_output, tmp_input, mode );
      for(int i=0;i<DIM_C3HLAC_981_BIN_1_3;i++)
	output[i+DIM_C3HLAC_981_1_3] = tmp_output[i];    
      break;
    }
    
  case DIM_C3HLAC_981_BIN_1_3:
  case DIM_C3HLAC_981_1_3:
    if( output.size() != (std::vector<float>::size_type)dim )
      output.resize( dim );
    for(int i=0;i<6;i++)
      output[i]=input[i];
    for(int i=474;i<dim;i++)
      output[i]=input[i];
    
    switch( mode ){
    case R_MODE_1:
      for(int i=0;i<6;i++){
	for(int j=0;j<6;j++){
	  output[  8 + i*9 + j*78 ] = input[  6 + i*9 + j*78 ];
	  output[ 11 + i*9 + j*78 ] = input[  7 + i*9 + j*78 ];
	  output[ 14 + i*9 + j*78 ] = input[  8 + i*9 + j*78 ];
	  output[  7 + i*9 + j*78 ] = input[  9 + i*9 + j*78 ];
	  output[ 10 + i*9 + j*78 ] = input[ 10 + i*9 + j*78 ];
	  output[ 13 + i*9 + j*78 ] = input[ 11 + i*9 + j*78 ];
	  output[  6 + i*9 + j*78 ] = input[ 12 + i*9 + j*78 ];
	  output[  9 + i*9 + j*78 ] = input[ 13 + i*9 + j*78 ];
	  output[ 12 + i*9 + j*78 ] = input[ 14 + i*9 + j*78 ];
	  output[ 62 + i*4 + j*78 ] = input[ 60 + i*4 + j*78 ];
	  output[ 63 + j*4 + i*78 ] = input[ 61 + i*4 + j*78 ]; // Swapping j for i
	  output[ 60 + j*4 + i*78 ] = input[ 62 + i*4 + j*78 ]; // Swapping j for i
	  output[ 61 + i*4 + j*78 ] = input[ 63 + i*4 + j*78 ];
	}
      }
      break;
    case R_MODE_2:
      for(int i=0;i<6;i++){
	for(int j=0;j<6;j++){
	  output[  8 + i*9 + j*78 ] = input[  6 + i*9 + j*78 ];
	  output[ 62 + i*4 + j*78 ] = input[  7 + i*9 + j*78 ];
	  output[ 12 + j*9 + i*78 ] = input[  8 + i*9 + j*78 ]; // Swapping j for i
	  output[ 11 + i*9 + j*78 ] = input[  9 + i*9 + j*78 ];
	  output[ 63 + j*4 + i*78 ] = input[ 10 + i*9 + j*78 ]; // Swapping j for i
	  output[  9 + j*9 + i*78 ] = input[ 11 + i*9 + j*78 ]; // Swapping j for i
	  output[ 14 + i*9 + j*78 ] = input[ 12 + i*9 + j*78 ];
	  output[ 60 + j*4 + i*78 ] = input[ 13 + i*9 + j*78 ]; // Swapping j for i
	  output[  6 + j*9 + i*78 ] = input[ 14 + i*9 + j*78 ]; // Swapping j for i
	  output[  7 + i*9 + j*78 ] = input[ 60 + i*4 + j*78 ];
	  output[ 61 + i*4 + j*78 ] = input[ 61 + i*4 + j*78 ];
	  output[ 13 + j*9 + i*78 ] = input[ 62 + i*4 + j*78 ]; // Swapping j for i
	  output[ 10 + i*9 + j*78 ] = input[ 63 + i*4 + j*78 ];
	}
      }
      break;
    case R_MODE_3:
      for(int i=0;i<6;i++){
	for(int j=0;j<6;j++){
	  output[ 12 + i*9 + j*78 ] = input[  6 + i*9 + j*78 ];
	  output[ 13 + i*9 + j*78 ] = input[  7 + i*9 + j*78 ];
	  output[ 14 + i*9 + j*78 ] = input[  8 + i*9 + j*78 ];
	  output[ 62 + j*4 + i*78 ] = input[  9 + i*9 + j*78 ]; // Swapping j for i
	  output[ 61 + j*4 + i*78 ] = input[ 10 + i*9 + j*78 ]; // Swapping j for i
	  output[ 60 + j*4 + i*78 ] = input[ 11 + i*9 + j*78 ]; // Swapping j for i
	  output[  8 + j*9 + i*78 ] = input[ 12 + i*9 + j*78 ]; // Swapping j for i
	  output[  7 + j*9 + i*78 ] = input[ 13 + i*9 + j*78 ]; // Swapping j for i
	  output[  6 + j*9 + i*78 ] = input[ 14 + i*9 + j*78 ]; // Swapping j for i
	  output[  9 + i*9 + j*78 ] = input[ 60 + i*4 + j*78 ];
	  output[ 10 + i*9 + j*78 ] = input[ 61 + i*4 + j*78 ];
	  output[ 11 + i*9 + j*78 ] = input[ 62 + i*4 + j*78 ];
	  output[ 63 + i*4 + j*78 ] = input[ 63 + i*4 + j*78 ];
	}
      }
      break;
    case R_MODE_4:
      for(int i=0;i<6;i++){
	for(int j=0;j<6;j++){
	  output[ 12 + i*9 + j*78 ] = input[  6 + i*9 + j*78 ];
	  output[  9 + i*9 + j*78 ] = input[  7 + i*9 + j*78 ];
	  output[  6 + i*9 + j*78 ] = input[  8 + i*9 + j*78 ];
	  output[ 13 + i*9 + j*78 ] = input[  9 + i*9 + j*78 ];
	  output[ 10 + i*9 + j*78 ] = input[ 10 + i*9 + j*78 ];
	  output[  7 + i*9 + j*78 ] = input[ 11 + i*9 + j*78 ];
	  output[ 14 + i*9 + j*78 ] = input[ 12 + i*9 + j*78 ];
	  output[ 11 + i*9 + j*78 ] = input[ 13 + i*9 + j*78 ];
	  output[  8 + i*9 + j*78 ] = input[ 14 + i*9 + j*78 ];
	  output[ 62 + j*4 + i*78 ] = input[ 60 + i*4 + j*78 ]; // Swapping j for i
	  output[ 63 + i*4 + j*78 ] = input[ 61 + i*4 + j*78 ];
	  output[ 60 + i*4 + j*78 ] = input[ 62 + i*4 + j*78 ];
	  output[ 61 + j*4 + i*78 ] = input[ 63 + i*4 + j*78 ]; // Swapping j for i
	}
      }
      break;
    default:
      std::cerr << "ERR (in C3_HLAC::rotateFeature90): unknown RotateMode." << std::endl;
      exit( EXIT_FAILURE );
      break;
    }
    break;
  default:
    std::cerr << "ERR (in C3_HLAC::rotateFeature90): improper dimension: " << dim << std::endl;
    exit( EXIT_FAILURE );
    break;
  }
}

//*********************************************************//
//* functions for C3HLACSignature117 (rotation-invariant) *//
//*********************************************************//
template <typename PointT, typename PointOutT>
pcl::C3HLAC117Estimation<PointT, PointOutT>::C3HLAC117Estimation () : voxel_size(0), hist_num (1), color_threshold_r (-1), color_threshold_g (-1), color_threshold_b (-1){
  feature_name_ = "C3HLAC117Estimation";
  relative_coordinates.resize(3, 13);
  int idx = 0;
  // 0 - 8
  for( int i=-1; i<2; i++ ){
    for( int j=-1; j<2; j++ ){
      relative_coordinates( 0, idx ) = i;
      relative_coordinates( 1, idx ) = j;
      relative_coordinates( 2, idx ) = -1;
      idx++;
    }
  }
  // 9 - 11
  for( int i=-1; i<2; i++ ){
    relative_coordinates( 0, idx ) = i;
    relative_coordinates( 1, idx ) = -1;
    relative_coordinates( 2, idx ) = 0;
    idx++;
  }
  // 12
  relative_coordinates( 0, idx ) = -1;
  relative_coordinates( 1, idx ) = 0;
  relative_coordinates( 2, idx ) = 0;
};

template <typename PointT, typename PointOutT> bool
pcl::C3HLAC117Estimation<PointT, PointOutT>::setVoxelFilter ( pcl::VoxelGrid<PointT> grid_, const int subdivision_size_, const int offset_x_, const int offset_y_, const int offset_z_, const float voxel_size_ )
{ 
  grid = grid_;
  voxel_size = voxel_size_;

  if( subdivision_size_ > 0 ){
    inverse_subdivision_size = 1.0 / subdivision_size_;
    offset_x = offset_x_;
    offset_y = offset_y_;
    offset_z = offset_z_;
    div_b_ = grid.getNrDivisions();
    min_b_ = grid.getMinBoxCoordinates();
    if( ( div_b_[0] <= offset_x ) || ( div_b_[1] <= offset_y ) || ( div_b_[2] <= offset_z ) ){
      std::cerr << "(In setVoxelFilter) offset values (" << offset_x << "," << offset_y << "," << offset_z << ") exceed voxel grid size (" << div_b_[0] << "," << div_b_[1] << "," << div_b_[2] << ")."<< std::endl;
      return false;
    }

    subdiv_b_ = Eigen::Vector3i ( ceil( ( div_b_[0] - offset_x )*inverse_subdivision_size ), ceil( ( div_b_[1] - offset_y )*inverse_subdivision_size ), ceil( ( div_b_[2] - offset_z )*inverse_subdivision_size ) );
    subdivb_mul_ = Eigen::Vector3i ( 1, subdiv_b_[0], subdiv_b_[0] * subdiv_b_[1] );
    hist_num = subdiv_b_[0] * subdiv_b_[1] * subdiv_b_[2];
  }
  else if( subdivision_size_ < 0 ){
    std::cerr << "(In setVoxelFilter) Invalid subdivision size: " << subdivision_size_ << std::endl;
    return false;
  }
  return true;
}

template <typename PointT, typename PointOutT> void
pcl::C3HLAC117Estimation<PointT, PointOutT>::normalizeC3HLAC ( PointCloudOut &output )
{
  for( int h=0; h<hist_num; h++ ){
    for(int i=0; i<6; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_117_0;
    for(int i=6; i<42; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_117_1;
    for(int i=42; i<DIM_C3HLAC_117_1_3; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_1; // not divided by 13.
    for(int i=DIM_C3HLAC_117_1_3; i<69; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_117_0_BIN;
    for(int i=69; i<105; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_117_1_BIN;
    for(int i=105; i<DIM_C3HLAC_117_1_3_ALL; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_1_BIN; // not divided by 13.
  }
}

template <typename PointT, typename PointOutT> void 
pcl::C3HLAC117Estimation<PointT, PointOutT>::computeC3HLAC ( const pcl::PointCloud<PointT> &cloud,
					       PointCloudOut &output, const int center_idx )
{
  // calc hist_idx
  int hist_idx;
  if( hist_num == 1 ) hist_idx = 0;
  else{
    const int tmp_x = floor( (*surface_).points[ center_idx ].x/voxel_size ) - min_b_[ 0 ] - offset_x;
    const int tmp_y = floor( (*surface_).points[ center_idx ].y/voxel_size ) - min_b_[ 1 ] - offset_y;
    const int tmp_z = floor( (*surface_).points[ center_idx ].z/voxel_size ) - min_b_[ 2 ] - offset_z;

    if( ( tmp_x < 0 ) || ( tmp_y < 0 ) || ( tmp_z < 0 ) ){
      return; // ignore idx smaller than offset.
    }

    Eigen::Vector3i ijk = Eigen::Vector3i ( floor ( tmp_x * inverse_subdivision_size), floor ( tmp_y * inverse_subdivision_size), floor ( tmp_z * inverse_subdivision_size) );
    hist_idx = ijk.dot (subdivb_mul_);
  }
  float* histogram = output.points[hist_idx].histogram;

  color = *reinterpret_cast<const int*>(&(cloud.points[center_idx].rgb));
  center_r  = (0xff0000 & color) >> 16;
  center_g  = (0x00ff00 & color) >> 8;
  center_b  =  0x0000ff & color;
  center_bin_r = binarizeR( center_r );
  center_bin_g = binarizeG( center_g );
  center_bin_b = binarizeB( center_b );
  c3_hlac::addC3HLACcol0Bin117( &histogram, center_bin_r, center_bin_g, center_bin_b );

  c3_hlac::setColor( center_r, center_g, center_b, center_r_, center_g_, center_b_ );
  c3_hlac::addC3HLACcol0117( &histogram, center_r, center_r_, center_g, center_g_, center_b, center_b_ );

  const std::vector<int> neighbors = grid.getNeighborCentroidIndices (cloud.points[center_idx], relative_coordinates);

  int r, g, b, r_, g_, b_;
  for (int i = 0; i < 13; ++i){
    // Check if the point is invalid
    if ( neighbors[i]!=-1 ){
      color = *reinterpret_cast<const int*>(&(cloud.points[neighbors[i]].rgb));
      r  = (0xff0000 & color) >> 16;
      g  = (0x00ff00 & color) >> 8;
      b  =  0x0000ff & color;
      c3_hlac::addC3HLACcol1Bin117( &histogram, center_bin_r, center_bin_g, center_bin_b, binarizeR(r), binarizeG(g), binarizeB(b) );

      c3_hlac::setColor( r, g, b, r_, g_, b_ );
      c3_hlac::addC3HLACcol1117( &histogram, center_r, center_r_, center_g, center_g_, center_b, center_b_, r, g, b, r_, g_, b_ );
    }
  }
}

template <typename PointT, typename PointOutT> void
pcl::C3HLAC117Estimation<PointT, PointOutT>::computeFeature (PointCloudOut &output)
{
  if( (color_threshold_r<0)||(color_threshold_g<0)||(color_threshold_b<0) ){
    std::cerr << "Invalid color_threshold: " << color_threshold_r << " " << color_threshold_g << " " << color_threshold_b << std::endl;
    return;
  }

  output.points.resize (hist_num);
  output.width = hist_num;
  output.height = 1;

  // initialize histogram
  for( int h=0; h<hist_num; h++ )
    for( int t=0; t<DIM_C3HLAC_117_1_3_ALL; t++ )
      output.points[h].histogram[t] = 0;
  
  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
    computeC3HLAC (*surface_, output, (*indices_)[idx] );
  normalizeC3HLAC( output );
}

//*******************************************************//
//* functions for C3HLACSignature981 (rotation-variant) *//
//*******************************************************//
template <typename PointT, typename PointOutT> void
pcl::C3HLAC981Estimation<PointT, PointOutT>::normalizeC3HLAC ( PointCloudOut &output )
{
  for( int h=0; h<hist_num; h++ ){
    for(int i=0; i<6; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_0;
    for(int i=6; i<DIM_C3HLAC_981_1_3; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_1;
    for(int i=DIM_C3HLAC_981_1_3; i<501; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_0_BIN;
    for(int i=501; i<DIM_C3HLAC_981_1_3_ALL; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_1_BIN;
  }
}

template <typename PointT, typename PointOutT> void 
pcl::C3HLAC981Estimation<PointT, PointOutT>::computeC3HLAC ( const pcl::PointCloud<PointT> &cloud,
					       PointCloudOut &output, const int center_idx )
{
  // calc hist_idx
  int hist_idx;
  if( hist_num == 1 ) hist_idx = 0;
  else{
    const int tmp_x = floor( (*surface_).points[ center_idx ].x/voxel_size ) - min_b_[ 0 ] - offset_x;
    const int tmp_y = floor( (*surface_).points[ center_idx ].y/voxel_size ) - min_b_[ 1 ] - offset_y;
    const int tmp_z = floor( (*surface_).points[ center_idx ].z/voxel_size ) - min_b_[ 2 ] - offset_z;

    if( ( tmp_x < 0 ) || ( tmp_y < 0 ) || ( tmp_z < 0 ) ){
      return; // ignore idx smaller than offset.
    }

    Eigen::Vector3i ijk = Eigen::Vector3i ( floor ( tmp_x * inverse_subdivision_size), floor ( tmp_y * inverse_subdivision_size), floor ( tmp_z * inverse_subdivision_size) );
    hist_idx = ijk.dot (subdivb_mul_);
  }
  float* histogram = output.points[hist_idx].histogram;

  color = *reinterpret_cast<const int*>(&(cloud.points[center_idx].rgb));
  center_r  = (0xff0000 & color) >> 16;
  center_g  = (0x00ff00 & color) >> 8;
  center_b  =  0x0000ff & color;
  center_bin_r = binarizeR( center_r );
  center_bin_g = binarizeG( center_g );
  center_bin_b = binarizeB( center_b );
  c3_hlac::addC3HLACcol0Bin981( &histogram, center_bin_r, center_bin_g, center_bin_b );

  c3_hlac::setColor( center_r, center_g, center_b, center_r_, center_g_, center_b_ );
  c3_hlac::addC3HLACcol0981( &histogram, center_r, center_r_, center_g, center_g_, center_b, center_b_ );

  const std::vector<int> neighbors = grid.getNeighborCentroidIndices (cloud.points[center_idx], relative_coordinates);

  int r, g, b, r_, g_, b_;
  for (int i = 0; i < 13; ++i){
    // Check if the point is invalid
    if ( neighbors[i]!=-1 ){
      color = *reinterpret_cast<const int*>(&(cloud.points[neighbors[i]].rgb));
      r  = (0xff0000 & color) >> 16;
      g  = (0x00ff00 & color) >> 8;
      b  =  0x0000ff & color;
      c3_hlac::addC3HLACcol1Bin981( &histogram, i, center_bin_r, center_bin_g, center_bin_b, binarizeR(r), binarizeG(g), binarizeB(b) );

      c3_hlac::setColor( r, g, b, r_, g_, b_ );
      c3_hlac::addC3HLACcol1981( &histogram, i, center_r, center_r_, center_g, center_g_, center_b, center_b_, r, g, b, r_, g_, b_ );
    }
  }
}

template <typename PointT, typename PointOutT> void
pcl::C3HLAC981Estimation<PointT, PointOutT>::computeFeature (PointCloudOut &output)
{
  if( (color_threshold_r<0)||(color_threshold_g<0)||(color_threshold_b<0) ){
    std::cerr << "Invalid color_threshold: " << color_threshold_r << " " << color_threshold_g << " " << color_threshold_b << std::endl;
    return;
  }

  output.points.resize (hist_num);
  output.width = hist_num;
  output.height = 1;

  // initialize histogram
  for( int h=0; h<hist_num; h++ )
    for( int t=0; t<DIM_C3HLAC_981_1_3_ALL; t++ )
      output.points[h].histogram[t] = 0;
  
  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
    computeC3HLAC (*surface_, output, (*indices_)[idx] );
  normalizeC3HLAC( output );
}

template class pcl::C3HLAC117Estimation<pcl::PointXYZRGB, pcl::C3HLACSignature117>;
template class pcl::C3HLAC117Estimation<pcl::PointXYZRGB, pcl::C3HLACSignature981>;
template class pcl::C3HLAC981Estimation<pcl::PointXYZRGB, pcl::C3HLACSignature117>;
template class pcl::C3HLAC981Estimation<pcl::PointXYZRGB, pcl::C3HLACSignature981>;

template class pcl::C3HLAC117Estimation<pcl::PointXYZRGBNormal, pcl::C3HLACSignature117>;
template class pcl::C3HLAC117Estimation<pcl::PointXYZRGBNormal, pcl::C3HLACSignature981>;
template class pcl::C3HLAC981Estimation<pcl::PointXYZRGBNormal, pcl::C3HLACSignature117>;
template class pcl::C3HLAC981Estimation<pcl::PointXYZRGBNormal, pcl::C3HLACSignature981>;
