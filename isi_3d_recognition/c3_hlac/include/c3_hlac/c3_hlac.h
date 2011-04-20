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

#ifndef C3_HLAC_H_
#define C3_HLAC_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/filters/voxel_grid.h>

const int DIM_C3HLAC_981_1_3 = 495;        // Dimension of feature vector (without RGB binalize)
const int DIM_C3HLAC_981_BIN_1_3 = 486;    // Dimension of feature vector (with RGB binalize)
const int DIM_C3HLAC_981_1_3_ALL = 981;    // = DIM_C3HLAC_981_1_3 + DIM_C3HLAC_981_BIN_1_3
const int DIM_C3HLAC_117_1_3 = 63;      // Dimension of feature vector (without RGB binalize) - rotation-invariant -
const int DIM_C3HLAC_117_BIN_1_3 = 54;  // Dimension of feature vector (with RGB binalize) - rotation-invariant -
const int DIM_C3HLAC_117_1_3_ALL = 117; // = DIM_C3HLAC_117_1_3 + DIM_C3HLAC_117_BIN_1_3

namespace pcl{
  struct C3HLACSignature117
  {
    float histogram[117];
  };
  inline std::ostream& operator << (std::ostream& os, const C3HLACSignature117& p)
  {
    for (int i = 0; i < 117; ++i)
      os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 116 ? ", " : ")");
    return (os);
  }

  struct C3HLACSignature981
  {
    float histogram[981];
  };
  inline std::ostream& operator << (std::ostream& os, const C3HLACSignature981& p)
  {
    for (int i = 0; i < 981; ++i)
      os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 980 ? ", " : ")");
    return (os);
  }

  //* mode for 90 degrees rotation
  enum RotateMode{ R_MODE_1, R_MODE_2, R_MODE_3, R_MODE_4 };

  //* calculate a feature vector when the target voxel data is rotated by 90 degrees.
  void rotateFeature90( std::vector<float> &output, const std::vector<float> &input, RotateMode mode );

  //* class for extracting rotation-invariant C3_HLAC (117 dim.)
  template <typename PointT, typename PointOutT>
  class C3HLAC117Estimation: public Feature<PointT, PointOutT>
    {
    public:
      using Feature<PointT, PointOutT>::feature_name_;
      //using Feature<PointT, PointOutT>::getClassName;
      using Feature<PointT, PointOutT>::indices_;
      using Feature<PointT, PointOutT>::surface_;
      //using Feature<PointT, PointOutT>::k_;
      //using Feature<PointT, PointOutT>::search_parameter_;
      typedef typename Feature<PointT, PointOutT>::PointCloudOut PointCloudOut;

      //* set thresholds for RGB
      inline void setColorThreshold ( int threshold_r, int threshold_g, int threshold_b ){ color_threshold_r = threshold_r; color_threshold_g = threshold_g; color_threshold_b = threshold_b; }

      //-------------------
      //* set voxel filter
      //*    subdivision_size : the number of voxels in each subdivision's side
      //*                       set 0 if you want to extract one C3_HLAC vector from the whole region
      //*    offset_x : the number of overlapping voxels in neighboring subdivisions in x-axis
      //*    offset_y : the number of overlapping voxels in neighboring subdivisions in y-axis
      //*    offset_z : the number of overlapping voxels in neighboring subdivisions in z-axis
      //*    voxel_size : the length of voxel side (unit: meter)
      bool setVoxelFilter ( pcl::VoxelGrid<PointT> grid_, const int subdivision_size_ = 0, const int offset_x_ = 0, const int offset_y_ = 0, const int offset_z_ = 0, const float voxel_size_ = 0.01 );

      //* get the number of subdivisions in x-, y- and z-axis.
      Eigen::Vector3i getSubdivNum(){ return subdiv_b_; };

      //* constructor
      C3HLAC117Estimation ();

    protected:
      inline int binarizeR ( int val ){
	if( val > color_threshold_r ) return 1;
	return 0;
      }
      inline int binarizeG ( int val ){
	if( val > color_threshold_g ) return 1;
	return 0;
      }
      inline int binarizeB ( int val ){
	if( val > color_threshold_b ) return 1;
	return 0;
      }

      virtual void computeC3HLAC ( const pcl::PointCloud<PointT> &cloud, PointCloudOut &output, const int center_idx );
      virtual void normalizeC3HLAC ( PointCloudOut &output );
      virtual void computeFeature (PointCloudOut &output);

    protected:
      float voxel_size;
      pcl::VoxelGrid<PointT> grid;
      Eigen::MatrixXi relative_coordinates;
      int hist_num;
      float inverse_subdivision_size;
      int offset_x;
      int offset_y;
      int offset_z;
      Eigen::Vector3i div_b_;
      Eigen::Vector3i min_b_;
      Eigen::Vector3i subdiv_b_;
      Eigen::Vector3i subdivb_mul_;
      int color;
      int center_r;
      int center_g;
      int center_b;
      int center_r_;
      int center_g_;
      int center_b_;
      int center_bin_r;
      int center_bin_g;
      int center_bin_b;
      int color_threshold_r;
      int color_threshold_g;
      int color_threshold_b;
  };

  //* class for extracting rotation-variant C3_HLAC (981 dim.)
  template <typename PointT, typename PointOutT>
  class C3HLAC981Estimation: public C3HLAC117Estimation<PointT, PointOutT>
    {
    public:
      using C3HLAC117Estimation<PointT, PointOutT>::feature_name_;
      //using C3HLAC117Estimation<PointT, PointOutT>::getClassName;
      using C3HLAC117Estimation<PointT, PointOutT>::indices_;
      using C3HLAC117Estimation<PointT, PointOutT>::surface_;
      //using C3HLAC117Estimation<PointT, PointOutT>::k_;
      //using C3HLAC117Estimation<PointT, PointOutT>::search_parameter_;

      typedef typename C3HLAC117Estimation<PointT, PointOutT>::PointCloudOut PointCloudOut;

      // constructor
      C3HLAC981Estimation ()
      {
        feature_name_ = "C3HLAC981Estimation";
      };

    protected:
      inline int binarizeR ( int val ){
	if( val > color_threshold_r ) return 1;
	return 0;
      }
      inline int binarizeG ( int val ){
	if( val > color_threshold_g ) return 1;
	return 0;
      }
      inline int binarizeB ( int val ){
	if( val > color_threshold_b ) return 1;
	return 0;
      }

      virtual void computeC3HLAC ( const pcl::PointCloud<PointT> &cloud, PointCloudOut &output, const int center_idx );
      virtual void normalizeC3HLAC ( PointCloudOut &output );
      virtual void computeFeature (PointCloudOut &output);

    protected:
      using C3HLAC117Estimation<PointT, PointOutT>::voxel_size;
      using C3HLAC117Estimation<PointT, PointOutT>::grid;
      using C3HLAC117Estimation<PointT, PointOutT>::relative_coordinates;
      using C3HLAC117Estimation<PointT, PointOutT>::hist_num;
      using C3HLAC117Estimation<PointT, PointOutT>::inverse_subdivision_size;
      using C3HLAC117Estimation<PointT, PointOutT>::offset_x;
      using C3HLAC117Estimation<PointT, PointOutT>::offset_y;
      using C3HLAC117Estimation<PointT, PointOutT>::offset_z;
      using C3HLAC117Estimation<PointT, PointOutT>::div_b_;
      using C3HLAC117Estimation<PointT, PointOutT>::min_b_;
      using C3HLAC117Estimation<PointT, PointOutT>::subdiv_b_;
      using C3HLAC117Estimation<PointT, PointOutT>::subdivb_mul_;
      using C3HLAC117Estimation<PointT, PointOutT>::color;
      using C3HLAC117Estimation<PointT, PointOutT>::center_r;
      using C3HLAC117Estimation<PointT, PointOutT>::center_g;
      using C3HLAC117Estimation<PointT, PointOutT>::center_b;
      using C3HLAC117Estimation<PointT, PointOutT>::center_r_;
      using C3HLAC117Estimation<PointT, PointOutT>::center_g_;
      using C3HLAC117Estimation<PointT, PointOutT>::center_b_;
      using C3HLAC117Estimation<PointT, PointOutT>::center_bin_r;
      using C3HLAC117Estimation<PointT, PointOutT>::center_bin_g;
      using C3HLAC117Estimation<PointT, PointOutT>::center_bin_b;
      using C3HLAC117Estimation<PointT, PointOutT>::color_threshold_r;
      using C3HLAC117Estimation<PointT, PointOutT>::color_threshold_g;
      using C3HLAC117Estimation<PointT, PointOutT>::color_threshold_b;
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::C3HLACSignature117,
                                   (float[117], histogram, c3_hlac_117));

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::C3HLACSignature981,
                                   (float[981], histogram, c3_hlac_981));

#endif  //#ifndef C3_HLAC_H_


