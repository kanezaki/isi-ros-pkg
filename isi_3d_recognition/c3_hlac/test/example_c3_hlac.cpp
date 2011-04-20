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

#include "c3_hlac/c3_hlac_tools.h"

//#define DIVID_TEST

int main( int argc, char** argv ){
  if( argc != 2 ){
    ROS_ERROR ("Need one parameter! Syntax is: %s {input_pointcloud_filename.pcd}\n", argv[0]);
    return(-1);
  }

  //* voxel_size : the length of voxel side (unit: meter)
  const double voxel_size = 0.01;

  //* read
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  if (pcl::io::loadPCDFile (argv[1], input_cloud) == -1){
    ROS_ERROR ("Couldn't read file %s",argv[1]);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(input_cloud.width * input_cloud.height), argv[1], pcl::getFieldsList (input_cloud).c_str ());

  //* voxelize
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled;
  getVoxelGrid( grid, input_cloud, cloud_downsampled, voxel_size );

#ifdef DIVID_TEST
  //* extract - C3_HLAC -
  std::vector< std::vector<float> > c3_hlac;
  extractC3HLACSignature981( grid, cloud_downsampled, c3_hlac, 127, 127, 127, voxel_size, 10 );

  //* write
  writeFeature( "c3hlac.pcd", c3_hlac );

#else
  //* extract - C3_HLAC -
  std::vector<float> c3_hlac;
  extractC3HLACSignature981( grid, cloud_downsampled, c3_hlac, 127, 127, 127, voxel_size );

  //* write
  writeFeature( "c3hlac.pcd", c3_hlac );
#endif

  return(0);
}
