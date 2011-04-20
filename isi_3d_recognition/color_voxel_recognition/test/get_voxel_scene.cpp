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

/***********************************************************************************************************/
/* voxelize point clouds of scene                                                                          */
/*   Note that this process is necessary only once when the system sees the environment for the first time */
/***********************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>
#include <color_voxel_recognition/param.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv)
{
  if( argc != 3 ){
    std::cerr << "usage: " << argv[0] << " [path] <registration_num>" << std::endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 1000 ];

  //* read the length of voxel side
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  const float voxel_size = Param::readVoxelSize( tmpname );

  //* Voxel Grid
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid.setSaveLeafLayout(true);

  const int obj_num = atoi(argv[2]);
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
  for(int i=0;i<obj_num;i++){
    sprintf( tmpname, "%s/scene/Points/%05d.pcd", argv[1], i );
    pcl::io::loadPCDFile (tmpname, input_cloud);

    //* voxelize
    grid.setInputCloud ( input_cloud.makeShared() );
    grid.filter (output_cloud);

    sprintf( tmpname, "%s/scene/Voxel/%05d.dat", argv[1], i ); 
    pcl::io::savePCDFile (tmpname, output_cloud, true);
  }

  return(0);    
}
