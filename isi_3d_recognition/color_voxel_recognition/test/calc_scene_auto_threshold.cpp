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

//**************************************************************************************************************************//
//* read point clouds, voxelize, and determine threshold for RGB binarlization                                             *//
//* output the result in color_voxel_recognition/demos/param/color_threshold.txt                                           *//
//*   Note that this process is necessary only once when the system sees the environment for the first time                *//
//*   Or, you can skip this process and set the values manually in color_voxel_recognition/demos/param/color_threshold.txt *//
//**************************************************************************************************************************//

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>
#include <color_voxel_recognition/param.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv)
{
  if( argc != 3 ){
    std::cerr << "usage: " << argv[0] << " [path] <registration_num>" << std::endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 1000 ];
  const int file_num = atoi( argv[2] );

  int totalNum = 0;                    // the number of occupied voxels
  int *threshold   = new int[ 3 ];     // threshold for RGB binarize (= results)
  double *totalAve = new double[ 3 ];  // total average of RGB values
  double **eachAve = new double*[ 3 ]; // each average of RGB values from 0 to j
  int **eachNum    = new int*[ 3 ];    // the number of occupied voxles with RGB values from 0 to j
  int **h          = new int*[ 3 ];    // histograms of RGB

  //* initialize
  for( int i = 0; i < 3; i++ ){
    threshold[ i ]    = 0;
    totalAve[ i ]     = 0;
    eachAve[ i ]      = new double[ 256 ];
    eachNum[ i ]      = new int[ 256 ];
    h[ i ]            = new int[ 256 ];
    for( int j = 0; j < 256; j++ )
      h[ i ][ j ]       = 0;
  }

  //* read the length of voxel side
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  const float voxel_size = Param::readVoxelSize( tmpname );

  //* Voxel Grid
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setLeafSize (voxel_size, voxel_size, voxel_size);

  //* read points, voxelize, and make histograms of RGB
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled;
  for( int i=0; i<file_num; i++ ){
    sprintf( tmpname, "%s/scene/Points/%05d.pcd", argv[1], i );
    pcl::io::loadPCDFile (tmpname, input_cloud);
    grid.setInputCloud ( input_cloud.makeShared() );
    grid.filter (cloud_downsampled);

    const int p_num = cloud_downsampled.points.size();
    for( int i = 0; i<p_num; i++ ){
      if( std::isfinite(cloud_downsampled.points[i].x) && std::isfinite(cloud_downsampled.points[i].y) && std::isfinite(cloud_downsampled.points[i].z) ){
	const int color = *reinterpret_cast<const int*>(&(cloud_downsampled.points[i].rgb));
	const int r = (0xff0000 & color) >> 16;
	const int g = (0x00ff00 & color) >> 8;
	const int b =  0x0000ff & color;
	
	totalNum++;
	h[ 0 ][ r ] ++;
	h[ 1 ][ g ] ++;
	h[ 2 ][ b ] ++;
      }
    }
  }
    
  //* total average of RGB values
  const double scale = 1 / (double)totalNum;
  for( int i = 0; i < 3; i++ ){
    for( int j = 0; j < 256; j++ )
      totalAve[ i ] += j * h[ i ][ j ];
    totalAve[ i ] *= scale;
  }

  //* each average of RGB values from 0 to j
  for( int i = 0; i < 3; i++ ){
    eachAve[ i ][ 0 ] = 0;
    eachNum[ i ][ 0 ] = h[ i ][ 0 ];
    int tmp_eachAve = 0;
    for( int j = 1; j < 256; j++ ){
      eachNum[ i ][ j ] = eachNum[ i ][ j-1 ] + h[ i ][ j ];
      tmp_eachAve += j * h[ i ][ j ];
      if( eachNum[ i ][ j ] == 0 )
	eachAve[ i ][ j ] = 0;
      else
	eachAve[ i ][ j ] = (double)tmp_eachAve / eachNum[ i ][ j ];
    }
  }

  //* determine threshold of RGB
  //*   by looking for the case when between-class variance becomes the largest
  for( int i = 0; i < 3; i++ ){
    double max_var = 0;
    for( int j = 1; j < 256; j++ ){
      if( eachNum[ i ][ j ] != 0 ){
	if( eachNum[ i ][ j ] == totalNum ) break;
	const double ave_sub = eachAve[ i ][ j ] - totalAve[ i ];
	const double var = ave_sub * ave_sub * ( eachNum[ i ][ j ] / (double)( totalNum - eachNum[ i ][ j ] ) );
	if( var > max_var ){
	  max_var = var;
	  threshold[ i ] = j;
	}
      }
    }
  }
  
  printf("totalAverage: %f %f %f\n",  totalAve[0], totalAve[1], totalAve[2]);
  printf("threshold: %d %d %d\n",threshold[0],threshold[1],threshold[2]);

  //* output results into file
  sprintf( tmpname, "%s/param/color_threshold.txt", argv[1] );
  FILE *fp = fopen( tmpname,"w" );
  fprintf(fp,"%d %d %d\n",threshold[0],threshold[1],threshold[2]);
  fclose(fp);

  delete[] h[ 0 ];
  delete[] h[ 1 ];
  delete[] h[ 2 ];
  delete[] h;

  return 0;
}
