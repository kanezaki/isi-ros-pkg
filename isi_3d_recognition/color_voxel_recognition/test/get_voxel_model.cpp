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

/*************************************************************************************************/
/* voxelize point clouds of the target object                                                    */
/*   Note that voxelization is also done for synthetically rotated point clouds                  */
/*   bounding box size is written into color_voxel_recognition/demos/models/$model_name/size.txt */
/*************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <color_voxel_recognition/param.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

//*****************************
//* determine bounding box size
void setSize( float &s1, float &s2, float &s3, const float val ){
  if( val > s1 ){
    s3 = s2; s2 = s1; s1 = val;
  }
  else if( val > s2 ){
    s3 = s2; s2 = val;
  }
  else if( val > s3 ){
    s3 = val;
  }
}

//*****************************************
//* get min max values of point coordinates
template <typename T>
void getMinMax_points( const T& input_cloud, float &val1, float &val2, float &val3 ){
  float x_min = FLT_MAX, y_min = FLT_MAX, z_min = FLT_MAX;
  float x_max = - FLT_MAX, y_max = - FLT_MAX, z_max = - FLT_MAX;
  const int v_num = input_cloud.points.size();
  for( int i=0; i<v_num; i++ ){
    if( x_min > input_cloud.points[ i ].x ) x_min = input_cloud.points[ i ].x;
    if( y_min > input_cloud.points[ i ].y ) y_min = input_cloud.points[ i ].y;
    if( z_min > input_cloud.points[ i ].z ) z_min = input_cloud.points[ i ].z;
    if( x_max < input_cloud.points[ i ].x ) x_max = input_cloud.points[ i ].x;
    if( y_max < input_cloud.points[ i ].y ) y_max = input_cloud.points[ i ].y;
    if( z_max < input_cloud.points[ i ].z ) z_max = input_cloud.points[ i ].z;
  }
  val1 = x_max - x_min;
  val2 = y_max - y_min;
  val3 = z_max - z_min;
}

//*****************************
//* synthetically rotate points
template <typename T>
bool rotatePoints( const T& input_cloud, T& output_cloud, const double roll, const double pan, const double roll2 ){
  output_cloud = input_cloud;
  double R1[9];
  R1[0]=cos(roll);
  R1[1]=-sin(roll);
  R1[2]=0;
  R1[3]=sin(roll);
  R1[4]=cos(roll);
  R1[5]=0;
  R1[6]=0;
  R1[7]=0;
  R1[8]=1;
  double R2[9];
  R2[0]=cos(pan);
  R2[1]=0;
  R2[2]=sin(pan);
  R2[3]=0;
  R2[4]=1;
  R2[5]=0;
  R2[6]=-sin(pan);
  R2[7]=0;
  R2[8]=cos(pan);
  double R3[9];
  R3[0]=cos(roll2);
  R3[1]=-sin(roll2);
  R3[2]=0;
  R3[3]=sin(roll2);
  R3[4]=cos(roll2);
  R3[5]=0;
  R3[6]=0;
  R3[7]=0;
  R3[8]=1;

  float x1, y1, z1, x2, y2, z2;
  const int v_num = input_cloud.points.size();
  for( int i=0; i<v_num; i++ ){
    x1 = R1[0] * input_cloud.points[ i ].x + R1[1] * input_cloud.points[ i ].y + R1[2] * input_cloud.points[ i ].z;
    y1 = R1[3] * input_cloud.points[ i ].x + R1[4] * input_cloud.points[ i ].y + R1[5] * input_cloud.points[ i ].z;
    z1 = R1[6] * input_cloud.points[ i ].x + R1[7] * input_cloud.points[ i ].y + R1[8] * input_cloud.points[ i ].z;
    
    x2 = R2[0]*x1 + R2[1]*y1 + R2[2]*z1;
    y2 = R2[3]*x1 + R2[4]*y1 + R2[5]*z1;
    z2 = R2[6]*x1 + R2[7]*y1 + R2[8]*z1;
	    
    output_cloud.points[ i ].x = R3[0]*x2 + R3[1]*y2 + R3[2]*z2;
    output_cloud.points[ i ].y = R3[3]*x2 + R3[4]*y2 + R3[5]*z2;
    output_cloud.points[ i ].z = R3[6]*x2 + R3[7]*y2 + R3[8]*z2;
  }

  return(1);
}

//********************************
//* main
int main(int argc, char **argv)
{
  if( argc != 4 ){
    std::cerr << "usage: " << argv[0] << " [path] [model_name] <registration_num>" << std::endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 1000 ];

  //* length of bounding box sides (initialize)
  float size1 = 0;
  float size2 = 0;
  float size3 = 0;
  float vals[3];

  //* read rotate_num
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  const int rotate_num = Param::readRotateNum( tmpname );

  //* read the number of voxels in each subdivision's side of a target object
  const float voxel_size = Param::readVoxelSize( tmpname );

  //* Voxel Grid
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid.setSaveLeafLayout(true);

  const int obj_num = atoi(argv[3]);
  int write_count = 0; // number for output file names
  for( int i=0; i<obj_num; i++ ){
    pcl::PointCloud<pcl::PointXYZRGB> ref_cloud;
    sprintf( tmpname, "%s/models/%s/Points/%05d.pcd", argv[1], argv[2], i );
    pcl::io::loadPCDFile (tmpname, ref_cloud);

    //* rotate point clouds synthetically to T postures
    //* T postures: obtained by rotation with each (90/$rotate_num) degrees for x-, y- and z-axis.
    for(int r3=0; r3 < rotate_num; r3++){
      for(int r2=0; r2 < rotate_num; r2++){
	for(int r1=0; r1 < rotate_num; r1++){
	  const double roll  = r3 * M_PI / (2*rotate_num);
	  const double pan   = r2 * M_PI / (2*rotate_num);
	  const double roll2 = r1 * M_PI / (2*rotate_num);

	  //* voxelize
	  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
	  pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
	  rotatePoints( ref_cloud, input_cloud, roll, pan, roll2 ); // rotation
	  grid.setInputCloud ( input_cloud.makeShared() );
	  grid.filter (output_cloud);
	  sprintf( tmpname, "%s/models/%s/Voxel/%05d.dat", argv[1], argv[2], write_count++ );
	  pcl::io::savePCDFile (tmpname, output_cloud, true);

	  //* determine bounding box size
	  getMinMax_points( output_cloud, vals[0], vals[1], vals[2] );
	  float tmp_size1 = 0;
	  float tmp_size2 = 0;
	  float tmp_size3 = 0;
	  for( int j=0; j<3; j++ )
	    setSize( tmp_size1, tmp_size2, tmp_size3, vals[ j ] );
	  if( tmp_size1 > size1 ) size1 = tmp_size1;
	  if( tmp_size2 > size2 ) size2 = tmp_size2;
	  if( tmp_size3 > size3 ) size3 = tmp_size3;
	  
	  if(r2==0) break;
	}
      }
    }
  }

  //* output bounding box size
  sprintf( tmpname, "%s/models/%s/size.txt", argv[1], argv[2] );
  FILE *fp = fopen( tmpname, "w" );
  fprintf( fp, "%f %f %f\n", size1, size2, size3 );
  fclose( fp );

  return(0);
}
