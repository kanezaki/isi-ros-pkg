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

/**************************************************************************************************/
/* calculate projection axis of the target object's subspace                                      */
/*   Note that features are also extracted from synthetically rotated point clouds (each 90 deg.) */
/**************************************************************************************************/

#include <iostream>
#include <c3_hlac/c3_hlac.h>
#include <c3_hlac/c3_hlac_tools.h>
#include <color_voxel_recognition/pca.h>
#include <color_voxel_recognition/param.h>
#include <color_voxel_recognition/FILE_MODE>

//**************************
//* compress feature vectors
const std::vector< float > compressFeature( std::vector< float > &feature, Eigen::MatrixXf &axis_t, Eigen::VectorXf &variance, const int dim ){
  Eigen::Map<Eigen::VectorXf> vec( &(feature[0]), feature.size() );
  std::vector< float > feature_compressed;
  feature_compressed.resize( dim );
  Eigen::VectorXf vec2 = axis_t*vec;
  if( WHITENING ){
    for(int t=0;t<dim;t++)
      feature_compressed[t] = vec2( t ) / sqrt( variance(t) );
  }
  else{
    for(int t=0;t<dim;t++)
      feature_compressed[t] = vec2( t );
  }
  return feature_compressed;
}

//********************************
//* main
int main( int argc, char* argv[])
{
  if( argc != 4 ){
    std::cerr << "usage: " << argv[0] << " [path] [label] <registration_num>" << std::endl;
    exit( EXIT_FAILURE );
  }
  const int file_num = atoi( argv[3] );
  char tmpname[ 1000 ];

  //* read the dimension of compressed feature vectors
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  const int dim = Param::readDim( tmpname );

  //* read flag for c3_hlac extraction
  const int c3_hlac_flg = Param::readC3HLACFlag( tmpname );

  //* read the projection matrix for feature compression
  PCA pca;
  sprintf( tmpname, "%s/models/compress_axis", argv[1] );
  pca.read( tmpname, ASCII_MODE_P );
  Eigen::MatrixXf tmpaxis = pca.getAxis();
  Eigen::MatrixXf axis = tmpaxis.block( 0,0,tmpaxis.rows(),dim );
  Eigen::MatrixXf axis_t = axis.transpose();
  Eigen::VectorXf variance = pca.getVariance();

  //*****************//
  //* read features *//
  //*****************//

  PCA pca_each( false );
  std::vector< std::vector<float> > feature;
  for( int n = 0; n < file_num; n++ ){
    printf("%d in %d...\n",n,file_num);
    sprintf( tmpname, "%s/models/%s/Features/%05d.pcd", argv[1], argv[2], n );
    readFeature( tmpname, feature );
    const int hist_num = feature.size();	      
    
    for( int h=0; h<hist_num; h++ ){
      //if( !if_zero_vec( feature[ h ] ) ){ 
      pca_each.addData( compressFeature( feature[ h ], axis_t, variance, dim ) );
      
      // add rotation by each 90 deg.
      if( c3_hlac_flg == 1 ){
	std::vector<float> feature_rotate;
	std::vector<float> feature_rotate_pre = feature[ h ];
	std::vector<float> feature_rotate_pre2;
	
	for(int t=0;t<3;t++){
	  pcl::rotateFeature90( feature_rotate,feature_rotate_pre,pcl::R_MODE_2);
	  pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	  feature_rotate_pre = feature_rotate;
	}
      
	pcl::rotateFeature90( feature_rotate,feature[ h ],pcl::R_MODE_3);
	pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	feature_rotate_pre  = feature_rotate;
	feature_rotate_pre2 = feature_rotate;
      
	for(int t=0;t<3;t++){
	  pcl::rotateFeature90( feature_rotate,feature_rotate_pre,pcl::R_MODE_2);
	  pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	  feature_rotate_pre = feature_rotate;
	}
      
	pcl::rotateFeature90( feature_rotate,feature_rotate_pre2,pcl::R_MODE_3);
	pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	feature_rotate_pre = feature_rotate;
	feature_rotate_pre2 = feature_rotate;
      
	for(int t=0;t<3;t++){
	  pcl::rotateFeature90( feature_rotate,feature_rotate_pre,pcl::R_MODE_2);
	  pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	  feature_rotate_pre = feature_rotate;
	}
      
	pcl::rotateFeature90( feature_rotate,feature_rotate_pre2,pcl::R_MODE_3);
	pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	feature_rotate_pre = feature_rotate;
      
	for(int t=0;t<3;t++){
	  pcl::rotateFeature90( feature_rotate,feature_rotate_pre,pcl::R_MODE_2);
	  pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	  feature_rotate_pre = feature_rotate;
	}
      
	pcl::rotateFeature90( feature_rotate,feature[ h ],pcl::R_MODE_1);
	pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	feature_rotate_pre = feature_rotate;
      
	for(int t=0;t<3;t++){
	  pcl::rotateFeature90( feature_rotate,feature_rotate_pre,pcl::R_MODE_2);
	  pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	  feature_rotate_pre = feature_rotate;
	}
      
	pcl::rotateFeature90( feature_rotate,feature[ h ],pcl::R_MODE_4);
	pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	feature_rotate_pre = feature_rotate;
      
	for(int t=0;t<3;t++){
	  pcl::rotateFeature90( feature_rotate,feature_rotate_pre,pcl::R_MODE_2);
	  pca_each.addData( compressFeature( feature_rotate, axis_t, variance, dim ) );
	  feature_rotate_pre = feature_rotate;
	}
      }
      //}
    }
  }

  //* solve PCA and write results
  pca_each.solve();  
  sprintf(tmpname,"%s/models/%s/pca_result",argv[1],argv[2]);
  pca_each.write( tmpname, ASCII_MODE_P );

  return 0;
}
