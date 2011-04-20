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
/* calculate projection axis for feature compression                                                       */
/*   Note that this process is necessary only once when the system sees the environment for the first time */
/***********************************************************************************************************/

#include <iostream>
#include <c3_hlac/c3_hlac_tools.h>
#include <color_voxel_recognition/pca.h>
#include <color_voxel_recognition/param.h>
#include <color_voxel_recognition/FILE_MODE>

int main(int argc, char** argv){
  if( argc != 3 ){
    std::cerr << "usage: " << argv[0] << " [path] <registration_num>" << std::endl;
    exit( EXIT_FAILURE );
  }
  const int file_num = atoi( argv[2] );
  char tmpname[ 1000 ];
  
  PCA pca( false );
  std::vector< std::vector<float> > feature;
  for( int i=0; i<file_num; i++ ){
    sprintf( tmpname, "%s/scene/Features/%05d.pcd", argv[1], i );
    readFeature( tmpname, feature );
    const int hist_num = feature.size();

    for( int h=0; h<hist_num; h++ ){
      //if( !if_zero_vec( feature[ h ] ) ){ 
      pca.addData( feature[ h ] );

      // std::vector<float> feature_rotate;
      // std::vector<float> feature_rotate_pre = feature[ h ];
      // std::vector<float> feature_rotate_pre2;
      
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
      
      // rotateFeature90( feature_rotate,feature[ h ],R_MODE_3);
      // pca.addData( feature_rotate );
      // feature_rotate_pre  = feature_rotate;
      // feature_rotate_pre2 = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
      // }
	
      // rotateFeature90( feature_rotate,feature_rotate_pre2,R_MODE_3);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
      // feature_rotate_pre2 = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
	
      // rotateFeature90( feature_rotate,feature_rotate_pre2,R_MODE_3);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
	
      // rotateFeature90( feature_rotate,feature[ h ],R_MODE_1);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
	
      // rotateFeature90( feature_rotate,feature[ h ],R_MODE_4);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
      //}
    }
  }

  //* solve PCA and write results
  pca.solve();
  sprintf( tmpname, "%s/scene/pca_result", argv[1] );
  pca.write( tmpname, ASCII_MODE_P );

  return 0;
}
