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

/*******************************************************************************************************************/
/* show eigenvalues of a target object's subspace                                                                  */
/*  options:                                                                                                       */
/* -d <dim> : show the accumulative contribution rate of the dimension                                             */
/* -c <accumulative contribution rate> : show the dimension of the accumulative contribution rate of the dimension */
/*******************************************************************************************************************/

#include <iostream>
#include <stdio.h>
#include <color_voxel_recognition/param.h>
#include <color_voxel_recognition/pca.h>
#include <color_voxel_recognition/FILE_MODE>

int main(int argc, char** argv)
{
  if((argc!=3)&&(argc!=5)){
    std::cerr << "usage: " << argv[0] << " [path] [model_name]" << std::endl;
    std::cerr << " or" << std::endl;
    std::cerr << "usage: " << argv[0] << " [path] [model_name] -d(or -c) <val>" << std::endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 1000 ];
  sprintf( tmpname, "%s/models/%s/pca_result", argv[1], argv[2] );

  PCA pca_each;
  pca_each.read( tmpname ,ASCII_MODE_P );
  Eigen::VectorXf variance = pca_each.getVariance();

  // read the dimension of compressed feature vectors
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  const int dim = Param::readDim( tmpname );

  if( argc==3 ){
    for( int i=0; i<dim; i++ )
      printf("%f\n",variance(i));
  }
  else{
    double c_all = 0;
    for( int i=0; i<dim; i++ )
      c_all += variance( i );

    if( argv[3][1]=='d' ){
      const int d = atoi(argv[4]);
      double c_ = 0;
      for( int i=0; i<d; i++ )
	c_ += variance( i );
      printf("%f\n",c_/c_all);
    }
    else if( argv[3][1]=='c' ){
      const double c = atof(argv[4]);
      double c_ = 0;
      for( int i=0; i<dim; i++ ){
	c_ += variance( i );
	if( c_/c_all >= c ){
	  printf("%d\n",i);
	  return 1;
	}
      }
      printf("%d\n",dim);
    }
    else
      std::cerr << "ERR: option command must be \"-d\" or \"-c\"" << std::endl;
  }
}
