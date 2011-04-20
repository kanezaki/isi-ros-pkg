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

#include <cstdio>
#include <iostream>
#include <fstream>
#include "color_voxel_recognition/pca.h"

PCA::PCA( bool _mean_flg ) :
  mean_flg(_mean_flg),
  dim(-1),
  nsample(0) {
}

//**********************************************************
// add feature vectors to the correlation matrix one by one
void PCA::addData( const std::vector<float> feature ){
  if( dim == -1 ){ // initial value
    dim = feature.size();
    correlation = Eigen::MatrixXf::Zero( dim, dim );
    mean = Eigen::VectorXf::Zero( dim );
  }
  else if( feature.size() != (std::vector<float>::size_type)dim ){
    std::cerr << "ERR (in PCA::addData): vector size differs" << std::endl;
    exit( EXIT_FAILURE );
  }

  if(mean_flg)
    for( int i=0; i<dim; i++ )
      mean(i) += feature[i];
  for( int i = 0; i < dim; i++ ){
    const float val = feature[ i ];
    int idx = i + i * dim;
    for(int j = i; j < dim; j++ )
      correlation( idx++ ) += val * feature[ j ];
  }
  nsample++;
}

//***********
// solve PCA
void PCA::solve( bool regularization_flg, float regularization_nolm ){
  if( dim==-1 ){
    std::cerr << "ERR (in PCA::solve): there is no data" << std::endl;
    exit( EXIT_FAILURE );
  }
  printf("sample: %lld\n",nsample);

  const double inv_nsample = 1.0 / (double)nsample;
  for(int i = 0; i < dim; i++ ){
    int idx = i + i * dim;
    for( int j = i; j < dim; j++ )
      correlation( idx++ ) *= inv_nsample;
  }
  for( int i = 0; i < dim; i++ )
    for( int j = i + 1; j < dim; j++ )
      correlation( i, j ) = correlation( j, i );

  if( mean_flg ){
    for(int i = 0; i < dim; i++ )
      mean(i) *= inv_nsample;    
    correlation -= mean * mean.transpose();
  }

  if( regularization_flg )
    for(int i = 0; i < dim; i++ )
      correlation( i, i ) += regularization_nolm;

  //* solve eigen problem
  Eigen::SelfAdjointEigenSolver< Eigen::MatrixXf > pca ( correlation );
  Eigen::MatrixXf tmp_axis = pca.eigenvectors();
  Eigen::VectorXf tmp_variance = pca.eigenvalues();
  sortVecAndVal( tmp_axis, tmp_variance );  
}

//****************************************
// get the mean vector of feature vectors
const Eigen::VectorXf& PCA::getMean() const {
  if( !mean_flg ){
    std::cerr << "ERR (in PCA::getMean): There is no mean vector (mean_flg=false)." << std::endl;
    exit( EXIT_FAILURE );
  }
  return mean;
}

//***************
// read PCA file
void PCA::read( const char *filename, bool ascii )
{
  FILE *fp;
  
  if( ascii )
    fp = fopen( filename, "r" );
  else
    fp = fopen( filename, "rb" );

  if( ascii ){
    //* dimension of feature vectors
    if( fscanf( fp, "%d\n", &dim ) != 1 ) std::cerr << "fscanf err." << std::endl;

    //* eigen vectors
    axis.resize( dim, dim );
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	if( fscanf( fp, "%f ", &( axis( j, i ) ) ) != 1 ) std::cerr << "fscanf err." << std::endl;
    
    //* eigen values
    variance.resize( dim );
    for( int i=0; i<dim; i++ )
      if( fscanf( fp, "%f\n", &( variance( i ) ) ) != 1 ) std::cerr << "fscanf err." << std::endl;

    //* mean vector of feature vectors
    float tmpVal;
    if( fscanf( fp, "%f\n", &tmpVal ) != 1 ){
      mean_flg = false;
    }
    else{
      mean_flg = true;
      mean.resize( dim );
      mean( 0 ) = tmpVal;
      for( int i=1; i<dim; i++ )
	if( fscanf( fp, "%f\n", &( mean( i ) ) ) != 1 ) std::cerr << "fscanf err." << std::endl;
    }
  }else{
    //* dimension of feature vectors
    if( fread( &dim, sizeof(int), 1, fp ) != 1 ) std::cerr << "fread err." << std::endl;

    //* eigen vectors
    axis.resize( dim, dim );
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	if( fread( &( axis( j, i ) ), sizeof(float), 1, fp ) != 1 ) std::cerr << "fread err." << std::endl;

    //* eigen values
    variance.resize( dim );
    for( int i=0; i<dim; i++ )
      if( fread( &( variance( i ) ), sizeof(float), 1, fp ) != 1 ) std::cerr << "fread err." << std::endl;

    //* mean vector of feature vectors
    float tmpVal;
    if( fread( &tmpVal, sizeof(float), 1, fp ) != 1 ){
      mean_flg = false;
    }
    else{
      mean_flg = true;
      mean.resize( dim );
      mean( 0 ) = tmpVal;
      for( int i=1; i<dim; i++ )
	if( fread( &( mean( i ) ), sizeof(float), 1, fp ) != 1 ) std::cerr << "fread err." << std::endl;
    }
  }
  
  fclose( fp );
}


//****************
// write PCA file
void PCA::write( const char *filename, bool ascii )
{ 
  const int dim = variance.size();
  FILE *fp;
  
  if( ascii )
    fp = fopen( filename, "w" );
  else
    fp = fopen( filename, "wb" );

  if( ascii ){
    //* dimension of feature vectors
    fprintf( fp, "%d\n", dim );

    //* eigen vectors
    for( int i=0; i<dim; i++ ){
      for( int j=0; j<dim; j++ )
	fprintf( fp, "%f ", axis( j, i ) );
      fprintf( fp, "\n" );
    }

    //* eigen values
    for( int i=0; i<dim; i++ )
      fprintf( fp, "%f\n", variance( i ) );

    if( mean_flg )
      //* mean vector of feature vectors
      for( int i=0; i<dim; i++ )
	fprintf( fp, "%f\n", mean( i ) );

  }else{
    //* dimension of feature vectors
    fwrite( &dim, sizeof(int), 1, fp );

    //* eigen vectors
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	fwrite( &( axis( j, i ) ), sizeof(float), 1, fp );

    //* eigen values
    for( int i=0; i<dim; i++ )
      fwrite( &( variance( i ) ), sizeof(float), 1, fp );

    if( mean_flg )
      //* mean vector of feature vectors
      for( int i=0; i<dim; i++ )
	fwrite( &( mean( i ) ), sizeof(float), 1, fp );
  }
  
  fclose( fp );
}

//*******
//* sort
void PCA::sortVecAndVal( Eigen::MatrixXf &vecs, Eigen::VectorXf &vals ){
  int *index = new int[ dim ];
  for( int i = 0; i < dim; i++ )
    index[ i ] = i;

  //* sort
  int tmpIndex;
  for( int i = 0; i < dim; i++ ){
    for( int j = 1; j < dim - i; j++ ){
      if( vals( index[ j - 1 ] ) < vals( index[ j ] ) ){
	tmpIndex       = index[ j ];
	index[ j ]     = index[ j - 1 ];
	index[ j - 1 ] = tmpIndex;
      }
    }
  }
  
  //* copy
  axis.resize( dim, dim );
  variance.resize( dim );
  for( int i = 0; i < dim; i++ ){
    variance( i ) = vals( index[ i ] );
    for( int j = 0; j < dim; j++ )
      axis( j, i ) = vecs( j, index[ i ] );
  }

  delete[] index;  
}
