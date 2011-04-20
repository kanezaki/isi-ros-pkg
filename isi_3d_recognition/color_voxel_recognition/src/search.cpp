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

#define QUIET

#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include "color_voxel_recognition/pca.h"
#include "color_voxel_recognition/search.h"

// time
double my_clock_()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

//******************************//
//* constructor and destructor *//
//******************************//

SearchObj::SearchObj() :
  max_x(NULL),
  max_y(NULL),
  max_z(NULL),
  max_mode(NULL),
  max_dot (NULL),
  exist_voxel_num(NULL),
  integral_features(NULL),
  compress_flg(false) {
}

SearchObj::~SearchObj(){
  if( max_x != NULL ) delete[] max_x;
  if( max_y != NULL ) delete[] max_y;
  if( max_z != NULL ) delete[] max_z;
  if( max_mode != NULL ) delete[] max_mode;
  if( max_dot  != NULL ) delete[] max_dot;
  if( exist_voxel_num != NULL ) delete[] exist_voxel_num;
  if( integral_features != NULL ) delete[] integral_features;
}

SearchObjMulti::SearchObjMulti() :
  model_num(0),
  max_x_multi(NULL),
  max_y_multi(NULL),
  max_z_multi(NULL),
  max_mode_multi(NULL),
  max_dot_multi (NULL),
  axis_q_multi (NULL) {
}

SearchObjMulti::~SearchObjMulti(){
  if( max_x_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_x_multi[i];
    delete max_x_multi;
    max_x_multi = NULL;
  }
  if( max_y_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_y_multi[i];
    delete max_y_multi;
    max_y_multi = NULL;
  }
  if( max_z_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_z_multi[i];
    delete max_z_multi;
    max_z_multi = NULL;
  }
  if( max_mode_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_mode_multi[i];
    delete max_mode_multi;
    max_mode_multi = NULL;
  }
  if( max_dot_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_dot_multi[i];
    delete max_dot_multi;
    max_dot_multi = NULL;
  }
  if( exist_voxel_num != NULL ) delete[] exist_voxel_num;
  if( integral_features != NULL ) delete[] integral_features;
}

//*****************//
//* set variables *//
//*****************//

//*************************
// set size of sliding box
void SearchObj::setRange( int _range1, int _range2, int _range3 ){
  range1 = _range1;
  range2 = _range2;
  range3 = _range3;
}

//***********************************************************************************
// set number of output regions which are $rank_num most similar to the target object
void SearchObj::setRank( int _rank_num ){
  rank_num = _rank_num;
  if( max_x != NULL ) delete[] max_x;
  if( max_y != NULL ) delete[] max_y;
  if( max_z != NULL ) delete[] max_z;
  if( max_mode != NULL ) delete[] max_mode;
  if( max_dot  != NULL ) delete[] max_dot;
  max_x = new int[ rank_num ];
  max_y = new int[ rank_num ];
  max_z = new int[ rank_num ];
  max_mode = new SearchMode[ rank_num ];
  max_dot = new double[ rank_num ];
  for( int i=0; i<rank_num; i++ )  max_dot[ i ] = 0;
}

//***********************************************************
// set threshold of number of occupied voxels in a box region
void SearchObj::setThreshold( int _exist_voxel_num_threshold ){
  exist_voxel_num_threshold = _exist_voxel_num_threshold;
}

//*****************************************************
// read projection axis of the target object's subspace
void SearchObj::readAxis( const char *filename, int dim, int dim_model, bool ascii, bool multiple_similarity ){
  PCA pca_each;
  pca_each.read( filename, ascii );
  Eigen::MatrixXf tmpaxis = pca_each.getAxis();
  Eigen::MatrixXf tmpaxis2 = tmpaxis.block(0,0,tmpaxis.rows(),dim_model);
  axis_q = tmpaxis2.transpose();
  if( multiple_similarity ){
    Eigen::VectorXf variance = pca_each.getVariance();
    for( int i=1; i<dim_model; i++ )
      for( int j=0; j<dim; j++ )
	axis_q( i, j ) = axis_q( i, j ) * sqrt( variance( i ) ) / sqrt( variance( 0 ) );
  }
}

//*********************************************************
// read integral feature table (and voxel numbers) of scene
void SearchObj::readData( const char *filenameF, const char *filenameN, int dim, bool ascii ){
  double tmpval_d;

  FILE *fp, *fp2;
  if( ascii ){
    fp = fopen( filenameF, "r" );
    if( fscanf(fp,"%d %d %d\n",&x_num,&y_num,&z_num) == EOF ) std::cerr<< "fscanf err" << std::endl;
    fp2 = fopen( filenameN,"r" );
  }
  else{
    fp = fopen( filenameF, "rb" );
    if( fread(&x_num,sizeof(int),1,fp) < 1 ) std::cerr<< "fread err" << std::endl;
    if( fread(&y_num,sizeof(int),1,fp) < 1 ) std::cerr<< "fread err" << std::endl;
    if( fread(&z_num,sizeof(int),1,fp) < 1 ) std::cerr<< "fread err" << std::endl;
    fp2 = fopen( filenameN, "rb" );
  }

  xy_num = x_num*y_num;
  const int xyz_num = xy_num * z_num;
  exist_voxel_num = new int[ xyz_num ];
  integral_features = new Eigen::VectorXf [ xyz_num ];
  for(int n=0;n<xyz_num;n++){
    integral_features[ n ].resize(dim);
    if( ascii ){
      int tmpval;
      for(int j=0;j<dim;j++){
	if( fscanf(fp,"%d:%lf ",&tmpval,&tmpval_d) == EOF ) std::cerr<< "fscanf err" << std::endl;
	integral_features[ n ][ j ] = tmpval_d;
      }
      if( fscanf(fp,"\n") == EOF ) std::cerr<< "fscanf err" << std::endl;
      if( fscanf(fp2,"%d\n",exist_voxel_num+n) == EOF ) std::cerr<< "fscanf err" << std::endl;
    }
    else{
      for(int j=0;j<dim;j++){
	if( fread(&tmpval_d,sizeof(tmpval_d),1,fp) < 1 ) std::cerr<< "fread err" << std::endl;
	integral_features[ n ][ j ] = tmpval_d;
      }
      if( fread(exist_voxel_num+n,sizeof(exist_voxel_num[n]),1,fp2) < 1 ) std::cerr<< "fread err" << std::endl;
    }
  }
  fclose(fp);
}

//************************//
//* get sliding box size *//
//************************//

//*************************************************
// return xrange, yrange and zrange of sliding box
inline void SearchObj::getRange( int &xrange, int &yrange, int &zrange, SearchMode mode ){
  switch( mode ){
  case S_MODE_1:
    xrange = range1;
    yrange = range2;
    zrange = range3;
    break;
  case S_MODE_2:
    xrange = range1;
    yrange = range3;
    zrange = range2;
    break;
  case S_MODE_3:
    xrange = range2;
    yrange = range1;
    zrange = range3;
    break;
  case S_MODE_4:
    xrange = range2;
    yrange = range3;
    zrange = range1;
    break;
  case S_MODE_5:
    xrange = range3;
    yrange = range1;
    zrange = range2;
    break;
  case S_MODE_6:
    xrange = range3;
    yrange = range2;
    zrange = range1;
    break;
  }
}

//*****************************
// return xrange of sliding box
inline int SearchObj::xRange( SearchMode mode ){
  switch( mode ){
  case S_MODE_1:
  case S_MODE_2:
    return range1;
    break;
  case S_MODE_3:
  case S_MODE_4:
    return range2;
    break;
  case S_MODE_5:
  case S_MODE_6:
    return range3;
    break;
  default:
    break;
  }
  return 0;
}

//*****************************
// return yrange of sliding box
inline int SearchObj::yRange( SearchMode mode ){
  switch( mode ){
  case S_MODE_3:
  case S_MODE_5:
    return range1;
    break;
  case S_MODE_1:
  case S_MODE_6:
    return range2;
    break;
  case S_MODE_2:
  case S_MODE_4:
    return range3;
    break;
  default:
    break;
  }
  return 0;
}

//*****************************
// return zrange of sliding box
inline int SearchObj::zRange( SearchMode mode ){
  switch( mode ){
  case S_MODE_4:
  case S_MODE_6:
    return range1;
    break;
  case S_MODE_2:
  case S_MODE_5:
    return range2;
    break;
  case S_MODE_1:
  case S_MODE_3:
    return range3;
    break;
  default:
    break;
  }
  return 0;
}

//*******************//
//* storing results *//
//*******************//

//****************************************************************************
// check if the reference region is overlapped by some region already detected
//   return the rank number of the overlapped region if there is,
//   return the lowest rank number otherwise.
inline int SearchObj::checkOverlap( int x, int y, int z, SearchMode mode ){
  int num;
  int xrange = 0, yrange = 0, zrange = 0;
  int val1, val2, val3;
  getRange( xrange, yrange, zrange, mode );

  for( num = 0; num < rank_num-1; num++ ){
    val1 = max_x[ num ] - x;
    if( val1 < 0 )
      val1 = - val1 - xRange( max_mode[ num ] );
    else
      val1 -= xrange;

    val2 = max_y[ num ] - y;
    if( val2 < 0 )
      val2 = - val2 - yRange( max_mode[ num ] );
    else
      val2 -= yrange;

    val3 = max_z[ num ] - z;
    if( val3 < 0 )
      val3 = - val3 - zRange( max_mode[ num ] );
    else
      val3 -= zrange;

    if( ( val1 <= 0 ) && ( val2 <= 0 ) && ( val3 <= 0 ) )
      return num;
  }
  return num;
}

//******************************************************************************
// copy the info of $src_num-th detected region to $dest_num-th detected region
inline void SearchObj::maxCpy( int src_num, int dest_num ){
  max_dot[ dest_num ] = max_dot[ src_num ];
  max_x[ dest_num ] = max_x[ src_num ];
  max_y[ dest_num ] = max_y[ src_num ];
  max_z[ dest_num ] = max_z[ src_num ];
  max_mode[ dest_num ] = max_mode[ src_num ];
}

//*************************************************
// replace the info of $dest_num-th detected region
inline void SearchObj::maxAssign( int dest_num, double dot, int x, int y, int z, SearchMode mode ){
  max_dot[ dest_num ] = dot;
  max_x[ dest_num ] = x;
  max_y[ dest_num ] = y;
  max_z[ dest_num ] = z;
  max_mode[ dest_num ] = mode;
}

//********************//
//* object detection *//
//********************//

//*******************************
// object detection with rotation
void SearchObj::search(){
  double t1,t2; // time
  t1 = my_clock_();
  if( range1 == range2 ){
    if( range2 == range3 ){ // range1 = range2 = range3
      searchPart( S_MODE_1 );
    }
    else{ // range1 = range2
      searchPart( S_MODE_1 );
      searchPart( S_MODE_2 );
      searchPart( S_MODE_5 );
    }
  }
  else if( range2 == range3 ){ // range2 = range3
    searchPart( S_MODE_1 );
    searchPart( S_MODE_5 );
    searchPart( S_MODE_6 );
  }
  else if( range1==range3 ){ // range1 = range3
    searchPart( S_MODE_1 );
    searchPart( S_MODE_5 );
    searchPart( S_MODE_3 );
  }
  else{ // range1 != range2 != range3
    searchPart( S_MODE_1 );
    searchPart( S_MODE_2 );
    searchPart( S_MODE_3 );
    searchPart( S_MODE_4 );
    searchPart( S_MODE_5 );
    searchPart( S_MODE_6 );
  }
  t2 = my_clock_();
  search_time = t2 - t1;
}

//**********************************
// object detection without rotation
void SearchObj::searchWithoutRotation(){
  double t1,t2; // time
  t1 = my_clock_();
  searchPart( S_MODE_1 );
  t2 = my_clock_();
  search_time = t2 - t1;
}

//**************************************
// object detection (in eash SearchMode)
void SearchObj::searchPart( SearchMode mode ){
  int xrange = 0, yrange = 0, zrange = 0;
  getRange( xrange, yrange, zrange, mode );

  const int x_end = x_num - xrange + 1;
  const int y_end = y_num - yrange + 1;
  const int z_end = z_num - zrange + 1;
  
  Eigen::VectorXf feature_tmp;
  Eigen::VectorXf tmpVector;
  double dot,sum;
  int overlap_num;
  int exist_num;
  if((x_end>0)&&(y_end>0)&&(z_end>0)){    
    for(int z=0;z<z_end;z++){	      
      for(int y=0;y<y_end;y++){
	for(int x=0;x<x_end;x++){

	  //* check if there are some occupied voxels in reference region
	  exist_num = clipValue( exist_voxel_num, x, y, z, xrange, yrange, zrange );
	  if(exist_num > exist_voxel_num_threshold){ // if there are

	    feature_tmp = clipValue( integral_features, x, y, z, xrange, yrange, zrange );
	    
	    //* similarity calculation
	    sum = feature_tmp.dot( feature_tmp );
	    tmpVector = axis_q * feature_tmp;
	    sum = sqrt(sum);
	    dot = tmpVector.dot( tmpVector );
	    dot = sqrt( dot );
	    dot /= sum ;
		
	    //* store region info if its similarity is high
	    for( int i=0; i<rank_num; i++ ){
	      if(dot>max_dot[ i ]){
		overlap_num = checkOverlap( x, y, z, mode );
		for( int j=0; j<overlap_num-i; j++ )
		  maxCpy( overlap_num-1-j, overlap_num-j );		  

		if( i<=overlap_num )
		  maxAssign( i, dot, x, y, z, mode );
		break;
	      }
	    }
	  }
	}
      }
    }
  }
}

//***********************************************************************************
// extract features and voxel numbers of reference region from integral feature table
template <typename T>
T SearchObj::clipValue( T* ptr, const int x, const int y, const int z, const int xrange, const int yrange, const int zrange ) {
  T result;
  if(z==0){
    if(y==0){
      if(x==0) // (0,0,0)
        result = ptr[ xrange-1 + (yrange-1)*x_num + (zrange-1)*xy_num ];
      else // (*,0,0)
        result = ptr[ x+xrange-1 + (yrange-1)*x_num + (zrange-1)*xy_num ]
          - ptr[ x-1 + (yrange-1)*x_num + (zrange-1)*xy_num ];
    }
    else{
      if(x==0) // (0,*,0)
        result = ptr[ xrange-1 + (y+yrange-1)*x_num + (zrange-1)*xy_num ]
          - ptr[ xrange-1 + (y-1)*x_num + (zrange-1)*xy_num ];
      else // (*,*,0)
        result = ptr[ x+xrange-1 + (y+yrange-1)*x_num + (zrange-1)*xy_num ]
          - ptr[ x-1 + (y+yrange-1)*x_num + (zrange-1)*xy_num ]
          - ptr[ x+xrange-1 + (y-1)*x_num + (zrange-1)*xy_num ]
          + ptr[ x-1 + (y-1)*x_num + (zrange-1)*xy_num ];
    }
  }
  else{
    if(y==0){
      if(x==0) // (0,0,*)	
        result = ptr[ xrange-1 + (yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ xrange-1 + (yrange-1)*x_num + (z-1)*xy_num ];
      else // (*,0,*)
        result = ptr[ x+xrange-1 + (yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x-1 + (yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x+xrange-1 + (yrange-1)*x_num + (z-1)*xy_num ]
          + ptr[ x-1 + (yrange-1)*x_num + (z-1)*xy_num ];
    }
    else{
      if(x==0) // (0,*,*)
        result = ptr[ xrange-1 + (y+yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ xrange-1 + (y+yrange-1)*x_num + (z-1)*xy_num ]
          - ptr[ xrange-1 + (y-1)*x_num + (z+zrange-1)*xy_num ]
          + ptr[ xrange-1 + (y-1)*x_num + (z-1)*xy_num ];
      else // (*,*,*)
        result = ptr[ x+xrange-1 + (y+yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x-1 + (y+yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x+xrange-1 + (y-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x+xrange-1 + (y+yrange-1)*x_num + (z-1)*xy_num ]
          + ptr[ x-1 + (y-1)*x_num + (z+zrange-1)*xy_num ]
          + ptr[ x-1 + (y+yrange-1)*x_num + (z-1)*xy_num ]
          + ptr[ x+xrange-1 + (y-1)*x_num + (z-1)*xy_num ]
          - ptr[ x-1 + (y-1)*x_num + (z-1)*xy_num ];
    }
  }
  return result;
}

//********************************************************
// set integral feature table (and voxel numbers) of scene
void SearchObj::setData( const Eigen::Vector3i subdiv_b_, std::vector< std::vector<float> > feature ){
  x_num = subdiv_b_[0];
  y_num = subdiv_b_[1];
  z_num = subdiv_b_[2];
  std::cout << x_num << " " << y_num << " " << z_num << std::endl;
  xy_num = x_num*y_num;
  const int xyz_num = xy_num * z_num;
  if( xyz_num < 1 )
    return;

  // exist_voxel_num = new int[ xyz_num ];
  // integral_features = new Eigen::VectorXf [ xyz_num ];

  //**********************//
  //* feature extraction *//
  //**********************//
  int idx = 0;
  const int dim_feature = feature_max.size(); // = 137
  //const int lower = 0;
  const int upper = 1;
  for(int z=0;z<z_num;z++){
    for(int y=0;y<y_num;y++){
      for(int x=0;x<x_num;x++){

	//* histogram normalization
	if( dim_feature > 0 ){
	  for( int t=0; t<dim_feature; t++ ){
	    if( feature_max[ t ] == 0 ) feature[ idx ][ t ] = 0;
	    else if( feature[ idx ][ t ] == feature_max[ t ] ) feature[ idx ][ t ] = upper;
	    else feature[ idx ][ t ] = upper * feature[ idx ][ t ] / feature_max[ t ];
	  }
	}

	Eigen::Map<Eigen::VectorXf> vec( &(feature[ idx ][0]), feature[ idx ].size() );
	if( compress_flg )
	  integral_features[ idx ] = axis_p * vec;
	else
	  integral_features[ idx ] = vec;

	//*******************************//
	//* make integral feature table *//
	//*******************************//
	    
	if(z==0){
	  if(y==0){
	    if(x!=0){ // (*,0,0)
	      exist_voxel_num[ idx ] += exist_voxel_num[ ( x - 1 ) + y*x_num + z*xy_num ];
	      integral_features[ idx ] += integral_features[ ( x - 1 ) + y*x_num + z*xy_num ];
	    }
	  }
	  else{
	    if(x==0){ // (0,*,0)
	      exist_voxel_num[ idx ] += exist_voxel_num[ x + ( y - 1 )*x_num + z*xy_num ];	    
	      integral_features[ idx ] += integral_features[ x + ( y - 1 )*x_num + z*xy_num ];	    
	    }
	    else{ // (*,*,0)
	      exist_voxel_num[ idx ] 
		+= exist_voxel_num[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  exist_voxel_num[ x + ( y - 1 )*x_num + z*xy_num ]
		-  exist_voxel_num[ ( x - 1 ) + ( y - 1 )*x_num + z*xy_num ];
	      integral_features[ idx ]
		+= integral_features[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  integral_features[ x + ( y - 1 )*x_num + z*xy_num ]
		-  integral_features[ ( x - 1 ) + ( y - 1 )*x_num + z*xy_num ];
	    }
	  }
	}
	else{
	  if(y==0){
	    if(x==0){ // (0,0,*)	
	      exist_voxel_num[ idx ] += exist_voxel_num[ x + y*x_num + ( z - 1 )*xy_num ];
	      integral_features[ idx ] += integral_features[ x + y*x_num + ( z - 1 )*xy_num ];
	    }
	    else {// (*,0,*)
	      exist_voxel_num[ idx ] 
		+= exist_voxel_num[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  exist_voxel_num[ x + y *x_num + ( z - 1 )*xy_num ]
		-  exist_voxel_num[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ];
	      integral_features[ idx ]
		+= integral_features[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  integral_features[ x + y *x_num + ( z - 1 )*xy_num ]
		-  integral_features[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ];
	    }
	  }
	  else{
	    if(x==0){ // (0,*,*)
	      exist_voxel_num[ idx ] 
		+= exist_voxel_num[ x + ( y - 1 )*x_num + z *xy_num ]
		+  exist_voxel_num[ x + y *x_num + ( z - 1 )*xy_num ]
		-  exist_voxel_num[ x + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	      integral_features[ idx ]
		+= integral_features[ x + ( y - 1 )*x_num + z *xy_num ]
		+  integral_features[ x + y *x_num + ( z - 1 )*xy_num ]
		-  integral_features[ x + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	    }
	    else{ // (*,*,*)
	      exist_voxel_num[ idx ] 
		+= exist_voxel_num[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  exist_voxel_num[ x + ( y - 1 )*x_num + z*xy_num ]
		+  exist_voxel_num[ x + y *x_num + ( z - 1 )*xy_num ]
		-  exist_voxel_num[ ( x - 1 ) + ( y - 1 )*x_num + z *xy_num ]
		-  exist_voxel_num[ x + ( y - 1 )*x_num + ( z - 1 )*xy_num ]
		-  exist_voxel_num[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ]
		+  exist_voxel_num[ ( x - 1 ) + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	      integral_features[ idx ] 
		+= integral_features[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  integral_features[ x + ( y - 1 )*x_num + z*xy_num ]
		+  integral_features[ x + y *x_num + ( z - 1 )*xy_num ]
		-  integral_features[ ( x - 1 ) + ( y - 1 )*x_num + z *xy_num ]
		-  integral_features[ x + ( y - 1 )*x_num + ( z - 1 )*xy_num ]
		-  integral_features[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ]
		+  integral_features[ ( x - 1 ) + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	    }
	  }
	}
	idx++;
      }
    }
  }
}

//*****************************
// output the results in a file
void SearchObj::writeResult( const char *filename, int box_size ){
  FILE *fp = fopen(filename,"w");
  int xrange = 0, yrange = 0, zrange = 0;
  for( int r=0; r<rank_num; r++ ){
    if( max_dot[ r ] == 0 ) break;
    int x_tmp_min = max_x[ r ] * box_size;
    int y_tmp_min = max_y[ r ] * box_size;
    int z_tmp_min = max_z[ r ] * box_size;
    getRange( xrange, yrange, zrange, max_mode[ r ] );
    int x_tmp_max = (max_x[ r ] + xrange) * box_size;
    int y_tmp_max = (max_y[ r ] + yrange) * box_size;
    int z_tmp_max = (max_z[ r ] + zrange) * box_size;
    //printf("max: %4d~%4d %4d~%4d %4d~%4d | max_dot: %f\n",x_tmp_min,x_tmp_max,y_tmp_min,y_tmp_max,z_tmp_min,z_tmp_max,max_dot[ r ]);
    fprintf(fp,"%d %d %d %d %d %d %f\n",x_tmp_min,x_tmp_max-x_tmp_min,y_tmp_min,y_tmp_max-y_tmp_min,z_tmp_min,z_tmp_max-z_tmp_min,max_dot[ r ]);
  }    
  fprintf(fp,"time: %f\n",search_time);
  fclose(fp);
}

//*******************
// delete the results
void SearchObj::cleanMax(){
  for( int i=0; i<rank_num; i++ ){
    max_x[ i ] = 0;
    max_y[ i ] = 0;
    max_z[ i ] = 0;
    max_dot[ i ] = 0;
  }
}

//****************************************************************
// set projection axis for feature compression (without whitening)
void SearchObj::setSceneAxis( Eigen::MatrixXf _axis ){
  axis_p = _axis;
  compress_flg = true;
}

//**************************************************************
// set projection axis for feature compression (with whitening)
void SearchObj::setSceneAxis( Eigen::MatrixXf _axis, Eigen::VectorXf var, int dim ){
  const int rows = _axis.rows();
  const int cols = _axis.cols();
  for( int i=0; i<rows; i++ ){
    for( int j=0; j<cols; j++ ){
      const float tmpval = 1/ sqrt( var( i ) );
      _axis( i, j ) = tmpval * _axis( i, j ) ;
    }
  }
  axis_p = _axis;
  compress_flg = true;
}

//***********************************************
// delete the results and integral feature table
void SearchObj::cleanData(){
  x_num = 0;
  y_num = 0;
  z_num = 0;
  xy_num = 0;
  for( int i=0; i<rank_num; i++ ){
    max_x[ i ] = 0;
    max_y[ i ] = 0;
    max_z[ i ] = 0;
    //max_mode[ i ] = 0;
    max_dot[ i ] = 0;
  }
  if( exist_voxel_num != NULL ) delete[] exist_voxel_num;
  if( integral_features != NULL ) delete[] integral_features;
  exist_voxel_num = NULL;
  integral_features = NULL;
}

//*******************************************************
// return x, y and z length of ($num-th) detected region
int SearchObj::maxXrange( int num ){ return xRange( max_mode[ num ] ); }
int SearchObj::maxYrange( int num ){ return yRange( max_mode[ num ] ); }
int SearchObj::maxZrange( int num ){ return zRange( max_mode[ num ] ); }

//*************************************
// set values for feature normalization
void SearchObj::setNormalizeVal( const char* filename ){ 
  FILE *fp = fopen( filename, "r" );
  float val;
  while( fscanf( fp, "%f\n", &val ) != EOF )
    feature_max.push_back( val );
  fclose( fp );    
}

//----------------------------//
// multiple objects detection //
//----------------------------//

//**********************************************
// delete the results and integral feature table
void SearchObjMulti::cleanData(){
  x_num = 0;
  y_num = 0;
  z_num = 0;
  xy_num = 0;
  for( int m=0; m<model_num; m++ ){
    for( int i=0; i<rank_num; i++ ){
      max_x_multi[ m ][ i ] = 0;
      max_y_multi[ m ][ i ] = 0;
      max_z_multi[ m ][ i ] = 0;
      //max_mode_multi[ m ][ i ] = 0;
      max_dot_multi[ m ][ i ] = 0;
    }
  }
  if( exist_voxel_num != NULL ) delete[] exist_voxel_num;
  if( integral_features != NULL ) delete[] integral_features;
  exist_voxel_num = NULL;
  integral_features = NULL;
}

//***********************************************************************************
// set number of output regions which are $rank_num most similar to the target object
void SearchObjMulti::setRank( int _rank_num ){
  rank_num = _rank_num;

  if( max_x_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_x_multi[i];
    delete max_x_multi;
  }
  if( max_y_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_y_multi[i];
    delete max_y_multi;
  }
  if( max_z_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_z_multi[i];
    delete max_z_multi;
  }
  if( max_mode_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_mode_multi[i];
    delete max_mode_multi;
  }
  if( max_dot_multi != NULL ){
    for( int i=0; i<model_num; i++ ) delete[] max_dot_multi[i];
    delete max_dot_multi;
  }
  max_x_multi = new int*[ model_num ];
  max_y_multi = new int*[ model_num ];
  max_z_multi = new int*[ model_num ];
  max_mode_multi = new SearchMode*[ model_num ];
  max_dot_multi = new double*[ model_num ];

  for( int m=0; m<model_num; m++ ){
    max_x_multi[ m ] = new int[ rank_num ];
    max_y_multi[ m ] = new int[ rank_num ];
    max_z_multi[ m ] = new int[ rank_num ];
    max_mode_multi[ m ] = new SearchMode[ rank_num ];
    max_dot_multi[ m ] = new double[ rank_num ];
    for( int i=0; i<rank_num; i++ )  max_dot_multi[ m ][ i ] = 0;
  }
}

//*****************************************************
// read projection axis of the target object's subspace
void SearchObjMulti::readAxis( char **filename, int dim, int dim_model, bool ascii, bool multiple_similarity ){
  if( axis_q_multi != NULL ) delete[] axis_q_multi;
  axis_q_multi = new Eigen::MatrixXf [ model_num ];

  for( int m=0; m<model_num; m++ ){
    PCA pca_each;
    pca_each.read( filename[ m ], ascii );
    std::cout << filename[ m ] << std::endl;
    Eigen::MatrixXf tmpaxis = pca_each.getAxis();
    Eigen::MatrixXf tmpaxis2 = tmpaxis.block(0,0,tmpaxis.rows(),dim_model);
    axis_q_multi[ m ] = tmpaxis2.transpose();
    if( multiple_similarity ){
      Eigen::VectorXf variance = pca_each.getVariance();
      for( int i=1; i<dim_model; i++ )
	for( int j=0; j<dim; j++ )
	  axis_q_multi[ m ]( i, j ) = axis_q_multi[ m ]( i, j ) * sqrt( variance( i ) ) / sqrt( variance( 0 ) );
    }
  }
}

//*******************
// delete the results
void SearchObjMulti::cleanMax(){
  for( int m=0; m<model_num; m++ ){
    for( int i=0; i<rank_num; i++ ){
      max_x_multi[ m ][ i ] = 0;
      max_y_multi[ m ][ i ] = 0;
      max_z_multi[ m ][ i ] = 0;
      max_dot_multi[ m ][ i ] = 0;
    }
  }
}

//*******************************************************
// return x, y and z length of ($num-th) detected region
int SearchObjMulti::maxXrange( int m_num, int num ){ return xRange( max_mode_multi[ m_num ][ num ] ); }
int SearchObjMulti::maxYrange( int m_num, int num ){ return yRange( max_mode_multi[ m_num ][ num ] ); }
int SearchObjMulti::maxZrange( int m_num, int num ){ return zRange( max_mode_multi[ m_num ][ num ] ); }

//****************************************************************************
// check if the reference region is overlapped by some region already detected
//   return the rank number of the overlapped region if there is,
//   return the lowest rank number otherwise.
inline int SearchObjMulti::checkOverlap( int m_num, int x, int y, int z, SearchMode mode ){
  int num;
  int xrange = 0, yrange = 0, zrange = 0;
  int val1, val2, val3;
  getRange( xrange, yrange, zrange, mode );

  for( num = 0; num < rank_num-1; num++ ){
    val1 = max_x_multi[ m_num ][ num ] - x;
    if( val1 < 0 )
      val1 = - val1 - xRange( max_mode_multi[ m_num ][ num ] );
    else
      val1 -= xrange;

    val2 = max_y_multi[ m_num ][ num ] - y;
    if( val2 < 0 )
      val2 = - val2 - yRange( max_mode_multi[ m_num ][ num ] );
    else
      val2 -= yrange;

    val3 = max_z_multi[ m_num ][ num ] - z;
    if( val3 < 0 )
      val3 = - val3 - zRange( max_mode_multi[ m_num ][ num ] );
    else
      val3 -= zrange;

    if( ( val1 <= 0 ) && ( val2 <= 0 ) && ( val3 <= 0 ) )
      return num;
  }
  return num;
}

//******************************************************************************
// copy the info of $src_num-th detected region to $dest_num-th detected region
inline void SearchObjMulti::maxCpy( int m_num, int src_num, int dest_num ){
  max_dot_multi[ m_num ][ dest_num ] = max_dot_multi[ m_num ][ src_num ];
  max_x_multi[ m_num ][ dest_num ] = max_x_multi[ m_num ][ src_num ];
  max_y_multi[ m_num ][ dest_num ] = max_y_multi[ m_num ][ src_num ];
  max_z_multi[ m_num ][ dest_num ] = max_z_multi[ m_num ][ src_num ];
  max_mode_multi[ m_num ][ dest_num ] = max_mode_multi[ m_num ][ src_num ];
}

//*************************************************
// replace the info of $dest_num-th detected region
inline void SearchObjMulti::maxAssign( int m_num, int dest_num, double dot, int x, int y, int z, SearchMode mode ){
  max_dot_multi[ m_num ][ dest_num ] = dot;
  max_x_multi[ m_num ][ dest_num ] = x;
  max_y_multi[ m_num ][ dest_num ] = y;
  max_z_multi[ m_num ][ dest_num ] = z;
  max_mode_multi[ m_num ][ dest_num ] = mode;
}

//**************************************
// object detection (in eash SearchMode)
void SearchObjMulti::searchPart( SearchMode mode ){
  int xrange = 0, yrange = 0, zrange = 0;
  getRange( xrange, yrange, zrange, mode );

  const int x_end = x_num - xrange + 1;
  const int y_end = y_num - yrange + 1;
  const int z_end = z_num - zrange + 1;
  
  Eigen::VectorXf feature_tmp;
  Eigen::VectorXf tmpVector;
  double dot,sum;
  int overlap_num;
  int exist_num;
  if((x_end>0)&&(y_end>0)&&(z_end>0)){    
    for(int z=0;z<z_end;z++){	      
      for(int y=0;y<y_end;y++){
	for(int x=0;x<x_end;x++){

	  //* check if there are some occupied voxels in reference region
	  exist_num = clipValue( exist_voxel_num, x, y, z, xrange, yrange, zrange );
	  if(exist_num > exist_voxel_num_threshold){  // if there are

	    feature_tmp = clipValue( integral_features, x, y, z, xrange, yrange, zrange );
	    
	    //* similarity calculation
	    sum = feature_tmp.dot( feature_tmp );
	    sum = sqrt(sum);

	    for( int m=0; m<model_num; m++ ){
	      tmpVector = axis_q_multi[ m ] * feature_tmp;
	      dot = tmpVector.dot( tmpVector );
	      dot = sqrt( dot );
	      dot /= sum ;
	      
	      //* store region info if its similarity is high
	      for( int i=0; i<rank_num; i++ ){
		if(dot>max_dot_multi[ m ][ i ]){
		  overlap_num = checkOverlap( m, x, y, z, mode );
		  for( int j=0; j<overlap_num-i; j++ )
		    maxCpy( m, overlap_num-1-j, overlap_num-j );		  
		  
		  if( i<=overlap_num )
		    maxAssign( m, i, dot, x, y, z, mode );
		  break;
		}
	      }
	    }

	  }
	}
      }
    }
  }
}

//****************************************************************************
// solve and remove overlap between detected regions for object1, object2, ...
void SearchObjMulti::removeOverlap(){
  for( int m=0; m<model_num; m++ ){
    //for( int i=0; i<rank_num; i++ ){
    for( int i=0; i<1; i++ ){
      //for( int m2=m+1; m2<model_num; m2++ ){
      for( int m2=0; m2<model_num; m2++ ){
	if( m2 != m ){
	  int overlap_num = checkOverlap( m2, maxX( m, i ), maxY( m, i ), maxZ( m, i ), maxMode( m, i ) );
	  if( maxDot( m, i ) > maxDot( m2, overlap_num ) )
	    for( int j=overlap_num; j<rank_num-1; j++ )
	      maxCpy( m2, j+1, j );
	    //max_dot_multi[ m2 ][ overlap_num ] = 0;
	  else
	    for( int j=i; j<rank_num-1; j++ )
	      maxCpy( m, j+1, j );
	    //max_dot_multi[ m ][ i ] = 0;
	}
      }
    }	
  }
}
