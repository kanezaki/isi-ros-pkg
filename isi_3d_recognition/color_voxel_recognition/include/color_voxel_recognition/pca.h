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

#ifndef COLOR_VOXEL_RECOGNITION_PCA_H_
#define COLOR_VOXEL_RECOGNITION_PCA_H_

#include <vector>
#include <Eigen/Eigenvalues>

/*****************/
/* class for PCA */
/*****************/

class PCA{
public:
  bool mean_flg;        // "true" when you substract the mean vector from the correlation matrix

  PCA( bool _mean_flg = true ); // _mean_flg should be "true" when you substract the mean vector from the correlation matrix
  
  ~PCA(){}
  
  // add feature vectors to the correlation matrix one by one
  void addData( const std::vector<float> feature );

  // solve PCA
  void solve( bool regularization_flg = false, float regularization_nolm = 0.0001 );
    
  // get eigen vectors
  const Eigen::MatrixXf &getAxis() const { return axis; }

  // get eigen values
  const Eigen::VectorXf &getVariance() const { return variance; }

  // get the mean vector of feature vectors
  const Eigen::VectorXf &getMean() const;
    
  // read PCA file
  void read( const char *filename, bool ascii = false );

  // write PCA file
  void write( const char *filename, bool ascii = false );
    
private:
  int dim;              // dimension of feature vectors
  long long nsample;    // number of feature vectors
  Eigen::VectorXf mean;        // mean vector of feature vectors
  Eigen::MatrixXf correlation; // self correlation matrix
  Eigen::MatrixXf axis;        // eigen vectors
  Eigen::VectorXf variance;    // eigen values

  void sortVecAndVal( Eigen::MatrixXf &vecs, Eigen::VectorXf &vals );
};

#endif
