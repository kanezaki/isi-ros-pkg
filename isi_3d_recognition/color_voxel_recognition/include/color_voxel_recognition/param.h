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

#ifndef COLOR_VOXEL_RECOGNITION_PARAM_H_
#define COLOR_VOXEL_RECOGNITION_PARAM_H_

/*****************************/
/* read and write parameters */
/*****************************/

class Param{
public:

  // read the length of voxel side (unit: meter)
  static float readVoxelSize( const char* filename = "param/parameters.txt" );

  // read the dimension of compressed feature vectors
  static int readDim( const char* filename = "param/parameters.txt" );

  // read the number of voxels in each subdivision's side of scene
  static int readBoxSizeScene( const char* filename = "param/parameters.txt" );

  // read the number of voxels in each subdivision's side of a target object
  static int readBoxSizeModel( const char* filename = "param/parameters.txt" );

  // // read the max number of voxels created once
  // static int readMaxVoxelNum( const char* filename = "param/parameters.txt" );

  // read the number for synthetic rotation in learning objects
  static int readRotateNum( const char* filename = "param/parameters.txt" );

  // read flag for c3_hlac extraction
  static int readC3HLACFlag( const char* filename = "param/parameters.txt" );

  // read the threshold for RGB binalize
  static void readColorThreshold( int &r, int &g, int &b, const char* filename = "param/color_threshold.txt" );

  // // read parameter of SR confidence for auto mesh construction
  // static int readConfTh( const char* filename );

  // // read parameters for auto mesh construction
  // static void readParamAuto( float& length_max_rate, float& length_th, float& distance_th, int& confidence_th, bool &relative_mode, const char* filename );

  // // write parameters for auto mesh construction
  // static void writeParamAuto( float length_max_rate, float length_th, float distance_th, int confidence_th, bool relative_mode, const char* filename );

private:
  static bool readParam( const char* filename, const char *param_string, int &val );
  static bool readParam( const char* filename, const char *param_string, float &val );

  // no constructor
  Param();
};

#endif
