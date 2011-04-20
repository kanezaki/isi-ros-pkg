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

#include <iostream>
#include <stdio.h>
#include <string.h>

#include "color_voxel_recognition/param.h"

//******************************
// read the length of voxel side
float Param::readVoxelSize( const char* filename ){
  float val;
  if( readParam( filename, "voxel_size:", val ) ){
    if( val <= 0 ){
      std::cerr << "ERR (in Param::readVoxelSize): voxel_size must be larger than 0." << std::endl;
      return -1;
    }
    return val;
  }
  std::cerr << "ERR (in Param::readVoxelSize): fail" << std::endl;
  return -1;
}

//*************************************************
// read the dimension of compressed feature vectors
int Param::readDim( const char* filename ){
  int val;
  if( readParam( filename, "dim:", val ) ){
    if( val < 1 ){
      std::cerr << "ERR (in Param::readDim): dim must be larger than 0." << std::endl;
      return -1;
    }
    return val;
  }
  std::cerr << "ERR (in Param::readDim): fail" << std::endl;
  return -1;
}

//**************************************************************
// read the number of voxels in each subdivision's side of scene
int Param::readBoxSizeScene( const char* filename ){
  int val;
  if( readParam( filename, "box_size(scene):", val ) ){
    if( val < 1 ){
      std::cerr << "ERR (in Param::readBoxSizeScene): box_size must be larger than 0." << std::endl;
      return -1;
    }
    return val;
  }
  std::cerr << "ERR (in Param::readBoxSizeScene): fail" << std::endl;
  return -1;
}

//************************************************************************
// read the number of voxels in each subdivision's side of a target object
int Param::readBoxSizeModel( const char* filename ){
  int val;
  if( readParam( filename, "box_size(model):", val ) ){
    if( val < 1 ){
      std::cerr << "ERR (in Param::readBoxSizeModel): box_size must be larger than 0." << std::endl;
      return -1;
    }
    return val;
  }
  std::cerr << "ERR (in Param::readBoxSizeModel): fail" << std::endl;
  return -1;
}

// //********************************************
// // read the max number of voxels created once
// int Param::readMaxVoxelNum( const char* filename ){
//   int val;
//   if( readParam( filename, "max_voxel_num:", val ) ){
//     if( val < 1 ){
//       std::cerr << "ERR (in Param::readMaxVoxelNum): max_voxel_num must be larger than 0." << std::endl;
//       return -1;
//     }
//     else if( val < 10000000 )
//       std::cerr << "Warning (in Param::readMaxVoxelNum): max_voxel_num is too small." << std::endl;
//     return val;
//   }
//   std::cerr << "ERR (in Param::readMaxVoxelNum): fail" << std::endl;
//   return -1;
// }

//***********************************************************
// read the number for synthetic rotation in learning objects
int Param::readRotateNum( const char* filename ){
  int val;
  if( readParam( filename, "rotate_num:", val ) ){
    if( val < 1 ){
      std::cerr << "ERR (in Param::readRotateNum): rotate_num must be larger than 0." << std::endl;
      return -1;
    }
    return val;
  }
  std::cerr << "ERR (in Param::readRotateNum): fail" << std::endl;
  return -1;
}

//*********************************
// read flag for c3_hlac extraction
int Param::readC3HLACFlag( const char* filename ){
  int val;
  if( readParam( filename, "c3_hlac_flg:", val ) ) return val;
  std::cerr << "ERR (in Param::readC3HLACFlag): fail" << std::endl;
  return -1;
}

//************************************
// read the threshold for RGB binalize
void Param::readColorThreshold( int &r, int &g, int &b, const char* filename ){
  FILE *fp = fopen( filename, "r" );
  if( fscanf(fp,"%d %d %d\n", &r, &g, &b) == EOF ) std::cerr<< "fscanf err" << std::endl;
  fclose(fp);
  if( ( r < 0 ) || ( r > 254 ) || ( g < 0 ) || ( g > 254 ) || ( b < 0 ) || ( b > 254 ) )
    std::cerr << "ERR (in Param::readColorThreshold): invalid RGB value." << std::endl;
}

// //***********************************************************
// // read parameter of SR confidence for auto mesh construction
// int Param::readConfTh( const char* filename ){
//   int val;
//   if( readParam( filename, "confidence_th:", val ) ){
//     if( ( val < 0 ) || ( val > 255 ) ){
//       std::cerr << "ERR (in Param::readConfTh): conf_th must be larger than -1 and less than 256." << std::endl;
//       return -1;
//     }
//     return val;
//   }
//   std::cerr << "ERR (in Param::readConfTh): fail" << std::endl;
//   return -1;
// }

// //********************************************
// // read parameters for auto mesh construction
// void Param::readParamAuto( float& length_max_rate, float& length_th, float& distance_th, int& confidence_th, bool &relative_mode, const char* filename ){
//   char line[ 100 ];
//   FILE *fp = fopen( filename, "r" );
//   int tmp;
//   if( fscanf( fp, "%s %f\n", line, &length_max_rate ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   if( fscanf( fp, "%s %f\n", line, &length_th ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   if( fscanf( fp, "%s %f\n", line, &distance_th ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   if( fscanf( fp, "%s %d\n", line, &confidence_th ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   if( fscanf( fp, "%s %d\n", line, &tmp ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   relative_mode = tmp;
//   fclose( fp );
// }

// //********************************************
// // write parameters for auto mesh construction
// void Param::writeParamAuto( float length_max_rate, float length_th, float distance_th, int confidence_th, bool relative_mode, const char* filename ){
//   FILE *fp = fopen( filename, "w" );
//   fprintf( fp, "length_max_rate: %f\n", length_max_rate );
//   fprintf( fp, "length_th:       %f\n", length_th );
//   fprintf( fp, "distance_th:     %f\n", distance_th );
//   fprintf( fp, "confidence_th:   %d\n", confidence_th );
//   fprintf( fp, "relative_mode:   %d\n", relative_mode );
//   fclose( fp );
// }

//*********************//
//* private functions *//
//*********************//

bool Param::readParam( const char* filename, const char *param_string, int &val ){
  char line[100];
  FILE *fp = fopen( filename, "r" );
  while( fscanf(fp,"%s %d",line,&val)!=EOF ){
    if( strcmp(line,param_string) == 0 ){
      fclose(fp);
      return true;
    }
  }
  fclose(fp);
  return false;
}

bool Param::readParam( const char* filename, const char *param_string, float &val ){
  char line[100];
  FILE *fp = fopen( filename, "r" );
  while( fscanf(fp,"%s %f",line,&val)!=EOF ){
    if( strcmp(line,param_string) == 0 ){
      fclose(fp);
      return true;
    }
  }
  fclose(fp);
  return false;
}
