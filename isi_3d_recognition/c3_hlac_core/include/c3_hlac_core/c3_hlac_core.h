/*
 * Software License Agreement (BSD License for non-commercial use)
 *
 *  Copyright (c) 2011, Asako Kanezaki <kanezaki@isi.imi.i.u-tokyo.ac.jp>
 *  All rights reserved.
 *
 * NOTE: This license is for non-commercial purpose use only.
 * For commercial purpose use, it is required to obtain royalty-bearing
 * license from the Owner. Commercial use of this software without such
 * royalty-bearing license is prohibited.
 *
 * To the extent of non-commercial purpose use, redistribution and use
 * in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
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

#ifndef C3_HLAC_CORE_H_
#define C3_HLAC_CORE_H_

namespace c3_hlac{
  void setColor( int &r, int &g, int &b, int &r_, int &g_, int &b_ );
  void addC3HLACcol0117 ( float* histogram[117], const int center_r, const int center_r_, const int center_g, const int center_g_, const int center_b, const int center_b_ );
  void addC3HLACcol0Bin117 ( float* histogram[117], const int center_bin_r, const int center_bin_g, const int center_bin_b );
  void addC3HLACcol1117 ( float* histogram[117], const int center_r, const int center_r_, const int center_g, const int center_g_, const int center_b, const int center_b_, const int r, const int g, const int b, const int r_, const int g_, const int b_ );
  void addC3HLACcol1Bin117 ( float* histogram[117], const int center_bin_r, const int center_bin_g, const int center_bin_b, const int r, const int g, const int b );

  void addC3HLACcol0981 ( float* histogram[981], const int center_r, const int center_r_, const int center_g, const int center_g_, const int center_b, const int center_b_ );
  void addC3HLACcol0Bin981 ( float* histogram[981], const int center_bin_r, const int center_bin_g, const int center_bin_b );
  void addC3HLACcol1981 ( float* histogram[981], const int neighbor_idx, const int center_r, const int center_r_, const int center_g, const int center_g_, const int center_b, const int center_b_, const int r, const int g, const int b, const int r_, const int g_, const int b_ );
  void addC3HLACcol1Bin981 ( float* histogram[981], const int neighbor_idx, const int center_bin_r, const int center_bin_g, const int center_bin_b, const int r, const int g, const int b );
}

#endif
