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

//***************************************************************************************//
//* sliding-box object detection and show the most similar regions to the target object *//
//***************************************************************************************//

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <float.h>
#include <color_voxel_recognition/pca.h>
#include <color_voxel_recognition/param.h>
#include <color_voxel_recognition/search_c3_hlac.h>
#include <c3_hlac/c3_hlac_tools.h>
#include <color_voxel_recognition/FILE_MODE>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

SearchC3HLAC search_obj;
int color_threshold_r, color_threshold_g, color_threshold_b;
int dim;
int box_size;
float voxel_size;
float region_size;
float sliding_box_size_x, sliding_box_size_y, sliding_box_size_z;
bool exit_flg = false;
bool start_flg = true;
float distance_th;
float detect_th = 0;
int rank_num;

//******************************************
//* limit points by certain depth threshold
template <typename T>
int limitPoint( const pcl::PointCloud<T> input_cloud, pcl::PointCloud<T> &output_cloud, const float dis_th ){
  output_cloud.width = input_cloud.width;
  output_cloud.height = input_cloud.height;
  const int v_num = input_cloud.points.size();
  output_cloud.points.resize( v_num );

  int idx = 0;
  for( int i=0; i<v_num; i++ ){
    if( input_cloud.points[ i ].z < dis_th ){
      output_cloud.points[ idx ] = input_cloud.points[ i ];
      idx++;
    }
  }
  output_cloud.width = idx;
  output_cloud.height = 1;
  output_cloud.points.resize( idx );
  std::cout << "from " << input_cloud.points.size() << " to " << idx << " points" << std::endl;
  return idx;
}

//**************
//* set marker
visualization_msgs::Marker setMarker( const int id_, const float pos_x, const float pos_y, const float pos_z, const float scale_x, const float scale_y, const float scale_z, const float ca, const float cr, const float cg, const float cb ){
  visualization_msgs::Marker marker_;
  marker_.header.frame_id = "/openni_rgb_optical_frame";
  //std::cout << frame_id << std::endl;
  marker_.header.stamp = ros::Time::now();
  marker_.ns = "BoxEstimation";
  marker_.id = id_;
  marker_.type = visualization_msgs::Marker::CUBE;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = pos_x;
  marker_.pose.position.y = pos_y;
  marker_.pose.position.z = pos_z;
  marker_.pose.orientation.x = 0;
  marker_.pose.orientation.y = 0;
  marker_.pose.orientation.z = 0;
  marker_.pose.orientation.w = 1;
  marker_.scale.x = scale_x;
  marker_.scale.y = scale_y;
  marker_.scale.z = scale_z;
  marker_.color.a = ca;
  marker_.color.r = cr;
  marker_.color.g = cg;
  marker_.color.b = cb;
  marker_.lifetime = ros::Duration();
  return marker_;
}

//**********************************************************
//* voxelize, extract features and detect the target object
class VoxelizeAndDetect {
protected:
  ros::NodeHandle nh_;
private:
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb_, cloud_xyzrgb;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled;
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  double t1, t1_2, t2, t0, t0_2, tAll;
  int process_count;
public:
  std::string cloud_topic_;
  ros::Subscriber sub_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;

  VoxelizeAndDetect() :
    tAll(0), process_count(0) {
  }

  void vad_cb(const sensor_msgs::PointCloud2ConstPtr& cloud) {
      if ((cloud->width * cloud->height) == 0)
        return;
      pcl::fromROSMsg (*cloud, cloud_xyzrgb_);
      t0 = my_clock();

      if( limitPoint( cloud_xyzrgb_, cloud_xyzrgb, distance_th ) > 10 ){
	std::cout << "voxelize...." << std::endl;
	getVoxelGrid( grid, cloud_xyzrgb, cloud_downsampled, voxel_size );
	std::cout << "     ...done.." << std::endl;
	
	const int pnum = cloud_downsampled.points.size();
	float x_min = 10000000, y_min = 10000000, z_min = 10000000;
	float x_max = -10000000, y_max = -10000000, z_max = -10000000;
	for( int p=0; p<pnum; p++ ){
	  if( cloud_downsampled.points[ p ].x < x_min ) x_min = cloud_downsampled.points[ p ].x;
	  if( cloud_downsampled.points[ p ].y < y_min ) y_min = cloud_downsampled.points[ p ].y;
	  if( cloud_downsampled.points[ p ].z < z_min ) z_min = cloud_downsampled.points[ p ].z;
	  if( cloud_downsampled.points[ p ].x > x_max ) x_max = cloud_downsampled.points[ p ].x;
	  if( cloud_downsampled.points[ p ].y > y_max ) y_max = cloud_downsampled.points[ p ].y;
	  if( cloud_downsampled.points[ p ].z > z_max ) z_max = cloud_downsampled.points[ p ].z;
	}
	//std::cout << x_min << " " << y_min << " " << z_min << std::endl;
	//std::cout << x_max << " " << y_max << " " << z_max << std::endl;
	//std::cout << grid.getMinBoxCoordinates() << std::endl;

	std::cout << "search start..." << std::endl;
	//****************************************
	//* object detection start
	t1 = my_clock();
	search_obj.cleanData();
	search_obj.setC3HLAC( dim, color_threshold_r, color_threshold_g, color_threshold_b, grid, cloud_downsampled, voxel_size, box_size );
	t1_2 = my_clock();
	if( ( search_obj.XYnum() != 0 ) && ( search_obj.Znum() != 0 ) )
	  search_obj.search();	
	t2 = my_clock();
	//* object detection end
	//****************************************
	std::cout << "  ...search done." << std::endl;

	//* show the processing time	
	tAll += t2 - t0;
	process_count++;
	std::cout << "voxelize           :"<< t1 - t0 << " sec" << std::endl;
	std::cout << "feature extraction : "<< t1_2 - t1 << " sec" <<std::endl;
	std::cout << "search             : "<< t2 - t1_2 << " sec" <<std::endl;
	std::cout << "all processes      : "<< t2 - t0 << " sec" << std::endl;
	std::cout << "AVERAGE            : "<< tAll / process_count << " sec" << std::endl;
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker_range", 1); 
	marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
	visualization_msgs::MarkerArray marker_array_msg_;
	
	//* show the limited space
	//setMarker( id_, pos_x, pos_y, pos_z, scale_x, scale_y, scale_z, ca, cr, cg, cb )
	marker_pub_.publish( setMarker( -1, (x_max+x_min)/2, (y_max+y_min)/2, (z_max+z_min)/2, x_max-x_min, y_max-y_min, z_max-z_min, 0.1, 1.0, 0.0, 0.0 ) );
	
	//* publish markers for detected regions
	for( int q=0; q<rank_num; q++ ){
	  if( search_obj.maxDot( q ) < detect_th )
	    marker_array_msg_.markers.push_back( setMarker( q, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ) );
	  else{
	    std::cout << search_obj.maxX( q ) << " " << search_obj.maxY( q ) << " " << search_obj.maxZ( q ) << std::endl;
	    std::cout << "dot " << search_obj.maxDot( q ) << std::endl;
	    marker_array_msg_.markers.push_back( setMarker( q, search_obj.maxX( q ) * region_size + sliding_box_size_x/2 + x_min, search_obj.maxY( q ) * region_size + sliding_box_size_y/2 + y_min, search_obj.maxZ( q ) * region_size + sliding_box_size_z/2 + z_min, sliding_box_size_x, sliding_box_size_y, sliding_box_size_z, 0.5, 0.0, 1.0, 0.0 ) );
	  }
	}
	//std::cerr << "MARKER ARRAY published with size: " << marker_array_msg_.markers.size() << std::endl; 
	marker_array_pub_.publish(marker_array_msg_);
      }
      std::cout << "Waiting msg..." << std::endl;
  }
  
  void loop(){
      cloud_topic_ = "input";
      sub_ = nh_.subscribe ("input", 1,  &VoxelizeAndDetect::vad_cb, this);
      ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (cloud_topic_).c_str ());
  }
};

//********************************
//* main
int main(int argc, char* argv[]) {
  if( (argc != 12) && (argc != 14) ){
    std::cerr << "usage: " << argv[0] << " [path] <rank_num> <exist_voxel_num_threshold> [model_pca_filename] <dim_model> <size1> <size2> <size3> <detect_th> <distance_th> /input:=/camera/rgb/points" << std::endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 1000 ];
  ros::init (argc, argv, "detectObj", ros::init_options::AnonymousName);

  // read the length of voxel side
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  voxel_size = Param::readVoxelSize( tmpname );

  detect_th = atof( argv[9] );
  distance_th = atof( argv[10] );
  rank_num = atoi( argv[2] );

  // read the number of voxels in each subdivision's side of scene
  box_size = Param::readBoxSizeScene( tmpname );

  // read the dimension of compressed feature vectors
  dim = Param::readDim( tmpname );

  // set the dimension of the target object's subspace
  const int dim_model = atoi(argv[5]);
  if( dim <= dim_model ){
    std::cerr << "ERR: dim_model should be less than dim(in dim.txt)" << std::endl;
    exit( EXIT_FAILURE );
  }

  // read the threshold for RGB binalize
  sprintf( tmpname, "%s/param/color_threshold.txt", argv[1] );
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b, tmpname );

  // determine the size of sliding box
  region_size = box_size * voxel_size;
  float tmp_val = atof(argv[6]) / region_size;
  int size1 = (int)tmp_val;
  if( ( ( tmp_val - size1 ) >= 0.5 ) || ( size1 == 0 ) ) size1++;
  tmp_val = atof(argv[7]) / region_size;
  int size2 = (int)tmp_val;
  if( ( ( tmp_val - size2 ) >= 0.5 ) || ( size2 == 0 ) ) size2++;
  tmp_val = atof(argv[8]) / region_size;
  int size3 = (int)tmp_val;
  if( ( ( tmp_val - size3 ) >= 0.5 ) || ( size3 == 0 ) ) size3++;
  sliding_box_size_x = size1 * region_size;
  sliding_box_size_y = size2 * region_size;
  sliding_box_size_z = size3 * region_size;

  // set variables
  search_obj.setRange( size1, size2, size3 );
  search_obj.setRank( rank_num );
  search_obj.setThreshold( atoi(argv[3]) );
  search_obj.readAxis( argv[4], dim, dim_model, ASCII_MODE_P, MULTIPLE_SIMILARITY );

  // read projection axis of the target object's subspace
  PCA pca;
  sprintf( tmpname, "%s/models/compress_axis", argv[1] );
  pca.read( tmpname, ASCII_MODE_P );
  Eigen::MatrixXf tmpaxis = pca.getAxis();
  Eigen::MatrixXf axis = tmpaxis.block( 0,0,tmpaxis.rows(),dim );
  Eigen::MatrixXf axis_t = axis.transpose();
  Eigen::VectorXf variance = pca.getVariance();
  if( WHITENING )
    search_obj.setSceneAxis( axis_t, variance, dim );
  else
    search_obj.setSceneAxis( axis_t );

  // object detection
  VoxelizeAndDetect vad;
  vad.loop();
  ros::spin();

  return 0;
}
