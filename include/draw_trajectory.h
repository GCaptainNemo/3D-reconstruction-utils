#pragma once
#include <vector>
#include <Eigen/Eigen>
#include <pcl/io/ply_io.h>
namespace visualize{
	// viusalize trajectory
	void draw_trajector(const std::vector <Eigen::Isometry3d> &poses, const char * option);

	// read trajectory file 
	void read_trajectory_file(const std::string & file_address);

	void visualize_rgbxyz(const std::string & pc_address);
}