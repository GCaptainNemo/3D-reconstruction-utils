#include "../include/range2pc.h"
#include "../include/utils.h"
#include "../include/visual_odometry.h"
#include <math.h>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Eigen>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include <fstream>
#include <iostream>




void vo_imu_range2pc(
	const std::vector<std::string> & depth_address_vec, const std::vector<double> & depth_timestamp_vec,
	const std::vector<std::string> & rgb_address_vec, const std::vector<double> & rgb_timestamp_vec,
	const std::vector<std::vector<double>> &extrinsic_vec, const std::vector<double> &extrinsic_timestamp_vec)
{
	const bool save = true;
	const float fx = 591.1;
	const float fy = 590.1;
	const float cx = 331.0;
	const float cy = 234.0;
	const int down_sample_factor = 50;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < depth_address_vec.size(); ++i)
	{
		if (i % down_sample_factor == 0) {
			// ////////////////////////////////////////////////////////////////////////////////////
			// 1. get depth image and correspond timestamp, quaternion, translate vec, rgb_img
			// ////////////////////////////////////////////////////////////////////////////////////
			const double depth_time_stamp = depth_timestamp_vec[i];
			const std::string depth_img_address = depth_address_vec[i];
			Eigen::Isometry3d euc3 =  VO::from_imu(extrinsic_timestamp_vec, extrinsic_vec, depth_time_stamp);
			
			// ///////////////////////////////////////////////////////////////////////
			double min_val_rgb = 10;
			int min_id_rgb = 0;
			for (int j = 0; j < rgb_timestamp_vec.size(); ++j)
			{
				double delta = abs(rgb_timestamp_vec[j] - depth_time_stamp);
				if (delta < min_val_rgb)
				{
					min_val_rgb = delta;
					min_id_rgb = j;
				}
			}
			printf("min_id_rgb = %d \n", min_id_rgb);
			const std::string rgb_img_address = rgb_address_vec[min_id_rgb];

			// ////////////////////////////////////////////////////////////////////
			// 2. read rgb_img and depth_img
			// ////////////////////////////////////////////////////////////////////
			std::cout << "depth_address = " << depth_img_address << std::endl;
			depth_img2pc(depth_img_address, rgb_img_address,
				fx, fy, cx, cy, euc3, false, cloud);
			printf("total_frame_xyzrgb.size() = %d\n", cloud->size());
		}
	}
	//save_ply_file("down_rgb_pc.ply", total_frame_xyzrgb);
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
	if (save) { pcl::io::savePCDFile("color_pc_data2.pcd", *cloud); }

};


void vo_epipolar_range2pc(
	const std::vector<std::string> & depth_address_vec, const std::vector<double> & depth_timestamp_vec,
	const std::vector<std::string> & rgb_address_vec, const std::vector<double> & rgb_timestamp_vec,
	const std::vector<std::vector<double>> &extrinsic_vec, const std::vector<double> &extrinsic_timestamp_vec) 
{
	const bool save = true;
	const float fx = 591.1;
	const float fy = 590.1;
	const float cx = 331.0;
	const float cy = 234.0;
	cv::Mat intrinsinc_mat = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0.0, 0.0, 1.0);

	const int down_sample_factor = 100;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Isometry3d cur_pose = Eigen::Isometry3d::Identity();
	int last_rgb_img_id;
	for (int i = 0; i < depth_address_vec.size(); ++i)
	{
		if (i % down_sample_factor == 0) {
			// ////////////////////////////////////////////////////////////////////////////////////
			// 1. get depth image 
			// ////////////////////////////////////////////////////////////////////////////////////
			const double depth_time_stamp = depth_timestamp_vec[i];
			const std::string depth_img_address = depth_address_vec[i];

			// ///////////////////////////////////////////////////////////////////////
			int nxt_rgb_id;
			bool has_bigger = false;
			for (int j = 0; j < rgb_timestamp_vec.size(); ++j)
			{
				if (rgb_timestamp_vec[j] > depth_time_stamp)
				{
					nxt_rgb_id = j;
					has_bigger = true;
					break;
				}
			}
			if (!has_bigger)
				break;
			const std::string nxt_rgbimg_address = rgb_address_vec[nxt_rgb_id];
			Eigen::Isometry3d relative_pose;
			if (i != 0) {
				const std::string last_img_address = rgb_address_vec[last_rgb_img_id];
				relative_pose = VO::from_rgb_imgs(last_img_address, nxt_rgbimg_address, intrinsinc_mat);
			}
			else 
			{
				relative_pose = Eigen::Isometry3d::Identity();
			}
			cur_pose = relative_pose * cur_pose;
			// ////////////////////////////////////////////////////////////////////
			// 2. read rgb_img and depth_img
			// ////////////////////////////////////////////////////////////////////
			std::cout << "depth_address = " << depth_img_address << std::endl;
			depth_img2pc(depth_img_address, nxt_rgbimg_address,
				fx, fy, cx, cy, cur_pose, false, cloud);
			printf("total_frame_xyzrgb.size() = %d\n", cloud->size());
			last_rgb_img_id = nxt_rgb_id;
		}
	}
	//save_ply_file("down_rgb_pc.ply", total_frame_xyzrgb);
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
	if (save) { pcl::io::savePCDFile("color_pc_data2.pcd", *cloud); }



};


void read_trajector_rgb_depth(const std::string & file_dir)
{
	
	const std::string trajector_path = file_dir + "/groundtruth.txt";
	const std::string rgb_path = file_dir + "/rgb.txt";
	const std::string depth_path = file_dir + "/depth.txt";
	std::cout << "traject = " << trajector_path << std::endl;
	// time stamp
	std::vector<double> extrinsic_timestamp_vec;
	std::vector<double> rgb_timestamp_vec;
	std::vector<double> depth_timestamp_vec;

	// address(content)
	std::vector<std::vector<double>> extrinsic_vec;
	std::vector<std::string> rgb_address_vec;
	std::vector<std::string> depth_address_vec;


	read_trajectory_file(trajector_path, extrinsic_vec, extrinsic_timestamp_vec);
	read_img_file(rgb_path, rgb_address_vec, rgb_timestamp_vec, file_dir);
	read_img_file(depth_path, depth_address_vec, depth_timestamp_vec, file_dir);

	printf("rgb_address_vec.size () = %d\n", rgb_address_vec.size());
	printf("depth_address_vec.size () = %d\n", depth_address_vec.size());
	printf("extrinsic_vec.size () = %d\n", extrinsic_vec.size());
	// camera intrinsic and depth image to unit(m) factor
	const float fx = 591.1f;
	const float fy = 590.1f;
	const float cx = 331.0f;
	const float cy = 234.0f;
	const float factor = 5000.0f;
	vo_epipolar_range2pc(depth_address_vec, depth_timestamp_vec, rgb_address_vec, rgb_timestamp_vec,
		extrinsic_vec, extrinsic_timestamp_vec);
	/*vo_imu_range2pc(depth_address_vec, depth_timestamp_vec, rgb_address_vec, rgb_timestamp_vec,
		extrinsic_vec, extrinsic_timestamp_vec);*/
};


void depth_img2pc(const std::string & depth_img_address, const std::string & rgb_img_address,
	const float & fx, const float & fy, const float & cx, const float & cy, float * rotate_mat,
	float * translate_vec, const bool is_save, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	cv::Mat depth_img = cv::imread(depth_img_address, -1);
	cv::Mat rgb_img = cv::imread(rgb_img_address, -1);
	const float factor = 5000.0f;
	const int img_col = depth_img.cols;
	const int img_row = depth_img.rows;
	
	for (int row = 0; row < img_row; ++row)
	{
		for (int col = 0; col < img_col; ++col)
		{
			pcl::PointXYZRGB rgb_p;
			const int depth = depth_img.at<uint16_t>(row, col);
			//printf("depth = %d\n", depth);
			if (depth == 0)
				continue;
			// camera coordinate
			const float Z = (float)(depth) / 5000;
			const float X = ((float)(col)-cx) / fx * Z;
			const float Y = ((float)(row)-cy) / fy * Z;
			// global coordinate(m)
			rgb_p.x = (rotate_mat[0 * 3 + 0] * X + rotate_mat[0 * 3 + 1] * Y + rotate_mat[0 * 3 + 2] * Z + translate_vec[0]) * 1.0f;
			rgb_p.y = (rotate_mat[1 * 3 + 0] * X + rotate_mat[1 * 3 + 1] * Y + rotate_mat[1 * 3 + 2] * Z + translate_vec[1])* 1.0f;
			rgb_p.z = (rotate_mat[2 * 3 + 0] * X + rotate_mat[2 * 3 + 1] * Y + rotate_mat[2 * 3 + 2] * Z + translate_vec[2])* 1.0f;
			rgb_p.r = rgb_img.at<cv::Vec3b>(row, col)[2];
			rgb_p.g = rgb_img.at<cv::Vec3b>(row, col)[1];
			rgb_p.b = rgb_img.at<cv::Vec3b>(row, col)[0];
			cloud->push_back(rgb_p);
		}
	}
};


void depth_img2pc(const std::string & depth_img_address, const std::string & rgb_img_address,
	const float & fx, const float & fy, const float & cx, const float & cy,
	const Eigen::Isometry3d & euc3, const bool is_save, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) 
{
	cv::Mat depth_img = cv::imread(depth_img_address, -1);
	cv::Mat rgb_img = cv::imread(rgb_img_address, -1);
	const float factor = 5000.0f;
	const int img_col = depth_img.cols;
	const int img_row = depth_img.rows;

	for (int row = 0; row < img_row; ++row)
	{
		for (int col = 0; col < img_col; ++col)
		{
			pcl::PointXYZRGB rgb_p;
			const int depth = depth_img.at<uint16_t>(row, col);
			//printf("depth = %d\n", depth);
			if (depth == 0)
				continue;
			// camera coordinate
			const float Z = (float)(depth) / 5000;
			const Eigen::Vector3d dst = euc3 * Eigen::Vector3d(((float)(col)-cx) / fx * Z, 
															   ((float)(row)-cy) / fy * Z, 
																Z);
			// global coordinate(m)
			rgb_p.x = dst[0] * 1.0f;
			rgb_p.y = dst[1] * 1.0f;
			rgb_p.z = dst[2] * 1.0f;
			rgb_p.r = rgb_img.at<cv::Vec3b>(row, col)[2];
			rgb_p.g = rgb_img.at<cv::Vec3b>(row, col)[1];
			rgb_p.b = rgb_img.at<cv::Vec3b>(row, col)[0];
			cloud->push_back(rgb_p);
		}
	}
};




void read_trajectory_file(const std::string &file_name, std::vector<std::vector<double>> & extrinsic_vec,
	std::vector<double> &extrinsic_timestamp_vec)
{
	std::cout << "file_name = " << file_name << std::endl;
	std::ifstream in;
	in.open(file_name);
	if (!in.is_open()) 
	{
		printf("open unsuccessfully!\n");
		return;
	}
	std::string s;
	int line_index = 0;
	std::string line;

	while (getline(in, s))//着行读取数据并存于s中，直至数据全部读取
	{
		if (line_index > 2) {
			std::vector<std::string> t_R_str = split(s, " ");
			std::vector<double> t_R(t_R_str.size() - 1);
			extrinsic_timestamp_vec.push_back(stod(t_R_str[0]));
			for (int i = 1; i < t_R_str.size(); ++i)
			{
				t_R[i - 1] = stod(t_R_str[i]);
			}
			extrinsic_vec.push_back(t_R);
		}
		line_index++;
	}
	printf("frame size in trajectory files  = %d", extrinsic_vec.size());
	
}


void read_img_file(const std::string &address, std::vector<std::string> & file_address_vec,
	std::vector<double> &timestamp_vec, const std::string & file_dir)
{
	std::ifstream in(address);
	std::string s;
	int line_index = 0;
	std::string line;

	while (std::getline(in, s))//着行读取数据并存于s中，直至数据全部读取
	{
		if (line_index > 2) {
			std::vector<std::string> t_R_str = split(s, " ");
			timestamp_vec.push_back(stod(t_R_str[0]));

			file_address_vec.push_back(file_dir + "/" + t_R_str[1]);
		}
		line_index++;
	}

};


