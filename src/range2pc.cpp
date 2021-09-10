#include "../include/range2pc.h"
#include "opencv2/opencv.hpp"
#include <math.h>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "opencv2/core.hpp"
#include <fstream>
#include <iostream>



template<class T>
void quaternion2matrix(T * quaternion, T * rotate_mat)
{
	// quaternion: qx, qy, qz, qw  --> rotate matrix
	const T w = quaternion[3];
	const T x = quaternion[0];
	const T y = quaternion[1];
	const T z = quaternion[2];
	rotate_mat[0] = 1 - 2 * pow(y, 2) - 2 * pow(z, 2);
	rotate_mat[1] = 2 * y * x - 2 * w * z;
	rotate_mat[2] = 2 * z * x + 2 * w * y;
	// 
	rotate_mat[3] = 2 * y * x + 2 * w * z;
	rotate_mat[4] = 1 - 2 * pow(x, 2) - 2 * pow(z, 2);
	rotate_mat[5] = 2 * y * z - 2 * w * x;
	// 
	rotate_mat[6] = 2 * z * x - 2 * w * y;
	rotate_mat[7] = 2 * y * z + 2 * w * x;
	rotate_mat[8] = 1 - 2 * pow(y, 2) - 2 * pow(x, 2);
};

void vec2float(std::vector<double> extrinsic_ve, float * translate_vec, float * quaternion)
{
	// tx, ty, tz
	for (int i = 0; i < 3; ++i)
	{
		translate_vec[i] = (float)extrinsic_ve[i];
	}
	// qx, qy, qz, w
	for (int j = 0; j < 4; ++j)
	{
		quaternion[j] = (float)extrinsic_ve[3 + j];
	}
};

void interpolate(std::vector<double> & last_extrinsic, std::vector<double> & next_extrinsic,
	const double & cur_time, const double & last_time, const double & next_time, float * quaternion, float * translate_vec) 
{
	float last_translate_vec[3];
	float last_quaternion[4];
	vec2float(last_extrinsic, last_translate_vec, last_quaternion);
	float nxt_translate_vec[3];
	float nxt_quaternion[4];
	vec2float(next_extrinsic, nxt_translate_vec, nxt_quaternion);
	printf("last_tranlate_ = ");
	for (int i = 0; i < 3; ++i) 
	{
		printf("%f, ", last_translate_vec[i]);
	}
	printf("\n");
	printf("nxt_tranlate_ = ");
	for (int i = 0; i < 3; ++i)
	{
		printf("%f, ", nxt_translate_vec[i]);
	}
	printf("\n");
	
	// 
	const double translate_t = (cur_time - last_time) / (next_time - last_time);
	printf("translate_t = %f\n", translate_t);
	printf("last_time = %f\n", last_time);
	printf("next_time = %f\n", next_time);
	printf("cur_time = %f\n", cur_time);

	for (int i = 0; i < 3; ++i) {
		translate_vec[i] = translate_t * nxt_translate_vec[i] + (1.0 - translate_t) * last_translate_vec[i];
	}
	printf("merge vec = ");
	for (int i = 0; i < 3; ++i)
	{
		printf("%f, ", translate_vec[i]);
	}
	printf("\n");
	// 
	double dot = 0.0;
	for (int i = 0; i < 4; ++i) 
	{
		dot += nxt_quaternion[i] * last_quaternion[i];
	}
	if (dot <= 0) 
	{
		for (int i = 0; i < 4; ++i)
		{
			nxt_quaternion[i] *= -1.0f;
		}
		dot *= -1;
	}
	
	//const double theta = acos(dot);
	//const double proportion_1 = sin(translate_t * theta) / sin(theta);
	//const double proportion_2 = sin((1 - translate_t) * theta) / sin(theta);
	//for (int i = 0; i < 4; ++i) {
	//	quaternion[i] = proportion_1 * nxt_quaternion[i] + proportion_2 * last_quaternion[i];
	//}
	float mode = 0.0f;
	for (int i = 0; i < 4; ++i) {
		quaternion[i] = translate_t * nxt_quaternion[i] + (1.0 - translate_t) * last_quaternion[i];
		mode += pow(quaternion[i], 2);
	}
	mode = sqrt(mode);
	for (int i = 0; i < 4; ++i) {
		quaternion[i] /= mode;
	}

	printf("last quaternion = ");
	for (int i = 0; i < 4; ++i)
	{
		printf("%f, ", last_quaternion[i]);
	}
	printf("\n");

	printf("next quaternion = ");
	for (int i = 0; i < 4; ++i)
	{
		printf("%f, ", nxt_quaternion[i]);
	}
	printf("\n");
	printf("merge quaternion = ");
	for (int i = 0; i < 4; ++i)
	{
		printf("%f, ", quaternion[i]);
	}
};


void launch_traversal_range_img(
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
			double min_val_extrin = 10;
			int min_id_extrinsic = 0;
			for (int j = 0; j < extrinsic_timestamp_vec.size(); ++j)
			{
				double delta = abs(extrinsic_timestamp_vec[j] - depth_time_stamp);
				if (delta < min_val_extrin)
				{
					min_val_extrin = delta;
					min_id_extrinsic = j;
				}
			}
			std::vector<double>  last_extrinsic;
			std::vector<double>  nxt_extrinsic;
			float translate_vec[3];
			float quaternion[4];
			double last_time;
			double nxt_time;
			if ((min_id_extrinsic == 0 && extrinsic_timestamp_vec[min_id_extrinsic] > depth_time_stamp) || 
				(min_id_extrinsic == extrinsic_timestamp_vec.size() -1 && extrinsic_timestamp_vec[min_id_extrinsic] < depth_time_stamp)) 
			{
				last_extrinsic = extrinsic_vec[min_id_extrinsic];
				vec2float(last_extrinsic, translate_vec, quaternion);
			}
			else {
				if (extrinsic_timestamp_vec[min_id_extrinsic] < depth_time_stamp) {
					last_extrinsic = extrinsic_vec[min_id_extrinsic];
					last_time = extrinsic_timestamp_vec[min_id_extrinsic];
					nxt_extrinsic = extrinsic_vec[min_id_extrinsic + 1];
					nxt_time = extrinsic_timestamp_vec[min_id_extrinsic + 1];
				}
				else {
					last_extrinsic = extrinsic_vec[min_id_extrinsic - 1];
					last_time = extrinsic_timestamp_vec[min_id_extrinsic - 1];
					nxt_extrinsic = extrinsic_vec[min_id_extrinsic];
					nxt_time = extrinsic_timestamp_vec[min_id_extrinsic];
				}
				interpolate(last_extrinsic, nxt_extrinsic, depth_time_stamp, last_time, nxt_time, quaternion, translate_vec);
			}
			printf("min_id = %d \n", min_id_extrinsic);
			
		

			float rotate_mat[9];
			quaternion2matrix(quaternion, rotate_mat);

			// ///////////////////////////////////////////////////////////////////////
			double min_val_rgb = 10;
			int min_id_rgb = 0;
			for (int j = 0; j < rgb_timestamp_vec.size(); ++j)
			{
				double delta = abs(rgb_timestamp_vec[j] - depth_time_stamp);
				if (delta < min_val_extrin)
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
			printf("min_id = %d", min_id_extrinsic);
			std::cout << "depth_address = " << depth_img_address << std::endl;
			depth_img2pc(depth_img_address, rgb_img_address,
				fx, fy, cx, cy, rotate_mat, translate_vec, false, cloud);
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

	launch_traversal_range_img(depth_address_vec, depth_timestamp_vec, rgb_address_vec, rgb_timestamp_vec,
		extrinsic_vec, extrinsic_timestamp_vec);
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


std::vector<std::string> split(const std::string &str, const std::string &pattern)
{
	std::vector<std::string> res;
	if (str == "")
		return res;
	std::string strs = str + pattern;
	size_t pos = strs.find(pattern);
	while (pos != strs.npos)
	{
		std::string temp = strs.substr(0, pos);
		res.push_back(temp);
		strs = strs.substr(pos + 1, strs.size());
		pos = strs.find(pattern);
	}
	return res;
}

