#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



// get rgb, depth, trajectory
void read_trajector_rgb_depth(const std::string & file_dir);

void vo_imu_range2pc(
	const std::vector<std::string> & depth_address_vec, const std::vector<double> & depth_timestamp_vec,
	const std::vector<std::string> & rgb_address_vec, const std::vector<double> & rgb_timestamp_vec,
	const std::vector<std::vector<double>> &extrinsic_vec, const std::vector<double> &extrinsic_timestamp_vec);

void vo_epipolar_range2pc(
	const std::vector<std::string> & depth_address_vec, const std::vector<double> & depth_timestamp_vec,
	const std::vector<std::string> & rgb_address_vec, const std::vector<double> & rgb_timestamp_vec,
	const std::vector<std::vector<double>> &extrinsic_vec, const std::vector<double> &extrinsic_timestamp_vec);


void depth_img2pc(const std::string & depth_img_address, const std::string & rgb_img_address,
	const float & fx, const float & fy, const float & cx, const float & cy, float * rotate_mat,
	float * translate_vec, const bool is_save, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void depth_img2pc(const std::string & depth_img_address, const std::string & rgb_img_address,
	const float & fx, const float & fy, const float & cx, const float & cy, 
	const Eigen::Isometry3d & euc3, const bool is_save, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// /////////////////////////////////////////////////////////////////////////////////
// read depth image, rgb img and trajectory ground truth(IMU)
// //////////////////////////////////////////////////////////////////////////////////
void read_img_file(const std::string &address, std::vector<std::string> & file_address_vec,
	std::vector<double> &timestamp_vec, const std::string & file_dir);

void read_trajectory_file(const std::string &file_name, std::vector<std::vector<double>> & extrinsic_vec,
	std::vector<double> &extrinsic_timestamp_vec);


