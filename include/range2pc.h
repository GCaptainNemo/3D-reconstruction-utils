#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void vec2float(std::vector<double> extrinsic_ve, float * translate_vec,
	float * quaternion);


void launch_traversal_range_img(
	const std::vector<std::string> & depth_address_vec, const std::vector<double> & depth_timestamp_vec,
	const std::vector<std::string> & rgb_address_vec, const std::vector<double> & rgb_timestamp_vec,
	const std::vector<std::vector<double>> &extrinsic_vec, const std::vector<double> &extrinsic_timestamp_vec);

void read_trajector_rgb_depth(const std::string & file_dir);

void depth_img2pc(const std::string & depth_img_address, const std::string & rgb_img_address,
	const float & fx, const float & fy, const float & cx, const float & cy, float * rotate_mat,
	float * translate_vec, const bool is_save, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

template<class T>
void quaternion2matrix(T * quaternion, T * rotate_mat);

void read_img_file(const std::string &address, std::vector<std::string> & file_address_vec,
	std::vector<double> &timestamp_vec, const std::string & file_dir);

void read_trajectory_file(const std::string &file_name, std::vector<std::vector<double>> & extrinsic_vec,
	std::vector<double> &extrinsic_timestamp_vec);

std::vector<std::string> split(const std::string &str, const std::string &pattern);


void interpolate(std::vector<double> & min_extrinsic, std::vector<double> & max_extrinsic, 
	const double & cur_time, const double & last_time, const double & next_time, float * quaternion, float * translate_vec);

