#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



// front-end 
namespace VO 
{
	// use IMU groud truth
	void from_imu(const std::vector <double>& extrinsic_timestamp_vec,
		const std::vector<std::vector<double>> &extrinsic_vec, const double & depth_time_stamp,
		float * translate_vec, float * rotate_mat);

	Eigen::Isometry3d from_imu(const std::vector <double>& extrinsic_timestamp_vec,
		const std::vector<std::vector<double>> &extrinsic_vec, const double & depth_time_stamp);

	// ORB-SLAM 2d-2d methods
	Eigen::Isometry3d from_rgb_imgs(const std::string & last_img_address, 
		const std::string & nxt_img_address, const cv::Mat & intrinsinc_mat);

	void feature_match(const cv::Mat & last_img, const cv::Mat & next_img,
		std::vector<cv::Point2f> & last_2f, std::vector<cv::Point2f> & next_2f,
		const cv::Mat & intrinsic_mat, cv::Mat & essential_mat, const bool is_draw);

	void pose_estimate_2d2d(const std::vector<cv::Point2f> & last_key_pts, 
		const std::vector<cv::Point2f> & next_key_pts, 
		cv::Mat & R, cv::Mat & t, const cv::Mat & intrinsic_mat, 
		const cv::Mat & essential_mat);

	void verify_epipolar_constraint(const cv::Mat & essential_mat, const cv::Mat & intrinsic_mat,
		const std::vector<cv::Point2f> & last_pts, const std::vector<cv::Point2f> & next_pts);

	// ICP 3d-3d methods
	Eigen::Isometry3d icp_depth_imgs(const std::string & last_dimg_address,
		const std::string & nxt_dimg_address, const cv::Mat & intrinsinc_mat);

	void depth_img2pc_xyz(const std::string & depth_img_address, const cv::Mat & intrinsinc_mat,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

}

