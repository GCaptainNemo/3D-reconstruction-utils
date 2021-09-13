#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


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
	Eigen::Isometry3d from_rgb_imgs(const std::string & last_img_address, const std::string & nxt_img_address, const cv::Mat & intrinsinc_mat);

	void find_feature_match(const cv::Mat & last_img, const cv::Mat & next_img,
		std::vector<cv::Point2f> & last_key_pts, std::vector<cv::Point2f> & next_key_pts, 
		std::vector<cv::DMatch> & matches, const cv::Mat & intrinsic_mat, cv::Mat & essential_mat);

	void pose_estimate_2d2d(const std::vector<cv::Point2f> & last_key_pts, const std::vector<cv::Point2f> & next_key_pts, 
		const std::vector<cv::DMatch> & matches, cv::Mat & R, cv::Mat & t, const cv::Mat & intrinsic_mat, 
		const cv::Mat & essential_mat);

}

