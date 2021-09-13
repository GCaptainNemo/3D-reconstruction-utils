#include "../include/visual_odometry.h"
#include "../include/utils.h"


// VO: use IMU groud truth
namespace VO
{
	void from_imu(const std::vector <double>& extrinsic_timestamp_vec,
		const std::vector<std::vector<double>> &extrinsic_vec, const double & depth_time_stamp,
		float * translate_vec, float * rotate_mat)
	{
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
		float quaternion[4];
		double last_time;
		double nxt_time;
		if ((min_id_extrinsic == 0 && extrinsic_timestamp_vec[min_id_extrinsic] > depth_time_stamp) ||
			(min_id_extrinsic == extrinsic_timestamp_vec.size() - 1 && extrinsic_timestamp_vec[min_id_extrinsic] < depth_time_stamp))
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
		quaternion2matrix(quaternion, rotate_mat);
	};

	Eigen::Isometry3d from_imu(const std::vector <double>& extrinsic_timestamp_vec,
		const std::vector<std::vector<double>> &extrinsic_vec, const double & depth_time_stamp
	)
	{
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
		float quaternion[4];
		float translate_vec[3];
		double last_time;
		double nxt_time;
		if ((min_id_extrinsic == 0 && extrinsic_timestamp_vec[min_id_extrinsic] > depth_time_stamp) ||
			(min_id_extrinsic == extrinsic_timestamp_vec.size() - 1 && extrinsic_timestamp_vec[min_id_extrinsic] < depth_time_stamp))
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
		Eigen::Isometry3d euc3(Eigen::Quaterniond(quaternion[3], quaternion[0], quaternion[1], quaternion[2]));
		euc3.pretranslate(Eigen::Vector3d(translate_vec[0], translate_vec[1], translate_vec[2]));
		return euc3;
	};

	// ORB-SLAM 2D-2D orb descriptors match methods
	Eigen::Isometry3d from_rgb_imgs(const std::string & last_img_address, const std::string & nxt_img_address, const cv::Mat & intrinsinc_mat)
	{
		cv::Mat last_img = cv::imread(last_img_address, cv::IMREAD_GRAYSCALE);
		cv::Mat next_img = cv::imread(nxt_img_address, cv::IMREAD_GRAYSCALE);
		std::vector<cv::Point2f> last_match_pts(0);
		std::vector<cv::Point2f> next_match_pts(0);
		std::vector<cv::DMatch> matches;

		// extract correspond points
		cv::Mat essential_mat;
		VO::find_feature_match(last_img, next_img, last_match_pts, next_match_pts, matches, intrinsinc_mat, essential_mat);
		

		// estimate extrinsic parameters
		cv::Mat R;
		cv::Mat t;
		
		
		VO::pose_estimate_2d2d(last_match_pts, next_match_pts, matches, R, t, intrinsinc_mat, essential_mat);
		Eigen::Isometry3d res = Eigen::Isometry3d::Identity();
		Eigen::Vector3d translate_vec;
		std::cout << t.cols << "  " << t.rows << std::endl;
		translate_vec << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0);
		Eigen::Matrix3d rotation_mat = Eigen::Matrix3d::Identity();
		rotation_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
						R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
						R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
		std::cout << "rotation_mat  = " << rotation_mat << std::endl;
		std::cout << "translate_vec = " << translate_vec << std::endl;

		res.rotate(rotation_mat);
		res.pretranslate(translate_vec);
		return res;

	};

	void find_feature_match(const cv::Mat & last_img, const cv::Mat & next_img,
		std::vector<cv::Point2f> & last_key_2f, std::vector<cv::Point2f> & next_key_2f, 
		std::vector<cv::DMatch> & matches, const cv::Mat & intrinsic_mat, cv::Mat & essential_mat)
	{
		cv::Ptr<cv::ORB> orb_last_extractor = cv::ORB::create();
		cv::Ptr<cv::ORB> orb_right_extractor = cv::ORB::create();
		std::vector<cv::KeyPoint> last_key_pts;
		std::vector<cv::KeyPoint> next_key_pts;
		cv::Mat last_des;				// 用于保存图中的特征点的特征描述
		cv::Mat next_des;
		orb_last_extractor->detectAndCompute(last_img, cv::noArray(), last_key_pts, last_des);
		orb_right_extractor->detectAndCompute(next_img, cv::noArray(), next_key_pts, next_des);
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
		matcher->match(last_des, next_des, matches);
		for (auto& each : matches)
		{
			last_key_2f.push_back(last_key_pts[each.queryIdx].pt);
			next_key_2f.push_back(next_key_pts[each.trainIdx].pt);
		}
		std::vector<unsigned char> vTemp(last_key_2f.size());
		essential_mat = cv::findEssentialMat(last_key_2f, next_key_2f, 
			intrinsic_mat, cv::RANSAC, 0.999, 0.5, vTemp);
		std::cout << "essential_mat = " << essential_mat << std::endl;

		std::vector<cv::DMatch> optimizeM;
		for (int i = 0; i < vTemp.size(); i++)
		{
			if (vTemp[i])
			{
				optimizeM.push_back(matches[i]);
			}
		}
		matches.swap(optimizeM);
		std::cout << essential_mat << std::endl;
		cv::Mat optimizeP;
		cv::drawMatches(last_img, last_key_pts, next_img, next_key_pts, matches, optimizeP);
		cv::namedWindow("matches", cv::WINDOW_NORMAL);
		cv::imshow("matches", optimizeP);
		cv::waitKey(0);

	};


	void pose_estimate_2d2d(const std::vector<cv::Point2f> & last_match_pts, const std::vector<cv::Point2f> & next_match_pts,
		const std::vector<cv::DMatch> & matches, cv::Mat & R, cv::Mat & t, const cv::Mat & intrinsic_mat, const cv::Mat & essential_mat) 
	{
		// use epipolar constraint to recover camera poses
		cv::recoverPose(essential_mat, last_match_pts, next_match_pts, intrinsic_mat, R, t);
		std::cout << "R = " << R << std::endl;
		std::cout << "t = " << t << std::endl;

		// calculate homography matrix to prevent degenracy
		cv::Mat homography_mat = cv::findHomography(last_match_pts, next_match_pts, cv::RANSAC, 3);
		std::cout << homography_mat << std::endl;

		
		
	};

}