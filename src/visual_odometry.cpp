#include "../include/visual_odometry.h"
#include "../include/utils.h"
#include <math.h>


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

		// extract correspond points
		cv::Mat essential_mat;
		VO::find_feature_match(last_img, next_img, last_match_pts, next_match_pts, intrinsinc_mat, essential_mat, false);
		// ////////////////////////////////////////////////////////////////////////////////////////////////
		// verify epipolar constraint
		VO::verify_epipolar_constraint(intrinsinc_mat, essential_mat, last_match_pts, next_match_pts);


		// estimate extrinsic parameters
		cv::Mat R;
		cv::Mat t;
		VO::pose_estimate_2d2d(last_match_pts, next_match_pts, R, t, intrinsinc_mat, essential_mat);

		
		// to Isometry
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
		std::vector<cv::Point2f> & last_2f, std::vector<cv::Point2f> & next_2f,
		const cv::Mat & intrinsic_mat, cv::Mat & essential_mat, const bool is_draw)
	{
		// initialize
		std::vector<cv::KeyPoint>last_key_pts, next_key_pts;
		cv::Mat descriptors_last, descriptors_next;
		cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
		cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

		// 1. calculate Oriented Fast corner point
		detector->detect(last_img, last_key_pts);
		detector->detect(next_img, next_key_pts);

		// 2. calculate BRIEF descriptor according to corner points
		descriptor->compute(last_img, last_key_pts, descriptors_last);
		descriptor->compute(next_img, next_key_pts, descriptors_next);

		// 3. Match two images BRIEF descriptors using hamming distance
		std::vector <cv::DMatch> coarse_matches;
		matcher->match(descriptors_last, descriptors_next, coarse_matches);

		// 4. filter matching pts
		// calculate minimal maximal distance
		auto min_max = minmax_element(coarse_matches.begin(), coarse_matches.end(),
			[](const cv::DMatch & m1, const cv::DMatch & m2) {return m1.distance < m2.distance; });
		double min_dist = min_max.first->distance;
		double max_dist = min_max.second->distance;
		std::cout << "min dist = " << min_dist << std::endl;
		std::cout << "max dist = " << max_dist << std::endl;

		std::vector<cv::DMatch> good_matches;
		for (int i = 0; i < descriptors_last.rows; ++i) 
		{
			if (coarse_matches[i].distance <= (2 * min_dist > 30.0? 2 * min_dist : 30.0))
			{
				good_matches.push_back(coarse_matches[i]);
			}
		}

		
		// 5. use essential matrix Ransac filter again
		std::vector <cv::Point2f> points_last;
		std::vector <cv::Point2f> points_next;
		for (int i = 0; i < (int)good_matches.size(); ++i)
		{
			points_last.push_back(last_key_pts[good_matches[i].queryIdx].pt);
			points_next.push_back(next_key_pts[good_matches[i].trainIdx].pt);
		}

		std::vector<unsigned char> mask(points_last.size());
		essential_mat = cv::findEssentialMat(points_last, points_next,
			intrinsic_mat, cv::RANSAC, 0.999, 1.0, mask);
		std::cout << "essential_mat = " << essential_mat << std::endl;
		std::vector<cv::DMatch> optimize_matches;
		for (int i = 0; i < mask.size(); i++)
		{
			if (mask[i])
			{
				last_2f.push_back(points_last[i]);
				next_2f.push_back(points_next[i]);
				optimize_matches.push_back(good_matches[i]);
			}
		}
		std::cout << "optimize_matches.size = " << optimize_matches.size() << std::endl;

		// 6. draw match result
		if (is_draw) 
		{
			cv::Mat coarse_match_imgs;
			cv::Mat good_match_imgs;
			cv::Mat optimize_match_imgs;
			cv::drawMatches(last_img, last_key_pts, next_img, next_key_pts, coarse_matches, coarse_match_imgs);
			cv::drawMatches(last_img, last_key_pts, next_img, next_key_pts, good_matches, good_match_imgs);
			cv::drawMatches(last_img, last_key_pts, next_img, next_key_pts, optimize_matches, optimize_match_imgs);

			cv::namedWindow("coarse_matches", cv::WINDOW_NORMAL);
			cv::namedWindow("good_matches", cv::WINDOW_NORMAL);
			cv::namedWindow("optimize_matches", cv::WINDOW_NORMAL);

			cv::imshow("coarse_matches", coarse_match_imgs);
			cv::imshow("good_matches", good_match_imgs);
			cv::imshow("optimize_matches", optimize_match_imgs);
			cv::waitKey(0);
		}

	};


	void pose_estimate_2d2d(const std::vector<cv::Point2f> & last_match_pts, const std::vector<cv::Point2f> & next_match_pts,
		cv::Mat & R, cv::Mat & t, const cv::Mat & intrinsic_mat, const cv::Mat & essential_mat) 
	{
		// use epipolar constraint to recover camera poses
		cv::recoverPose(essential_mat, last_match_pts, next_match_pts, intrinsic_mat, R, t);
		std::cout << "R = " << R << std::endl;
		std::cout << "t = " << t << std::endl;
		cv::Mat t_x = (cv::Mat_<double>(3, 3) << 
			0.0, -t.at<double>(2, 0), t.at<double>(1, 0),
			t.at<double>(2, 0), 0.0, -t.at<double>(0, 0),
			-t.at<double>(1, 0), t.at<double>(0, 0), 0.0);
		cv::Mat verify_essential_mat = t_x * R;
		std::cout << "essential_mat = " << essential_mat << std::endl;
		std::cout << "verify_essential_mat = " << verify_essential_mat << std::endl;

		// calculate homography matrix to prevent degenracy
		//cv::Mat homography_mat = cv::findHomography(last_match_pts, next_match_pts, cv::RANSAC, 3);
		//std::cout << "homography matrix" << homography_mat << std::endl;
	};

	void verify_epipolar_constraint(const cv::Mat & essential_mat, const cv::Mat & intrinsic_mat,
		const std::vector<cv::Point2f> & last_pts, const std::vector<cv::Point2f> & next_pts)
	{
		cv::Mat inv_K;
		cv::invert(intrinsic_mat, inv_K);
		std::cout << "eye£¨3£© = " << intrinsic_mat * inv_K << std::endl;
		cv::Mat fundamental_mat = inv_K.t() * essential_mat * inv_K;
		for (int i = 0; i < last_pts.size(); ++i) 
		{
			cv::Mat p1 = (cv::Mat_<double>(3, 1) << last_pts[i].x, last_pts[i].y, 1);
			cv::Mat p2 = (cv::Mat_<double>(3, 1) << next_pts[i].x, next_pts[i].y, 1);

			cv::Mat constraint_loss = p2.t() * fundamental_mat * p1;
			std::cout << "epipolar constraint = " << constraint_loss << std::endl;
		}
	
	};


}