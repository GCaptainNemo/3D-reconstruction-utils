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
}