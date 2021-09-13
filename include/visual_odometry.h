#include <string>
#include <vector>
#include <Eigen/Eigen>


// front-end 
namespace VO 
{
	// use IMU groud truth
	void from_imu(const std::vector <double>& extrinsic_timestamp_vec,
		const std::vector<std::vector<double>> &extrinsic_vec, const double & depth_time_stamp,
		float * translate_vec, float * rotate_mat);

	Eigen::Isometry3d from_imu(const std::vector <double>& extrinsic_timestamp_vec,
		const std::vector<std::vector<double>> &extrinsic_vec, const double & depth_time_stamp);


}

