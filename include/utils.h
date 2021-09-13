#include <vector>

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
;

void vec2float(std::vector<double> extrinsic_ve, float * translate_vec,
	float * quaternion);

std::vector<std::string> split(const std::string &str, const std::string &pattern);


void interpolate(std::vector<double> & min_extrinsic, std::vector<double> & max_extrinsic,
	const double & cur_time, const double & last_time, const double & next_time, float * quaternion, float * translate_vec);
