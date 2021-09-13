#include "../include/utils.h"


void vec2float(std::vector<double> extrinsic_ve, float * translate_vec, float * quaternion)
{
	// tx, ty, tz
	for (int i = 0; i < 3; ++i)
	{
		translate_vec[i] = (float)extrinsic_ve[i];
	}
	// qx, qy, qz, qw
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
		// linear interpolate, for the angle is small. 
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
