#include "../include/demos.h"
#include "../include/draw_trajectory.h"
#include "../include/range2pc.h"

void test_draw_trajectory()
{
	const std::string traj_file = "../data/groundtruth.txt";
	visualize::read_trajectory_file(traj_file);
}

void test_rangeimg2pc() 
{
	const std::string file_dir = "../rgbd_dataset_freiburg1_xyz";
	read_trajector_rgb_depth(file_dir);
};
