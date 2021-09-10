#include "../include/draw_trajectory.h"

int main() 
{
	/*const std::string traj_file = "../data/groundtruth.txt";
	read_trajectory_file(traj_file);
	*/
	const std::string ply_file = "D:/pg_cuda/cuda-tsdf/build/total_rgb_pc.ply";
	visualize_rgbxyz(ply_file);
	return 0;
}

