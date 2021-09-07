#include "../include/visualize.h"

int main() 
{
	const std::string traj_file = "../data/groundtruth.txt";
	read_trajectory_file(traj_file);
	return 0;
}

