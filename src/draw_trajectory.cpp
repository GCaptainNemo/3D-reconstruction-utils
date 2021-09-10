#include <iostream>
#include "../include/draw_trajectory.h"
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


void draw_trajector(const std::vector <Eigen::Isometry3d> &poses, const char * option)
{
	pcl::visualization::PCLVisualizer visulizer;
	//visulizer.addCoordinateSystem();
	const float coordinate_len = 0.01;
	pcl::PointXYZ last_O_pos;
	if (strcmp(option, "all") == 0) {
		for (int i = 0; i < poses.size(); i++)
		{
			printf("frame %d \n", i);
			pcl::PointXYZ O_pose;
			O_pose.x = poses[i].translation()[0];
			O_pose.y = poses[i].translation()[1];
			O_pose.z = poses[i].translation()[2];
			if (i == 0)
				visulizer.addText3D("start", O_pose, coordinate_len, 1.0, 1.0, 1.0, "start_point");
			if (i > 0)
				visulizer.addLine(last_O_pos, O_pose, 255, 255, 255, "trac_" + std::to_string(i));

			pcl::PointXYZ X;
			Eigen::Vector3d Xw = poses[i] * (coordinate_len * Eigen::Vector3d(1, 0, 0));
			X.x = Xw[0];
			X.y = Xw[1];
			X.z = Xw[2];
			visulizer.addLine(O_pose, X, 255, 0, 0, "X_" + std::to_string(i));

			pcl::PointXYZ Y;
			Eigen::Vector3d Yw = poses[i] * (coordinate_len * Eigen::Vector3d(0, 1, 0));
			Y.x = Yw[0];
			Y.y = Yw[1];
			Y.z = Yw[2];
			visulizer.addLine(O_pose, Y, 0, 255, 0, "Y_" + std::to_string(i));

			pcl::PointXYZ Z;
			Eigen::Vector3d Zw = poses[i] * (coordinate_len * Eigen::Vector3d(0, 0, 1));
			Z.x = Zw[0];
			Z.y = Zw[1];
			Z.z = Zw[2];
			visulizer.addLine(O_pose, Z, 0, 0, 255, "Z_" + std::to_string(i));

			last_O_pos = O_pose;
		}
	}
	else if (strcmp(option, "traj") == 0)
	{
		for (int i = 0; i < poses.size(); i++)
		{
			printf("frame %d \n", i);
			pcl::PointXYZ O_pose;
			O_pose.x = poses[i].translation()[0];
			O_pose.y = poses[i].translation()[1];
			O_pose.z = poses[i].translation()[2];
			if (i == 0)
				visulizer.addText3D("start", O_pose, coordinate_len, 1.0, 1.0, 1.0, "start_point");
			if (i > 0)
				visulizer.addLine(last_O_pos, O_pose, 255, 255, 255, "trac_" + std::to_string(i));
			last_O_pos = O_pose;

		}
	}
	visulizer.addText3D("end", last_O_pos, coordinate_len, 1.0, 1.0, 1.0, "end_point");
	visulizer.spin();
}


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


void read_trajectory_file(const std::string & file_address) 
{
	std	::vector<Eigen::Isometry3d> poses;

	ifstream in(file_address);
	std::string s;
	int line_index = 0;
	std::string line;
	const int down_sample_factor = 20;
	while (getline(in, s))//着行读取数据并存于s中，直至数据全部读取
	{
		double time, tx, ty, tz, qx, qy, qz, qw;
		if (line_index > 2 && line_index % down_sample_factor == 0) {
			std::vector<std::string> t_R_str = split(s, " ");
			time = stod(t_R_str[0]);
			tx = stod(t_R_str[1]);
			ty = stod(t_R_str[2]);
			tz = stod(t_R_str[3]);
			qx = stod(t_R_str[4]);
			qy = stod(t_R_str[5]);
			qz = stod(t_R_str[6]);
			qw = stod(t_R_str[7]);
			Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz)); //quaternion to matrix
			Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));  // translate vec initialize
			poses.emplace_back(Twr);
		}
		line_index++;
	}

	printf("finish extract camera pose\n");
	printf("total %d frames\n", line_index - 2);
	draw_trajector(poses, "all");
}

void visualize_rgbxyz(const std::string & pc_address) 
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pc_address, *cloud) == -1) {
		std::cout << "Couldn't read file" << "\n";
		return;
	}
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}

};

