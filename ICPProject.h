#pragma once
using namespace std;
#include <map>
#include "happly.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Eigenvalues"

namespace ICP
{
	void WriteToOBJ(string filename, Eigen::MatrixX3d mat);
	double distance(Eigen::Vector3d src, Eigen::Vector3d dst);
	std::map<int, std::pair<int, double>> closest_point_slow(const Eigen::MatrixX3d& src, const Eigen::MatrixX3d& dst);
	std::map<int, std::pair<int, double>> closest_point_fast(const Eigen::MatrixX3d& src, const Eigen::MatrixX3d& dst);
	Eigen::MatrixX3d load_vertices(happly::PLYData& data);
	Eigen::MatrixX3d initialTransform(double rad_angle, Eigen::Vector3d axis, Eigen::MatrixX3d vertices);
	Eigen::Vector4d calculate_center_of_mass(Eigen::MatrixX3d vertices);
	Eigen::Matrix4d calculate_rotation_base(int n, Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::MatrixX3d src, Eigen::MatrixX3d dst, std::map<int, std::pair<int, double>> matches);
	Eigen::Quaterniond calculate_max_eigen(Eigen::Matrix4d mat);
	double calculate_mean_squared_error(std::map<int, std::pair<int, double>> pairs);
	Eigen::MatrixX3d ICPIteration(Eigen::MatrixX3d src, Eigen::MatrixX3d dst, std::map<int, std::pair<int, double>> closest_pairs);
	Eigen::MatrixX3d run_ICP(Eigen::MatrixX3d src, Eigen::MatrixX3d dst, int maxiter, double minerror);







};