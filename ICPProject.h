#pragma once
using namespace std;
#include <map>
#include "happly.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Eigenvalues"

static class ICP
{
public:
	static void WriteToOBJ(string filename, Eigen::MatrixX3d mat);
	static double distance(Eigen::Vector3d src, Eigen::Vector3d dst);
	static std::map<int, std::pair<int, double>> ICP::nearest_neighbor(const Eigen::MatrixX3d& src, const Eigen::MatrixX3d& dst);
	static Eigen::MatrixX3d load_vertices(happly::PLYData& data);
	static Eigen::MatrixX3d initialTransform(double rad_angle, Eigen::Vector3d axis, Eigen::MatrixX3d vertices);
	static Eigen::Vector4d calculate_center_of_mass(Eigen::MatrixX3d vertices);
	static Eigen::Matrix4d ICP::calculate_rotation_base(int n, Eigen::Vector3d center1, Eigen::Vector3d center2, Eigen::MatrixX3d src, Eigen::MatrixX3d dst, std::map<int, std::pair<int, double>> matches);
	static Eigen::Quaternionf calculate_max_eigen(Eigen::Matrix4d mat);
	static double calculate_mean_squared_error(std::map<int, std::pair<int, double>> pairs);
	static Eigen::MatrixX3d ICPIteration(Eigen::MatrixX3d src, Eigen::MatrixX3d dst, std::map<int, std::pair<int, double>> closest_pairs);
	static Eigen::MatrixX3d run_ICP(Eigen::MatrixX3d src, Eigen::MatrixX3d dst, int maxiter, double minerror);







};