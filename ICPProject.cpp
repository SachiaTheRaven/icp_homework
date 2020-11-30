#include <fstream>
#include "ICPProject.h"
//#include "Eigen/QuaternionBase"

using namespace std;

//happly::PLYData bunny_ply("bun000.ply");

void ICP::WriteToOBJ(string filename, Eigen::MatrixX3d mat)
{
	ofstream output;
	output.open(filename);

	for (int i = 0; i < mat.rows(); i++)
	{
		output << "v " << mat(i, 0) << " " << mat(i, 1) << " " << mat(i, 2) << endl;
	}

	for (int i = 0; i < mat.rows(); i++)
	{
		output << "p " << i + 1 << endl;
	}

	output.close();
}

double ICP::distance(Eigen::Vector3d src, Eigen::Vector3d dst)
{
	Eigen::Vector3d diff = src - dst;
	return abs(diff.norm());
}


std::map<int, std::pair<int, double>> ICP::nearest_neighbor(const Eigen::MatrixX3d& src, const Eigen::MatrixX3d& dst) 
{
	int row_src = src.rows();
	int row_dst = dst.rows();
	Eigen::Vector3d vec_src;
	Eigen::Vector3d vec_dst;
	double min = 100;
	int index = 0;
	double dist_temp = 0;
	std::map<int, std::pair<int, double>> closest_pairs;

	for (int i = 0; i < row_src; i++) {
		vec_src = Eigen::Vector3d(src.block<1, 3>(i, 0).transpose());
		min = 100;
		index = 0;
		dist_temp = 0;
		for (int j = 0; j < row_dst; j++) {
			vec_dst = Eigen::Vector3d(dst.block<1, 3>(j, 0).transpose());
			dist_temp = distance(vec_src, vec_dst);
			if (dist_temp < min) {
				min = dist_temp;
				index = j;
			}
		}
		std::pair<int, double> jmin(index, min);
		std::pair<int, std::pair<int, double>> imin(i, jmin);
		closest_pairs.insert(imin);
	}

	return closest_pairs;
}


/*Eigen::MatrixX3d ICP::load_vertices(happly::PLYData& data)
{
	cout << "Loading vertices\n";
	vector<double> model_x = data.getElement("vertex").getProperty<double>("x");
	vector<double> model_y = data.getElement("vertex").getProperty<double>("y");
	vector<double> model_z = data.getElement("vertex").getProperty<double>("z");
	


	Eigen::VectorXd vx = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(model_x.data(), model_x.size());
	Eigen::VectorXd vy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(model_y.data(), model_y.size());
	Eigen::VectorXd vz = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(model_z.data(), model_z.size());


	int size = vx.rows();
	cout << "#rows:" << size << endl;

	Eigen::MatrixX3d vertices;
	cout << "cols:" <<vertices.cols() << endl;
	vertices.conservativeResize(100, Eigen::NoChange);
	vertices << vx.block < 100, 1>(0,0) , vy.block < 100, 1>(0, 0), vz.block < 100, 1>(0, 0);

	cout << "Eventual size:" << vertices.rows() << "x" << vertices.cols() << endl;
	cout << "vertices loaded!\n";

	/*for (int i = 0; i < size; i++)
	{
		vertices.block<1, 3>(i, 0) = Eigen::Vector3d(model_x[i], model_y[i], model_z[i]);
	}*/
	/*return vertices;
}
*/

Eigen::MatrixX3d ICP::initialTransform(double rad_angle, Eigen::Vector3d axis, Eigen::MatrixX3d vertices)
{
	Eigen::Transform<double, 3, Eigen::Affine> transform(Eigen::AngleAxisd(rad_angle, axis));
	Eigen::Matrix3Xd transformed = transform * vertices.transpose();
	return transformed.transpose();

}


/*Eigen::Vector4d ICP::calculate_center_of_mass(Eigen::MatrixX3d vertices)
{
	Eigen::Vector4d center(0, 0, 0,1);
	for (int i = 0; i < vertices.rows(); i++)
	{
		auto vertex = vertices.block<1, 3>(i, 0);
		center.head(3) += vertex.transpose();
	}
	center.head(3) /= vertices.rows();
	return center;
}
*/
/*Eigen::Matrix4d ICP::calculate_rotation_base(int n, Eigen::Vector3d center1, Eigen::Vector3d center2,
	Eigen::MatrixX3d src, Eigen::MatrixX3d dst, std::map<int, std::pair<int, double>> matches)
{
	//cross-covariance mtx
	Eigen::Matrix3d crosscov = Eigen::Matrix3d::Zero(3,3);
	for (int i = 0; i < n; i++)
	{
		auto newpiece1 = (Eigen::Vector3d(src.block<1, 3>(i, 0)) - center1);
		auto newpiece2=(Eigen::Vector3d(dst.block<1, 3>(matches[i].first, 0)) - center2).transpose();
		auto newproduct = newpiece1 * newpiece2;
		crosscov += newproduct;

	}
	crosscov /= n;

	//the Magic Q mtx
	Eigen::Matrix3d A = crosscov - crosscov.transpose();
	Eigen::Vector3d delta = Eigen::Vector3d(A(1,2), A(2, 0), A(0, 1)).transpose();
	double trace = crosscov.trace();

	Eigen::Matrix4d Q(4, 4);
	Q(0, 0) = trace;
	Q.block<1, 3>(0, 1) = delta.transpose();
	Q.block<3, 1>(1, 0) = delta;
	Q.block<3, 3>(1, 1) = crosscov + crosscov.transpose();
	Q.block<3, 3>(1, 1) -= trace * Eigen::Matrix3d::Identity(3, 3);

	return Q;
}
*/
/*Eigen::Quaternionf ICP::calculate_max_eigen(Eigen::Matrix4d mat)
{
	Eigen::EigenSolver<Eigen::Matrix4d> solver(mat,true);
	Eigen::Vector4cf vals = solver.eigenvalues();
	Eigen::Vector4d real_vals = vals.real();
	Eigen::Matrix4d::Index maxindex;
	/*Eigen::scomplex maxval = *//*real_vals.maxCoeff(&maxindex);
	auto maxvector = solver.eigenvectors().col(maxindex);
	Eigen::Quaternionf quat(maxvector[0].real(), maxvector[1].real(), maxvector[2].real(), maxvector[3].real());
	return quat;
}
*/
/*double ICP::calculate_mean_squared_error(std::map<int, std::pair<int, double>> pairs)
{
	double meansquared = 0;
	for (auto pair : pairs)
	{
		meansquared += pow(pair.second.second, 2);
	}
	meansquared /= pairs.size();
	return meansquared;
}
*/
/*Eigen::MatrixX3d ICP::ICPIteration(Eigen::MatrixX3d src, Eigen::MatrixX3d dst, std::map<int, std::pair<int, double>> closest_pairs)
{

	Eigen::Vector4d center_of_mass_src = calculate_center_of_mass(src);
	Eigen::Vector4d center_of_mass_dst = calculate_center_of_mass(dst);


	Eigen::Matrix4d rotation_base_mtx = calculate_rotation_base(src.rows(), center_of_mass_src.head(3), center_of_mass_dst.head(3), src, dst, closest_pairs);
	Eigen::Quaternionf quatR = calculate_max_eigen(rotation_base_mtx);
	Eigen::Matrix3d quatRMat = quatR.normalized().toRotationMatrix();

	//convert rotmat to 4d
	Eigen::Matrix4d rot_4d = Eigen::Matrix4d().Zero(4, 4);
	rot_4d.block(0, 0, 3, 3) = quatRMat;
	rot_4d(3, 3) = 1;

	//compute translation
	Eigen::Vector4d quatT = center_of_mass_dst - rot_4d* center_of_mass_src;

	Eigen::Affine3d trans(Eigen::Translation3d(quatT[0], quatT[1], quatT[2]));
	Eigen::Matrix4d transMat = trans.matrix();


	//convert the original src  to 4d
	int rows = src.rows();
	Eigen::MatrixX4d src_4d = Eigen::MatrixX4d().Zero(rows+1, 4);
	src_4d.block(0,0,rows,3) = src;
	src_4d(rows, 3) = 1;

	



	auto trans_rot = transMat * rot_4d;
	Eigen::MatrixX4d new_src;
	new_src.transpose()=trans_rot* src_4d.transpose();
	return new_src.block(0,0,rows,3);
}
*/

/*Eigen::MatrixX3d ICP::run_ICP(Eigen::MatrixX3d src, Eigen::MatrixX3d dst, int maxiter, double minerror)
{
	int i = 0;
	double error = 10000000; //sok
	while (i<maxiter && error>minerror)
	{
		std::map<int, std::pair<int, double>> closest_pairs = ICP::nearest_neighbor(src, dst);
		error = calculate_mean_squared_error(closest_pairs);
		cout << i << ". iteration: error =  " << error << endl;
		src = ICPIteration(src, dst, closest_pairs);
		i++;
	}

	return src;
}*/
/*int main(int argc, char* argv[])
{
	cout << "I'm running!\n";
	auto bunny_vertices = load_vertices(bunny_ply);
	WriteToOBJ("icp_in_original.obj", bunny_vertices);
	cout << "Bunny loaded!\n";
	auto bunny_rotated = initialTransform(0.2f, Eigen::Vector3d(0.25, 0.25, 0.25), bunny_vertices);
	WriteToOBJ("icp_in_transformed.obj", bunny_rotated);
	cout << "Bunny rotated!\n";
	auto new_bunny = ICP(bunny_vertices, bunny_rotated, 300, 0.00001);
	cout << "Bunny ICP'd\n";
	WriteToOBJ("icp_result.obj", new_bunny);

}*/