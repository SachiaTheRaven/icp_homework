// -*- mode: c++ -*-
#pragma once

#include <string>

#include <QGLViewer/qglviewer.h>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/EigenVectorT.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/Sqrt3T.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/LoopT.hh>
#include <QtWidgets>
#include <QHBoxLayout>
#include "ICPProject.h"

using qglviewer::Vec;

class MyViewer : public QGLViewer {
	Q_OBJECT

public:
	explicit MyViewer(QWidget* parent);
	virtual ~MyViewer();


	
	

	inline double getCutoffRatio() const;
	inline void setCutoffRatio(double ratio);
	inline double getMeanMin() const;
	inline void setMeanMin(double min);
	inline double getMeanMax() const;
	inline void setMeanMax(double max);
	inline const double* getSlicingDir() const;
	inline void setSlicingDir(double x, double y, double z);
	inline double getSlicingScaling() const;
	inline void setSlicingScaling(double scaling);
	//void findFurthestVertex();
	void loopSubdivision();
	void sqrt3Subdivision();
	Vec& cp(const int i, const int j);
	Vec PartialU(double u, double v, std::vector<double> bernsteinU, std::vector<double>bernsteinV);
	Vec PartialV(double u, double v, std::vector<double> bernsteinU, std::vector<double>bernsteinV);
	Vec PartialUV(double u, double v, std::vector<double> bernsteinU, std::vector<double>bernsteinV);
	Vec PartialUU(double u, double v, std::vector<double> bernsteinU, std::vector<double>bernsteinV);
	Vec PartialVV(double u, double v, std::vector<double> bernsteinU, std::vector<double>bernsteinV);
	void ElevateBezier();
	bool openMesh(const std::string& filename, bool update_view = true, bool is_point_cloud=true);
	bool openBezier(const std::string& filename, bool update_view = true);
	bool saveBezier(const std::string& filename);


signals:
	void startComputation(QString message);
	void midComputation(int percent);
	void endComputation();

protected:
	virtual void init() override;
	virtual void draw() override;
	virtual void drawWithNames() override;
	virtual void postSelection(const QPoint& p) override;
	virtual void keyPressEvent(QKeyEvent* e) override;
	virtual void mouseMoveEvent(QMouseEvent* e) override;
	virtual QString helpString() const override;

private:
	
	struct MyTraits : public OpenMesh::DefaultTraits {
		using Point = OpenMesh::Vec3d; // the default would be Vec3f
		using Normal = OpenMesh::Vec3d;
		VertexTraits{
			double mean;              // approximated mean curvature
			double u;
			double v;
		};
		FaceTraits{
			bool toBeColored = false;
		};

	};
	//Modified for Eigen
	struct EigenTraits : public  OpenMesh::DefaultTraits {
		using Point = Eigen::Vector3d;
		using Normal = Eigen::Vector3d;
		///using TexCoord2D = Eigen::Vector2d;

	
		VertexTraits{
			double mean;              // approximated mean curvature
			double u;
			double v;
		};
		FaceTraits{
			bool toBeColored = false;
		};
	};


	using Vector = Eigen::Vector3d; //OpenMesh::VectorT<double, 3>;
	using MyMesh = OpenMesh::TriMesh_ArrayKernelT<EigenTraits>; //changed from MyTraits




	double GetMeanCurvatureOfBezier(MyMesh mesh, OpenMesh::VertexHandle vtx);


	// Mesh
	void updateMesh(bool update_mean_range = true);
	void updateVertexNormals();
	void localSystem(const Vector& normal, Vector& u, Vector& v);
	double voronoiWeight(MyMesh mesh,MyMesh::HalfedgeHandle in_he);
	void updateMeanMinMax();
	void updateMeanCurvature(bool update_min_max = true);
	void DrawMesh(MyMesh mesh);
	void DuplicateMesh();

	Eigen::MatrixX3d load_vertices_from_mesh(MyMesh mesh);
	void load_vertices_into_mesh(Eigen::MatrixX3d vertices,MyMesh mesh);


	// Bezier
	static void bernsteinAll(size_t n, double u, std::vector<double>& coeff);
	void generateMesh(MyMesh mesh);

	// Visualization
	void setupCamera();
	Vec meanMapColor(double d) const;
	void drawControlNet() const;
	void drawAxes() const;
	void drawAxesWithNames() const;
	static Vec intersectLines(const Vec& ap, const Vec& ad, const Vec& bp, const Vec& bd);

	// Other
	void fairMesh();

	//////////////////////
	// Member variables //
	//////////////////////

	enum class ModelType { NONE, MESH, BEZIER_SURFACE, POINT_CLOUD } model_type;
	
	// Mesh
	std::vector<MyMesh> meshes;

	//Matrix of vertices
	std::vector<Eigen::MatrixX3d> mesh_vertices;

	// Bezier
	size_t degree[2];
	std::vector<Vec> control_points;

	// Visualization
	double mean_min, mean_max, cutoff_ratio;
	bool show_control_points, show_solid, show_wireframe;
	enum class Visualization { PLAIN, MEAN, SLICING, ISOPHOTES } visualization;
	GLuint isophote_texture, environment_texture, current_isophote_texture, slicing_texture;
	Vector slicing_dir;
	double slicing_scaling;
	int selected_vertex;
	struct ModificationAxes {
		bool shown;
		float size;
		int selected_axis;
		Vec position, grabbed_pos, original_pos;
	} axes;
	std::string last_filename;

	//homework
	OpenMesh::PolyConnectivity::VertexHandle furthest;
	QMessageBox newMessageBox;
	int sizeN;
	int sizeM;





};

#include "MyViewer.hpp"