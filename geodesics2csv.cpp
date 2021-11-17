#include <iostream>
#include <Eigen/Eigen>
#include <igl/read_triangle_mesh.h>
#include <igl/exact_geodesic.h>

int main(int argc, char **argv) {
	if(argc < 2 || argc > 3) {
		std::cerr << "Syntax: " << argv[0] << " filename.obj [start_vertex_index]" << std::endl;
		exit(1);
	}

	int start_vertex_idx = 0;
	if(argc == 3) {
		start_vertex_idx = atoi(argv[2]);
	}

	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	igl::read_triangle_mesh(argv[1], V, F);

	Eigen::VectorXi VS, FS, VT, FT;
	// The selected vertex is the source
	VS.resize(1);
	VS << start_vertex_idx;
	// All vertices are the targets
	VT.setLinSpaced(V.rows(), 0, V.rows() - 1);
	Eigen::VectorXd d;
	igl::exact_geodesic(V, F, VS, FS, VT, FT, d);

	assert(V.cols() == 3);
	assert(d.rows() == V.rows());

	std::ofstream ofs(std::string(argv[1]) + ".geodesic.csv");
	for(Eigen::Index i = 0; i < V.rows(); i++) {
		ofs << d[i] << " " << V.coeff(i, 0) << " " << V.coeff(i, 1) << " " << V.coeff(i, 2) << std::endl;
	}
	ofs.close();
}