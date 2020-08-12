/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Eigen Geometry module provides two different types of transformations.
    Abstract Transformation:
        Such as rotations(angle and axis, quaternions), translations, scaling.
        These transformations are not represented by matrices, never the less
        we can mix them with matrices and vectors in expressions and convert
        them to the matrices if required.
    Projective/Affine Transformation:
        These are represented by the "Transform" class. These are matrices.
        We can construct the Transform from abstract transformation as follows
            Transform t(AngleAxis(angle,axis))
    Note:
        Eigen default to column major storage.
        Projective transformation is the most general set of transformation.
        Affine transformation is a subset of the projective transformation.
        Euclidian transformation is a subset of the Affine transformation.
        Length, Angles and Areas are preserved in the Euclidian transformation.
        Parallel lines remain parallel in  Affine transformation.
    REFERENCE:
        https://eigen.tuxfamily.org/dox/GettingStarted.html
        https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
        Size can be 2,3,4 for fixed size square matrices or X for dynamic size
 */

#include <iostream>//cout, std
#include <Eigen/Dense>//Matrix
#include <Eigen/Geometry>//Transformation
#include <vector>//vector
#include <random>//default_random_engine, uniform_real_distribution

#define D2R(x) (x*M_PI/180.0)
#define R2D(x) (x*180.0/M_PI)

using namespace std;
using namespace Eigen;

int main() {
	default_random_engine rng;
	uniform_real_distribution<double> dist(-5, 5);

	// Create Transformation object
	Transform<double, 3, Projective> t = Transform<double, 3, Projective>::Identity();
	t.scale(1);
	t.translate(Vector3d(1, 2, 3));
	t.rotate(Quaterniond(AngleAxisd(D2R(30), Vector3d::UnitZ())));
	cout << "Transformation:" << endl;
	cout << "Matrix:" << endl << t.matrix() << endl;
	cout << "Translation:" << endl << t.translation() << endl;
	cout << "Rotation:" << endl << t.rotation().eulerAngles(0, 1, 2)*R2D(1) << endl;

	const int N = 5;
	Matrix<double, 3, N> pc1;

	for (int i = 0; i < N; i++) {
		//Vector3d p1;
		//p1 << dist(rng), dist(rng), dist(rng);
		//pc1.conservativeResize(NoChange, pc1.cols() + 1);
		//pc1.col(pc1.cols() - 1) = p1;
		pc1.col(i) << dist(rng), dist(rng), dist(rng);
	}
	cout << "pc1:" << endl << pc1 << endl << "---" << endl;

	// Extract each column and convert it to a homegenous vector and transform it
	// After the transformation remove the last row
	//MatrixXd pc2 = (t * pc1.colwise().homogeneous()).block(0, 0, 3, N);
	MatrixXd pc2 = t * pc1.colwise().homogeneous();
	pc2.conservativeResize(3, NoChange);
	cout << "pc2:" << endl << pc2 << endl << "---" << endl;

	//Extract the mean vector for both pointclouds
	Vector3d mu1 = pc1.rowwise().mean();
	Vector3d mu2 = pc2.rowwise().mean();
	cout << "mu1:" << endl << mu1 << endl << "---" << endl;
	cout << "mu2:" << endl << mu2 << endl << "---" << endl;

	// Subtract the respective mean from each point in the pointcloud
	pc1.colwise() -= mu1;
	pc2.colwise() -= mu2;
	cout << "pc1:" << endl << pc1 << endl << "---" << endl;
	cout << "pc2:" << endl << pc2 << endl << "---" << endl;

	// Calculate the sum of dot products: Sum(pc1 * pc2^T)
	Matrix3d W = pc1 * pc2.transpose();
	cout << "W:" << endl << W << endl << "---" << endl;

	// Calculate the SVD of the matrix
	JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
	MatrixXd S = svd.singularValues();
	MatrixXd U = svd.matrixU();
	MatrixXd V = svd.matrixV();
	cout << "SVD:" << endl;
	cout << "Singular Values:" << endl << S << endl;
	cout << "U:" << endl << U << endl;
	cout << "V:" << endl << V << endl;

	// Compute rotation
	// R = U*V^T
	// t = mu_x - R*mu_p

	//Matrix3d R = U * V.transpose();
	//Vector3d T = mu1 - R * mu2;
	Matrix3d R = V * U.transpose();
	Vector3d T = mu2 - R * mu1;
	cout << "R:" << endl << R << endl;
	cout << "det(R):" << endl << R.determinant() << endl;
	cout << "Rotation:" << endl << R.eulerAngles(0, 1, 2)*R2D(1) << endl;
	cout << "Translation:" << endl << T << endl;

	return 0;
}

