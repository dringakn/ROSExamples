/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:

    Note:
        Eigen default to column major storage.
        Size can be 2,3,4 for fixed size square matrices or X for dynamic size.
    REFERENCE:
        https://eigen.tuxfamily.org/dox/GettingStarted.html
        https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
 */

#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class point {
public:
	double x, y;
	point() { x = y = 0; }
	~point() {}
	friend ostream& operator <<(ostream& os, const point& pt) { os << "(" << pt.x << "," << pt.y << ")"; return os; }
};

int main() {
	MatrixXd d;//Dynamic size matrix, stored on heap
	Matrix3d f;//Fixed size matrix, stored on stack, faster access
	d.resize(5, 5); //Resize size, 25
	cout << "d.size():" << d.size() << endl;//25
	cout << "f.size():" << f.size() << endl;// 9
	// Populate matrix
	f << 1, 2, 3,
		4, 5, 6,
		7, 8, 9;
	cout << "m3:" << endl << f << endl;

	// Inititialize as identity
	f = Matrix3d::Identity();
	cout << "f:" << endl << f << endl;

	// Initialize as uniform random [-1,1]
	f = Matrix3d::Random();
	cout << "f:" << endl << f << endl;

	// Initialize with constant
	f = Matrix3d::Constant(3.14152);
	cout << "f:" << endl << f << endl;

	// Custom fixed size matrix
	typedef Matrix<double, 6, 6> Matrix6d;
	Matrix6d m6;
	// Initialize as uniform random [-1,1]
	m6 = Matrix6d::Random();
	cout << "m6:" << endl << m6 << endl;

	/*
	// Custom type dynamic matrix
	typedef Matrix<point, Dynamic, Dynamic> Matrixptd;
	Matrixptd mpt;
	mpt = Matrixptd::Random(3, 3);
	cout << "mpt:" << endl << mpt << endl;
	//*/
	point pt; cout << pt << endl;

	// Define a vector
	//VectorXd v1 = VectorXd::Random(2);
	VectorXd v1(2); v1 << 1, 2;
	cout << "v1:" << endl << v1 << endl;
	cout << "v1.size():" << v1.size() << endl;

	// Matrix multiplication, MatrixXd*MatrixXd results into wrong product
	Matrix<double, 1, 2> m1;
	Matrix<double, 2, 2> m2;
	m1 << 3, 4;
	m2 << 5, 6, 7, 8;
	cout << "m1:" << endl << m1 << endl;
	cout << "m2:" << endl << m2 << endl;

	cout << "m1*m2:" << endl << m1 * m2 << endl;
	cout << "m2*m1^t:" << endl << m2 * m1.transpose() << endl;
	cout << "v1^t*m2:" << endl << v1.transpose() * m2 << endl;
	cout << "m2*v1:" << endl << m2 * v1 << endl;

	MatrixXd m3 = MatrixXd::Zero(3, 3);
	m3 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	cout << "m3+1:" << endl << m3.array() + 1 << endl;
	MatrixXd m4 = exp(m3.array());
	cout << "exp(m3):" << endl << m4 << endl;

	//Eigen values
	cout << "eig(m3):" << endl << m4.eigenvalues() << endl;

	//Solve
	Matrix<double, 2, 1> v2; v2 << 1, 2;
	cout << "v2:" << endl << v2 << endl;
	cout << "m2.lu().sove(v1):" << endl << m2.lu().solve(v1) << endl;
	cout << "m2.lu().sove(v2):" << endl << m2.lu().solve(v2) << endl;

}

