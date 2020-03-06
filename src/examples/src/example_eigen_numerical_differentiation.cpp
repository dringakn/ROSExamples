/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    	Eigen numerical differentation example.
*/

#include <iostream> // cout, ios, endl, std etc
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <random> // randome number generator
#include <eigen3/unsupported/Eigen/NumericalDiff>

using namespace std;
using namespace Eigen;

// Create a generic functor to describe system model
template<typename T, int NX = Dynamic, int NY = Dynamic>
struct Functor {
	typedef T Scalar;
	enum {
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }

};

// Create a system derived from the generic functor.
// The System constructor is used to describe the number of system inputs
// and the number of output vector functions.
// The System is described by overloading the () operator.
struct System : Functor<double> {
	System(void) : Functor<double>(3, 3) {}
	int operator()(const VectorXd &x, VectorXd &fvec) const {
		double x1 = x[0], x2 = x[1], x3 = x[2];
		fvec[0] = x1 * x1;
		fvec[1] = x2 * x2 * x2;
		fvec[2] = x3 * x3 * x3 * x3;
		return 0;
	}
};

int main() {
	// Create a system object
	System s1;
	// Create a numerical diff object, specifying the system functor type
	// and num diff type <Forward/Central>.
	NumericalDiff<System, Forward> ndF(s1);
	NumericalDiff<System, Central> ndC(s1);
	VectorXd x(3);// Input vectors for evaluation at specified points
	MatrixXd J(3, 3); // Output jacobian
	x << 0, 0, 0;
	int nfevl = 0; // Number of system evaluations for diff type.
	nfevl = ndF.df(x, J);
	cout << "Forward Jacobian:" << nfevl << endl << J << endl << endl;
	nfevl = ndC.df(x, J);
	cout << "Central Jacobian:" << nfevl << endl << J << endl << endl;
}
