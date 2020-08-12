/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    	Non-linear solution using Lavenberg Marquardt algorith.
*/

#include <iostream> // cout, ios, endl, std etc
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <random> // randome number generator
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>

using namespace std;
using namespace Eigen;

/***********************************************************************************************/

// Generic functor
// See http://eigen.tuxfamily.org/index.php?title=Functors
// C++ version of a function pointer that stores meta-data about the function
template<typename T, int NX = Dynamic, int NY = Dynamic>
struct Functor {

	// Information that tells the caller the numeric type (eg. double) and size (input / output dim)
	typedef T Scalar; //Required by numericaldiff
	enum { // Required by numerical differentiation module
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};

	// Tell the caller the matrix sizes associated with the input, output, and jacobian
	typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	// Local copy of the number of inputs
	int m_inputs, m_values;

	// Two constructors:
	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	// Get methods for users to determine function input and output dimensions
	int inputs() const { return m_inputs; }
	int values() const { return m_values; }

};

/***********************************************************************************************/

// https://en.wikipedia.org/wiki/Test_functions_for_optimization
// Booth Function
// Implement f(x,y) = (x + 2*y -7)^2 + (2*x + y - 5)^2
struct BoothFunctor : Functor<double> {
	// Simple constructor
	BoothFunctor() : Functor<double>(2, 2) {}

	// Implementation of the objective function
	int operator()(const VectorXd &z, VectorXd &fvec) const {
		double x = z(0);   double y = z(1);
		/*
		 * Evaluate the Booth function.
		 * Important: LevenbergMarquardt is designed to work with objective functions that are a sum
		 * of squared terms. The algorithm takes this into account: do not do it yourself.
		 * In other words: objFun = sum(fvec(i)^2)
		 */
		fvec(0) = x + 2 * y - 7;
		fvec(1) = 2 * x + y - 5;
		return 0;
	}
};

/***********************************************************************************************/

// https://en.wikipedia.org/wiki/Test_functions_for_optimization
// Himmelblau's Function
// Implement f(x,y) = (x^2 + y - 11)^2 + (x + y^2 - 7)^2
struct HimmelblauFunctor : Functor<double> {
	// Simple constructor
	HimmelblauFunctor() : Functor<double>(2, 2) {}

	// Implementation of the objective function
	int operator()(const VectorXd &z, VectorXd &fvec) const {
		double x = z(0);   double y = z(1);
		/*
		 * Evaluate Himmelblau's function.
		 * Important: LevenbergMarquardt is designed to work with objective functions that are a sum
		 * of squared terms. The algorithm takes this into account: do not do it yourself.
		 * In other words: objFun = sum(fvec(i)^2)
		 */
		fvec(0) = x * x + y - 11;
		fvec(1) = x + y * y - 7;
		return 0;
	}
};

/***********************************************************************************************/

/*
	struct my_functor_w_df : NumericalDiff<my_functor> {};
	my_functor_w_df functor;
	LevenbergMarquardt<my_functor_w_df> lm(functor);
*/
struct my_functor : Functor<double> {
	my_functor(void) : Functor<double>(2, 2) {}
	int operator()(const VectorXd &x, VectorXd &fvec) const {
		// Implement y = 10*(x0+3)^2 + (x1-5)^2
		fvec(0) = 10.0*pow(x(0) + 3.0, 2) + pow(x(1) - 5.0, 2);
		fvec(1) = 0;
		return 0;
	}
};

/***********************************************************************************************/

void testBoothFun() {
	cout << "Testing the Booth function..." << endl;
	VectorXd zInit(2); zInit << 1.87, 2.032;
	cout << "zInit: " << zInit.transpose() << endl;
	VectorXd zSoln(2); zSoln << 1.0, 3.0;
	cout << "zSoln: " << zSoln.transpose() << endl;

	BoothFunctor functor;
	NumericalDiff<BoothFunctor> numDiff(functor);
	LevenbergMarquardt<NumericalDiff<BoothFunctor>, double> lm(numDiff);
	lm.parameters.maxfev = 1000;
	lm.parameters.xtol = 1.0e-10;
	cout << "max fun eval: " << lm.parameters.maxfev << endl;
	cout << "x tol: " << lm.parameters.xtol << endl;

	VectorXd z = zInit;
	int ret = lm.minimize(z);
	cout << "iter count: " << lm.iter << endl;
	cout << "return status: " << ret << endl;
	cout << "zSolver: " << z.transpose() << endl;
	cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
}

/***********************************************************************************************/

void testHimmelblauFun() {
	cout << "Testing the Himmelblau function..." << endl;
	// VectorXd zInit(2); zInit << 0.0, 0.0;  // soln 1
	// VectorXd zInit(2); zInit << -1, 1;  // soln 2
	// VectorXd zInit(2); zInit << -1, -1;  // soln 3
	VectorXd zInit(2); zInit << 1, -1;  // soln 4
	cout << "zInit: " << zInit.transpose() << endl;
	cout << "soln 1: [3.0, 2.0]" << endl;
	cout << "soln 2: [-2.805118, 3.131312]" << endl;
	cout << "soln 3: [-3.77931, -3.28316]" << endl;
	cout << "soln 4: [3.584428, -1.848126]" << endl;

	HimmelblauFunctor functor;
	NumericalDiff<HimmelblauFunctor> numDiff(functor);
	LevenbergMarquardt<NumericalDiff<HimmelblauFunctor>, double> lm(numDiff);
	lm.parameters.maxfev = 1000;
	lm.parameters.xtol = 1.0e-10;
	cout << "max fun eval: " << lm.parameters.maxfev << endl;
	cout << "x tol: " << lm.parameters.xtol << endl;

	VectorXd z = zInit;
	int ret = lm.minimize(z);
	cout << "iter count: " << lm.iter << endl;
	cout << "return status: " << ret << endl;
	cout << "zSolver: " << z.transpose() << endl;
	cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
}

/***********************************************************************************************/

void test() {
	VectorXd x(2);
	x(0) = 2.0;
	x(1) = 3.0;
	cout << "x: " << x << endl;

	my_functor functor;
	NumericalDiff<my_functor> numDiff(functor);
	LevenbergMarquardt<NumericalDiff<my_functor>, double> lm(numDiff);
	lm.parameters.maxfev = 2000;
	lm.parameters.xtol = 1.0e-10;
	cout << lm.parameters.maxfev << endl;

	int ret = lm.minimize(x);
	cout << lm.iter << endl;
	cout << ret << endl;

	cout << "x that minimizes the function: " << x << endl;
	cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;

}

/***********************************************************************************************/

int main(int argc, char *argv[]) {

	cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
	testBoothFun();
	testHimmelblauFun();
	test();
	return 0;
}

