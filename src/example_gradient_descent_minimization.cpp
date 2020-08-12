/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        Determine optimized parameters using N measurements and an initial
    Notes:
        Array is used Instead of Vector because it support element operations.
        include_directory (... include ${Eigen_INCLUDE_DIRS} ...)

*/
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/AutoDiff>
#include <iostream> // cout, ios, endl, std etc
#include <random>   // randome number generator
#include <ros/ros.h>
#include <random>

using namespace std;
using namespace Eigen;

double f(Vector2d x) {
	return pow(x(0), 2) + 3 * pow(x(1), 2); // f(x,y) = x^2 + 3y^2
}

// Jacobian: J=[df/dx df/dy]
Vector2d df(Vector2d x) {
	return Vector2d(2 * x(0), 6 * x(1)); // f(x,y) = x^2 + 3y^2
}

// Hessian: H=[d2f/d2x df/dydx]
//			  [df/dxdy d2f/d2y]
Matrix2d d2f(Vector2d x = Vector2d::Zero()) {
	Matrix2d H;
	H << 2, 0, 0, 6;
	return H; // f(x,y) = x^2 + 3y^2
}

double f(Vector2d x, Vector2d theta) {
	return theta(0)*pow(x(0), 2.0) + theta(1)*pow(x(1), 2.0); // f(x,y) = a*x^2 + b*y^2
}

Vector2d df(Vector2d x, Vector2d theta) {
	return Vector2d(x(0)*x(0), x(1)*x(1)); // f(x,y) = theta1 * x1^2 + theta2 * x2^2
}

int main(int argc, char** argv) {
	
	cout.precision(5);
	cout.setf(ios::fixed | ios::showpos);
	
	Vector2d p(2, 2); // Initial guess
	int n = 100;
	int N = 1000;
	const double eps = 1e-6;
	double fnew, fprev = f(p), error, tau;

	for (int i = 0; i < N; i++) {
		Vector2d p2(p(0)*p(0), p(1)*p(1));
		// tau can be solved by ∇f(ξ)f(x)=0, i.e. dot product of jacobian evaluated at (ξ=x- τ∇f(x)) and x
		tau = (p2(0) + 9 * p2(1)) / (2 * p2(0) + 54 * p2(1)); // τ = (x^2+9y^2)/(2x^2+54y^2)
		p -= tau * df(p); // xn = x - tau * df/dx // Gradient descent method
		//p -= d2f().inverse() * df(p); // xn = x - (H)^-1 * df/dx // Newton Method
		fnew = f(p);
		error = abs(fnew - fprev);
		fprev = fnew;
		cout << i << '\t' << tau << '\t' << p(0) << '\t' << p(1) << '\t' << error << endl;

		if (error <= eps) {
			break; // Stop the iterations if the error becomes small
		}
	}

	/*
		Second Approach - Inefficent but works when, τ can't be computable
	*/
	p.setConstant(20); // Initial guess
	tau = 0.1; // τ = constant/dynamic!!!
	fprev = f(p);
	cout << "---" << endl;
	for (int i = 0; i < N; i++) {
		p -= tau * df(p); // xn = x - tau * df/dx
		fnew = f(p);
		error = abs(fnew - fprev);
		cout << i << '\t' << p(0) << '\t' << p(1) << '\t' << error << endl;
		fprev = fnew;
		if (error <= eps) {
			break; // Stop the iterations if the error becomes small
		}
	}

	/*
		Least square using Gradient Descent
		Example:
			Assming a model f(x,y) = a*x^2 + b*y^2, determine the parameters X=[a,b]
			using gradient descent algorithm.
			In term of SSE to be minimum
				J(x,y,X) = (1/n)* Sum(a*x^2 + b*y^2 - f)^2
				X = X - tau * dJ		---		(A)
				Where tau < 1 (step-size must be small for convergence)
				dJ(x,y,X) = (1/2n)* Sum(d/dX(a*x^2 + b*y^2 - f)^2)
				first parameters
						  = (1/n)*Sum((a*x^2 + b*y^2 - f)*d(a*x^2 + b*y^2 - f)/da)
						  = (1/n)*Sum((a*x^2 + b*y^2 - f)*(x^2))
						  = (1/n)*Sum((a*x^2 + b*y^2 - f)*(x^2))
				Similarly second parameter
						  = (1/n)*Sum((a*x^2 + b*y^2 - f)*(y^2))
				dJ = (1/n)*Sum((a*xi^2 + b*yi^2 - fi)*[x^2;y^2])
				Use the equation (A) to find X iteratively.
	*/

	default_random_engine rng;
	normal_distribution<double> dist(0, 0.01);
	auto grng = [&](double) {return dist(rng); };
	ArrayXd x = ArrayXd::LinSpaced(n, -3, 3);
	ArrayXd y = ArrayXd::LinSpaced(n, -3, 3);
	ArrayXd fval = x.pow(2) + 3 * y.pow(2) + ArrayXd::NullaryExpr(n, 1, grng);
	//cout << "x:" << endl << x.transpose() << endl;
	//cout << "y:" << endl << y.transpose() << endl;
	//cout << "f:" << endl << fval.transpose() << endl;

	p = Vector2d(2, 4); // Initial guess, diverges if away
	tau = 0.01; // τ = constant
	Vector2d X(x(0), y(0));
	fprev = f(X, p);
	cout << "---" << endl;
	for (int i = 0; i < N; i++) {
		Vector2d J(0, 0);
		for (int j = 0; j < n; j++) {
			X = Vector2d(x(j), y(j));
			double res = f(X, p) - fval(j);
			J += (res*df(X, p));
		}
		J *= (1.0 / n);
		p -= (tau * J); // xn = x - tau * J
		fnew = f(X, p);
		error = abs(fnew - fprev);
		cout << i << '\t' << p(0) << '\t' << p(1) << '\t' << error << endl;
		fprev = fnew;
		if (error <= eps) {
			break; // Stop the iterations if the error becomes small
		}
	}

	return 0;
}

