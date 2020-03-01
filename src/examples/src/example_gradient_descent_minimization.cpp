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

using namespace std;
using namespace Eigen;

double f(Vector2d x) {
	return pow(x(0), 2) + 3 * pow(x(1), 2); // f(x,y) = x^2 + 3y^2
}

Vector2d df(Vector2d x) {
	return Vector2d(2 * x(0), 6 * x(1));
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
		p -= tau * df(p); // xn = x - tau * df/dx
		fnew = f(p);
		error = abs(fnew - fprev);
		fprev = fnew;
		cout << i << '\t' << tau << '\t' << p(0) << '\t' << p(1) << '\t' << error << endl;

		if (error <= eps) {
			break; // Stop the iterations if the error becomes small
		}
	}

	// Second Approach - Inefficent but works when, τ can't be computable
	p.setConstant(2); // Initial guess
	tau = 0.1; // τ = constant/dynamic!!!
	fprev = f(p);
	cout << "---" << endl;
	for (int i = 0; i < N; i++) {
		Vector2d p2(p(0)*p(0), p(1)*p(1));
		p -= tau * df(p); // xn = x - tau * df/dx
		fnew = f(p);
		error = abs(fnew - fprev);
		cout << i << '\t' << tau << '\t' << p(0) << '\t' << p(1) << '\t' << error << endl;
		fprev = fnew;
		if (error <= eps) {
			break; // Stop the iterations if the error becomes small
		}
	}

	return 0;
}

