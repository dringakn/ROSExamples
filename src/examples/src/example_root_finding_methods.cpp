/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
	Given real-valued continuous function f(x), find a root 'r' such that f(r)=0
	Example: f(x) = x^3 + cos(x) + exp(x) + (x+1)^0.5 => x=-0.9994988
	Bisection method:
		Find two points such that f(a)*f(b) < 0
		Let c = (a+b)/2 and compute f(c), f(c) will have oppisite sign to atleast one of
		the point a,b (because both are oppisite sign).
		If f(c)*f(a) < 0 then r in (a,c)
		If f(c)*f(b) < 0 then r in (c,b)
		If f(c) = 0 then r = c
		Repeat above steps until one of the following criteria is fullfilled
		|f(c)| <= tolerance1
		|b-a| <= tolerance2
		Number of Maximum iterations
		Combination of the above

	Fixed Point Iteration:
		Find the find the root of equation f(x) = 0, we rewrite it as x+f(x)=x => x=g(x)
		Idea: the root r=g(r); r is called fixed point.
		Initial guess x0, compute g(x0), hopefully x1 = g(x0) is close to r
		Algorithm:
			Choose x0
			Do iterations until stop criteria
				x(k+1) = g(x(k))
			end
		Stop criteria:
		- |x(k+1) - x(k)| <= eps
		- |g(x(k)) - x(k)| <= eps
		- Maximum number of iterations have reached
		- Any combination of the following

	Netwon Method:
		Find r s.t. f(r) = 0, given initial guess x0
		The main idea is to approximate f(x) by linearization at x(k) to obtain x(k+1).
		x(k+1) = x(k) - f(x(k))/f'(x(k))
	Secant Method:
		It is similar to the newton method, but used when difficult to find the f'(x).
		It requires two initial guesses.
		It's convergence is super-linear i.e e(k+1) = M * e(k)^1.62   0.5(1+5^0.5)=1.62
		x(k+2) = x(k) - f(x(k))*(x(k)-x(k+1))/(f(x(k))-f(x(k+1))
		For implementation, store the previous values of f(x).


	Note:
		Bisect always converges but slow, it can be used as a starting point for complex problem.
		n >= (ln(b-a))/(2*eps*ln2)
		It is a roboust method but very slow.

		Fixed point iteration is even slower compared to the bisect method.
		It's convergence is linear. It doesn't gurantes convergence.
		If |g'(x)| < 1, solution converges, otherwise
		If |g'(x)| > 1, solution diverges

		Newton method is the fastes convergence method.
		It's convergence is quadratic i.e. e(k+1) = M*e(k)^2
		But it's not good with initial guess.
		Bisection is very stable, if we manage to find a and b. But it's convergence
		is slow, if you can start with bisection method and switch to Newton method
		when one find a good initial solution or sufficient close.

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

double f1(double x) {
	// f(X) = x^3 + cos(x) + exp(x) + (x+1)^0.5
	return pow(x, 3) + cos(x) + exp(x) + sqrt(x + 1);
}

double f2(double x) {
	// f(X) = x - cos(x)
	return x - cos(x);
}

double df2(double x) {
	// f(X) = x - cos(x)
	return 1.0 + sin(x);
}

double f3(double x) {
	// f(X) = exp(-2x)*(x-1)
	return exp(-2 * x)*(x - 1.0);
}

double f4(double x) {
	// f(X) = x^2 - 3
	return pow(x, 2) - 3.0;
}

int main(int argc, char** argv) {

	cout.precision(6);
	cout.setf(ios::fixed | ios::showpos);
	int N = 1000;
	const double eps = 1e-9;
	double a = -1e0, b = 1e0, c = (a + b) / 2; // How to find initial two opposit numbers
	double fa, fb, fc;
	double error = 0;

	cout << "Bisection-------------" << endl;
	// Bisection method
	for (int i = 0; i < N; i++) {
		cout << i << '\t' << a << '\t' << b << '\t' << c << '\t' << f2(c) << endl;
		fa = f2(a);
		fb = f2(b);
		c = 0.5*(a + b);
		fc = f2(c);
		if (fa*fc < 0) {
			b = c;
		} else if (fc*fb < 0) {
			a = c;
		} else if (fc == 0) {
			break; // c is the root
		}
		if (abs(fc) <= eps || abs(a - b) <= eps) {
			break; // Stop the iterations if the error becomes small
		}
	}

	cout << "Fixed-------------" << endl;
	// Fixed point method
	double x = 0.99; // Initial guess
	double fx, gx; // Given f(x) = 0, rewrite it x = g(x)
	double xp = x;
	for (int i = 0; i < N; i++) {
		fx = x - cos(x);
		gx = cos(x);
		error = abs(x - gx);
		x = gx;
		cout << i << '\t' << x << '\t' << fx << '\t' << error << endl;
		if (error <= eps || abs(xp - x) <= eps) {
			break; // Stop the iterations if the error becomes small
		}
		xp = x;
	}

	cout << "Newton-------------" << endl;
	// Newton method
	x = 0.99; // Initial guess
	xp = x;
	double dfx; // Given f(x) = 0, x(k+1) = x(k) - f(x(k))/f'(x(k))
	for (int i = 0; i < N; i++) {
		fx = f2(x);
		dfx = df2(x);
		x = x - fx / dfx;
		error = abs(x - xp);
		cout << i << '\t' << x << '\t' << fx << '\t' << dfx << endl;
		if (error <= eps) {
			break; // Stop the iterations if the error becomes small
		}
		xp = x;
	}

	cout << "Secent-------------" << endl;
	// Secent method
	double p[2] = { 5.51,5.50 }; // Two initial guesses
	double fp[2] = { f4(p[0]),f4(p[1]) };// x(k+1) = x(k) - f(x(k))*(x(k)-x(k-1))/(f(x(k))-f(x(k-1))
	for (int i = 0; i < N; i++) {
		error = abs(p[0] - p[1]);
		cout << i << '\t' << p[0] << '\t' << p[1] << '\t' << fp[0] << '\t' << fp[1] << '\t' << error << endl;
		dfx = (fp[1] - fp[0]) / (p[1] - p[0]); // Numerical derivative
		double temp = p[1] - (fp[1] / dfx); // Important to implement the memory
		fp[0] = fp[1];
		fp[1] = f4(temp);
		p[0] = p[1];
		p[1] = temp;
		if (error <= eps ) {
			break; // Stop the iterations if the error becomes small
		}
	}

	return 0;
}

