/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    	Non-linear Least square using Newton Mehtod
	Example:
		Assming a model f(x,X) = a*x * sin(b*x), determine the parameters X=[a,b]
		given some observations of x and f(x,X),
		We can use following SSE function as a criteria to check quality of data fitting.
			SSE(x,y,X) = (1/n)*Sum(f(xi,X)-yi)^2
			In order to find the parameters that minimize the SSE
			dSSE(x,y,X)/dX = (2/n)*Sum(f(xi,X)-yi) * df(xi,X)/dX
			Where df(xi,X)/dX = [df(xi,X)/da df(xi,X)/db]
			=>
				(2/n) * Sum((a*x*sin(b*x)-yi) * d(a*x*sin(b*x))/dX) = 0
				(2/n) * Sum((a*x*sin(b*x)-yi) * [d(a*x*sin(b*x))/da d(a*x*sin(b*x))/db]) = 0
				Sum((a*x*sin(b*x)-yi) * [d(a*x*sin(b*x))/da d(a*x*sin(b*x))/db]) = 0
				Sum((a*x*sin(b*x)-yi) * [x*sin(b*x)*d(a)/da a*x*d(sin(b*x))/db]) = 0
				Sum((a*x*sin(b*x)-yi) * [x*sin(b*x) a*x^2*cos(b*x)]) = 0
				Therefore, we have the following set of non-linear equations F(x,X)=[F1(x,X) F2(x,X)]^T
			=>  F1(x,X) = Sum((a*x*sin(b*x)-yi) * x*sin(b*x)) = 0
			=>  F2(x,X) = Sum((a*x*sin(b*x)-yi) * a*x^2*cos(b*x)) = 0
			Since, these are non-linear, therefore, they may have multiple solutions.
			We can solve it iteratively using Newton method,
				x(k+1) = x(k) + f(x)/df(x)
			For multiple equations we can write it as follows
				X(k+1) = X(k) + F(x,X)/dF(x,X) = X(k) + J(x,X)^-1 * F(x,X)
				Where F(x,X) = |f(xi,X)-yi|*df(xi,X)
							 = |f(xi,X)-yi * df(xi,X)/da| = |(a*x*sin(b*x)-yi) * x*sin(b*x)    | = |F1(x,X)|
							   |f(xi,X)-yi * df(xi,X)/db| = |(a*x*sin(b*x)-yi) * a*x^2*cos(b*x)|   |F1(x,X)|
				where dF(x,X) = J(x,X) = |dF1(x,X)/dxa dF1(x,X)/dxb|
										 |dF2(x,X)/dxa dF2(x,X)/dxb|
									   = |(x*sin(b*x))^2					-x^2*cos(b*x)*(y-2*a*x*sin(b*x))		|
										 |-x^2*cos(b*x)*(y-2*a*x*sin(b*x))	a^2*x^4+a*x^3*sin(b*x)*(y-2*a*x*sin(b*x)|

    Matlab example to visulize local minima ussing multiple initial conditions:
        clc;close all;clear all
        n = 1000;m = 30;x = -1:2/n:1;
        [X,Y] = meshgrid(x,x);
        Z = X + Y * 1i; % setup complex numbers
        % f(z) = z^4 - 1 => z(k+1) = z(z) - (z^4-1)/(4*z^3)
        for i=1:1:m
            Z = Z - (Z.^4-1) ./ (4*Z.^3);
        end
        subplot(1,2,1);
        image([-1 1], [-1 1], 30*(1+round(imag(Z)))); title('imag(Z)');colorbar;
        subplot(1,2,2);
        image([-1 1], [-1 1], 30*(1+round(real(Z)))); title('real(Z)');colorbar;
        colormap('hot'); 

*/


#include <iostream> // cout, ios, endl, std etc
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <random> // randome number generator

using namespace std;
using namespace Eigen;

double f1(double x, Vector2d X) {
	// f(X) = a*x * sin(b*x)
	return X(0)*x * sin(X(1)*x);
}

// Jacobian: J=[df/da df/db]^T
Vector2d df1(double x, Vector2d X) {
	// f(X) = a*x * sin(b*x)
	double df_da = x * sin(X(1)*x);
	double df_db = X(0) * x*x * cos(X(1)*x);
	return Vector2d(df_da, df_db);
}

// Hessian: H=[d2f/d2a df/dbda]
//			  [df/dadb d2f/d2b]
Matrix2d d2f1(double x, Vector2d X) {
	// f(X) = a*x * sin(b*x)
	double d2f_d2a = 0;
	double df_dadb = x * x * cos(X(1)*x);
	double d2f_d2b = -X(0) * x*x*x * sin(X(1)*x);
	Matrix2d H;
	H << d2f_d2a, df_dadb, df_dadb, d2f_d2b;
	return H;
}


int main(int argc, char** argv) {

	cout.precision(6);
	cout.setf(ios::fixed | ios::showpos);
	int n = 100;
	int N = 1000;
	const double eps = 1e-9;
	double error = 0;

	cout << "Non-Linear Least Squares Newton Method------------" << endl;
	default_random_engine rng;
	normal_distribution<double> dist(0, 0.001);
	auto grng = [&](double) {return dist(rng); };
	ArrayXd x = ArrayXd::LinSpaced(n, -3, 3);
	ArrayXd y = 2 * x * (3 * x).sin() + ArrayXd::NullaryExpr(n, grng);
	//cout << "x:" << endl << x.transpose() << endl;
	//cout << "y:" << endl << y.transpose() << endl;
	//cout << "f:" << endl << fval.transpose() << endl;
    double a0 = (argc>1)?atof(argv[1]):2.1;
    double b0 = (argc>2)?atof(argv[2]):3.1;
	Vector2d X(a0, b0); // Initial guess
	Vector2d Xprev;
	for (int i = 0; i < N; i++) {
		cout << i << '\t' << '\t' << X(0) << '\t' << X(1) << '\t' << error << endl;
		double df_da = 0, df_db = 0;
		double d2f_da2 = 0, df_dadb = 0, d2f_db2 = 0;
		for (int j = 0; j < n; j+=2) { // Half samples.
			double xj = x(j), yj = y(j), a = X(0), b = X(1), theta = b * xj;
			double a2 = a * a, xj2 = xj * xj, xj3 = xj2 * xj, xj4 = xj3 * xj;
			double sin_th = sin(theta), cos_th = cos(theta), sin_th_2 = sin_th * sin_th;
			df_da += (a * xj2 * sin_th_2 - xj * yj * sin_th);
			df_db += (a2 * xj3 * sin_th * cos_th - a * yj * xj2 * cos_th);
			d2f_da2 += (xj2 * sin_th_2);
			df_dadb += (2 * a * xj3 * sin_th * cos_th - xj2 * yj * cos_th);
			d2f_db2 += (a2 * xj4 * (1.0 - 2.0 * sin_th_2) + a * yj * xj3 * sin_th);
		}
		Vector2d F(df_da, df_db);
		Matrix2d J;
		J << d2f_da2, df_dadb, df_dadb, d2f_db2;
		Xprev = X;
		X = X - J.inverse()*F;
		error = (X - Xprev).norm();
		if (error <= eps) {
			break; // Stop the iterations if the error becomes small
		}
	}
	return 0;
}

