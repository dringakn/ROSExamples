/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
	Euler integration:
		x(t+1) = x(t) + h * f(t, x(t))
	2nd Order Runge-Kutta
		x(t+1) = x(t) + h * f(t, x(t)) + h^2/2! * df(t, x(t))/dt
			   = x(t) + k2
			   where
			   k1 = h * f(t, x(t))
			   k2 = h * f(t + h/2, x(n) + k1/2)
	4th Order Runge-Kutta
		x(t+1) = x(t) + 1/6 * (k1 + 2*k2 + 2*k3 + k4)
			   where
			   k1 = h * f(t, x(t))
			   k2 = h * f(t + h/2, x(n) + k1/2)
			   k3 = h * f(t + h/2, x(n) + k2/2)
			   k4 = h * f(t + h, x(n) + k3)
	Runge-Kutta-Fehlberg Method (Adaptive Step-Size:
		it incorporates an adaptive procedure where an estimate of the turncation
		error withing prescribed error limit. At each step, the algorithm works by
		calculating  and coparing two estimates of the solution.
		If the two results are in close agreement, the calculated approximation is
		accepted. However, if the results don't agree within the prescribed accuracy,
		the step size is reduced. If the step-size is agree by more significant digits
		than is required, the step size is increased
		The local turncation error is O(h5) and calculated as follows:
		x^5(t+1) = x(t) + 16/135. * k1 + 6656/12825. * k3 + 28561/56430. * k4 - 9/50. * k5 + 2/55. * k6
		While the turncation error is of order O(h4) and calculated as follows:
		x^5(t+1) = x(t) + 25/216. * k1 + 1408/2565. * k3 + 2197/4104. * k4 - 1/5. * k5

		x(t+1) = x(t) + 1/6 * (k1 + 2*k2 + 2*k3 + k4)
			   where
			   k1 = h * f(t, x(t))
			   k2 = h * f(t + h/4, x(n) + k1/4)
			   k3 = h * f(t + h*3/8, x(n) + k1*3/32 + k2*9/32)
			   k4 = h * f(t + h*12/13, x(n) + k1*1932/2197 - k2*7200/2197 + k3*7296/2197)
			   k5 = h * f(t + h, x(n) + k1*439/216 - k2*8 + k3*3680/513 - k4*845/4104)
			   k6 = h * f(t + h/2, x(n) - k1*8/27 + k2*2 - k3*3544/2565 + k4*1859/4104 -k5*11/40)
			   The optimal step-size, s, is determined by simply multiplying  the scaling factor
			   s by the current step-size h, where
			   s = (tolerance/(2*error)^0.25
			   tolerance must be specified
			   error = (1/h) * |k1/360 - k3*128/4275 - k4*2197/75240 + k5/50 + k6*2/55|
			   h_min = 0.0001
			   h_max = 0.5000
			   s_min = 0.1
			   s_max = 4

    Note:
        Eigen default to column major storage.
        Size can be 2,3,4 for fixed size square matrices or X for dynamic size.
    REFERENCE:
        https://eigen.tuxfamily.org/dox/GettingStarted.html
        https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
*/

#include <iostream>//cout,cin
#include <iomanip>//setw
#include <Eigen/Dense> // Eigen vector and matrices

using namespace std;
using namespace Eigen;

// Spring-Mass damper system
VectorXd model(const VectorXd &x) {
	const double k = 4, b = 1, m = 4, F = 1;
	/*
		|dx| = |0		1	|   |x| + |0  |
		|  | = |			| * | | + |   | * F
		|dv| = |-k/m	-b/m|	|v| + |1/m|

		X = [x;v] => dX = [dx;dv]

		dX	 = A * X + B * U

		Where
			|-0.5 dx >= 0
		F = |
			|+0.5 dx < 0
		Matlab:
		clc;clear all;close all;format compact;
		b = 1;k = 4;m = 4;f = 1;h = 0.1;t_end = 30;
		t = 0:h:t_end;
		X = [0;0];
		A = [0 1;-k/m -b/m];
		B = [0;1/m];
		C = [1 0;0 1];
		D = [0];
		sys = ss(A,B,C,D);
		[y t] = step(sys,t)
		step(sys,t);
		% plot(t,y);
		grid
		title('Step-response');
	*/
	VectorXd dx = Vector2d::Zero();
	MatrixXd A = Matrix2d::Zero(); A << 0, 1, -k / m, -b / m;
	VectorXd B = Vector2d::Zero(); B << 0, 1 / m;
	//dx = A * x + B * ((x(1) >= 0) ? -f / m : f / m);
	dx = A * x + B * F;
	return dx;
}

// Electrical Motor-Speed system
VectorXd model2(const VectorXd &x) {
	const double b = 0.1, J = 0.01, K = 0.01, R = 1, L = 0.5, V = 1;
	/*
		|d2theta| = |-b/J	K/J	|   |dtheta| + |0  |
		|	    | = |			| * |      | + |   | * V
		|di     | = |-K/L	-R/L|	|i	   | + |1/L|

		X = [dtheta;i] => dX = [d2theta;di]

		dX	 = A * X + B * U

		Where
			|1 t >= 0
		V = |
			|0 dx < 0
		Matlab:
		clc;clear all;close all;format compact;
		b = 0.1;J = 0.01;K = 0.01;R = 1;L = 0.5;h=0.1;t_end = 10;
		t = 0:h:t_end;
		X = [0;0];
		A = [-b/J K/J;-K/L -R/L];
		B = [0;1/L];
		C = [1 0;0 1];
		D = [0];
		sys = ss(A,B,C,D);
		[y t] = step(sys,t)
		step(sys,t);
		% plot(t,y);
		grid
		title('Step-response');
	*/
	VectorXd dx = Vector2d::Zero();
	MatrixXd A = Matrix2d::Zero(); A << -b / J, K / J, -K / L, -R / L;
	VectorXd B = Vector2d::Zero(); B << 0, 1. / L;
	dx = A * x + B * V;
	return dx;
}

// Electrical Motor-Position system
VectorXd model3(const VectorXd &x) {
	const double b = 0.1, J = 0.01, K = 0.01, R = 1, L = 0.5, V = 1;
	/*
		|dtheta | = |0  1	 0	|   |theta | + |0  |
		|d2theta| = |0 -b/J	 K/J| * |dtheta| + |0  | * V
		|di     | = |0 -K/L	-R/L|	|i	   | + |1/L|

		X = [theta;dtheta;i] => dX = [dtheta;d2theta;di]

		dX	 = A * X + B * U

		Where
			|1 t >= 0
		V = |
			|0 dx < 0
		Matlab:
		clc;clear all;close all;format compact;
		b = 0.1;J = 0.01;K = 0.01;R = 1;L = 0.5;h=0.1;t_end = 10;
		t = 0:h:t_end;
		X = [0;0];
		A = [0 1 0;0 -b/J K/J;0 -K/L -R/L];
		B = [0;0;1/L];
		C = [1 0 0;0 1 0;0 0 1];
		D = [0];
		sys = ss(A,B,C,D);
		[y t] = step(sys,t)
		step(sys,t);
		% plot(t,y);
		grid
		title('Step-response');
	*/
	VectorXd dx = Vector3d::Zero();
	MatrixXd A = Matrix3d::Zero(); A << 0, 1, 0, 0, -b / J, K / J, 0, -K / L, -R / L;
	VectorXd B = Vector3d::Zero(); B << 0, 0, 1. / L;
	dx = A * x + B * V;
	return dx;
}

int main() {
	double h = 0.1, t = 0, t_end = 10;
	cout.precision(3);// Show three places after the decimal
	cout.setf(ios::fixed | ios::showpos);// Set fixed point format and '+'

	VectorXd x = Vector2d::Zero();
	cout << "Euler Numerical Integration:" << endl;
	cout << "t" << setw(30) << "(x[0],x[1])" << endl;
	while (t < t_end) {
		x = x + h * model2(x);
		cout << t << setw(15) << "(" << x(0) << "," << x(1) << ")" << endl;
		t += h;
	}

	x = Vector2d::Zero();
	t = 0;
	cout << "2nd Order Runge-Kutta Numerical Integration:" << endl;
	cout << "t" << setw(30) << "(x[0],x[1])" << endl;
	while (t < t_end) {
		VectorXd k1 = h * model2(x);
		VectorXd k2 = h * model2(x + k1 / 2.);
		x = x + k2;
		cout << t << setw(15) << "(" << x(0) << "," << x(1) << ")" << endl;
		t += h;
	}

	x = Vector2d::Zero();
	t = 0;
	cout << "4th Order Runge-Kutta Numerical Integration:" << endl;
	cout << "t" << setw(30) << "(x[0],x[1])" << endl;
	while (t < t_end) {
		VectorXd k1 = h * model2(x);
		VectorXd k2 = h * model2(x + k1 / 2.);
		VectorXd k3 = h * model2(x + k2 / 2.);
		VectorXd k4 = h * model2(x + k3);
		x = x + 1 / 6.0 * (k1 + 2. * k2 + 2. * k3 + k4);
		cout << t << setw(15) << "(" << x(0) << "," << x(1) << ")" << endl;
		t += h;
	}

	x = Vector3d::Zero();
	t = 0;
	double tolerance = 1e-6;
	double h_min = 0.0001, h_max = 0.5000;
	double s_min = 0.1, s_max = 4;

	cout << "Runge-Kutta-Fehlberg Numerical Integration:" << endl;
	cout << "t" << setw(15) << "h" << setw(30) << "(x[0],x[1],x[2])" << endl;
	while (t < t_end) {
		VectorXd k1 = h * model3(x);
		VectorXd k2 = h * model3(x + k1 / 4.);
		VectorXd k3 = h * model3(x + k1 * 3. / 32. + k2 * 9. / 32.);
		VectorXd k4 = h * model3(x + k1 * 1932. / 2197. - k2 * 7200. / 2197. + k3 * 7296. / 2197.);
		VectorXd k5 = h * model3(x + k1 * 439. / 216. - k2 * 8. + k3 * 3680. / 513. - k4 * 845. / 4104.);
		VectorXd k6 = h * model3(x - k1 * 8. / 27. + k2 * 2. - k3 * 3544. / 2565. + k4 * 1859. / 4104. - k5 * 11. / 40.);
		double error = fabs((k1 / 360. - k3 * 128. / 4275. - k4 * 2197. / 75240. + k5 / 50. + k6 * 2. / 55.).minCoeff()) / h;
		double s = pow(0.5 * tolerance / error, 0.25);
		if (s < s_min)s = s_min;
		if (s > s_max)s = s_max;
		if (error < tolerance) {
			x = x + k1 * 25. / 216. + k3 * 1408. / 2565. + k4 * 2197. / 4104. - k5 / 5.;
			cout << t << setw(15) << h << setw(15) << "(" << x(0) << "," << x(1) << "," << x(2) << ")" << endl;
			t += h;
		}
		h *= s;
		if (h < h_min)h = h_min;
		if (h > h_max)h = h_max;
		if (h > (t_end - t))h = t_end - t;// last step
	}

	return 0;
}

