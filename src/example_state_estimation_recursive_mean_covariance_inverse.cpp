/*
	Author: Dr. Ing. Ahmad Kamal Nasir
	Email: dringakn@gmail.com
	Description:
		Recursive means is similar to zero-th order (constant or 0th order polynomial)
		least square. Recursive mean can be calculated as follows
		
		mu(k) = mu(k-1) + (1/k) * (X*(k) - mu(k-1))
		Where
		k = sample number
		mu(k-1) = previous mean estimate vector
		mu(k) = new mean estimate vector at current time step
		X*(k) = new measurement vector at current time step

		Recursive covariance can be calculated as follows
		cov(k) = cov(k-1) + (1/(k-1)) * ((X*(k) - mu(k-1))*(X*(k) - mu(k))^T - cov(k-1))
		Note that updated mu(k) is used in order to improve the numerical accuracy.
		Where
		cov(k) = new estimate of covariance matrix
		cov(k-1) = previous covariabce matrix
		k = time step
		mu(k-1) = previous mean estimate
		mu(k) = updated mean estimate

		Recursive covariance inverse can be calculated as follows
		cinv(k) = (k/(k-1)) * (cinv(k-1) + (cinv(k-1)*(X*(k) - mu(k-1))*(X*(k) - mu(k))^T*cinv(k-1) )/(k+(X*(k) - mu(k-1))*cinv(k-1)*(X*(k) - mu(k))^T))
		Note that updated mu(k) is used in order to improve the numerical accuracy.
		Where
		cov(k) = new estimate of covariance matrix
		cov(k-1) = previous covariabce matrix
		k = time step
		mu(k-1) = previous mean estimate
		mu(k) = updated mean estimate

		Recursive covariance inverse can be calculated as follows
		cinv(k) = (k/(k-1)) * (cinv(k-1) + (cinv(k-1)*(X*(k) - mu(k-1))*(X*(k) - mu(k))^T*cinv(k-1) )/(k+(X*(k) - mu(k-1))*cinv(k-1)*(X*(k) - mu(k))^T))
		Note that updated mu(k) is used in order to improve the numerical accuracy.
		Where
		cov(k) = new estimate of covariance matrix
		cov(k-1) = previous covariabce matrix
		k = time step
		mu(k-1) = previous mean estimate
		mu(k) = updated mean estimate

*/

#include <eigen3/Eigen/Dense>    //Matrix
#include <iostream>              //cout, std

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
	int n = (argc > 1) ? atoi(argv[1]) : 10;
	MatrixXd Y = MatrixXd::Random(3, n);
	VectorXd mu = Y.col(0);
	MatrixXd cov = Matrix3d::Zero();
	MatrixXd inv = Matrix3d::Identity();	// Inverse is initialized as eye
	cout << "Y:" << endl << Y << endl;
	for (int i = 1; i < n; i++) {
		const double k = i + 1;
		VectorXd muP = mu;
		MatrixXd covP = cov;
		MatrixXd invP = cov.inverse();
		VectorXd v1 = (Y.col(i) - muP);
		VectorXd v2 = (Y.col(i) - mu);
		mu = muP + (1. / k)*(v2); //i+1 because, the index starts from zero
		cov = covP + (1. / (k - 1))*(v1*v2.transpose() - covP);
		inv = (k/(k-1))*(invP + (invP*v1*v2.transpose()*invP)/(k+v2.transpose()*invP*v1));
		cout << "Ymean:" << i << endl << mu << endl;
		cout << "YCov:" << i << endl << cov << endl;
		cout << "Cinv:" << i << endl << inv << endl;
	}
	// Actual mean and covariance
	cout << "Ymean:" << endl << Y.rowwise().mean() << endl;
	MatrixXd centered = Y.colwise() - Y.rowwise().mean();
	MatrixXd Ycov = (1. / (n - 1))*(centered*centered.transpose());
	cout << "YCov:" << endl << Ycov << endl;
	cout << "Cinv:" << endl << Ycov.inverse() << endl;
	return 0;
}

