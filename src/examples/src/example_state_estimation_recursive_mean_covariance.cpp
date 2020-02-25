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
	cout << "Y:" << endl << Y << endl;
	for (int i = 1; i < n; i++) {
		const double k = i + 1;
		VectorXd muP = mu;
		MatrixXd covP = cov;
		mu = muP + (1. / k)*(Y.col(i) - mu); //i+1 because, the index starts from zero
		cov = covP + (1. / (k - 1))*((Y.col(i) - muP)*(Y.col(i) - mu).transpose() - covP);
		//cov(k) = cov(k - 1) + (1 / (k - 1)) * ((X*(k)-mu(k - 1))*(X*(k)-mu(k)) ^ T - cov(k - 1))
		cout << "Ymean:" << i << endl << mu << endl;
		cout << "YCov:" << i << endl << cov << endl;
	}
	// Actual mean and covariance
	cout << "Ymean:" << endl << Y.rowwise().mean() << endl;
	MatrixXd centered = Y.colwise() - Y.rowwise().mean();
	MatrixXd Ycov = (1. / (n - 1))*(centered*centered.transpose());
	cout << "YCov:" << endl << Ycov << endl;
	return 0;
}

