/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    y(t) = a0 + a1*x(t) + a2*x^2(t) + a3*x^3(t) + ...
    Exmample: y(t) = 3x^2(t) + 2*x(t) + 1 + N(0,sigma^2)
    =>
    [a0] = |[n       sum(x)      sum(x^2)]    |^-1[sum(y)    ]
    [a1] = |[sum(x)  sum(x^2)    sum(x^3)]    |   [sum(x*y)  ]
    [a2] = |[sum(x^2)sum(x^3)    sum(x^4)]    |   [sum(x^2*y)]
    .
    .
    .
    [an] = |[sum(x^n)sum(x^(n+1) sum(x^(n+n))]|   [sum(x^n*y)]

    Example:
      Least square for a 2D line parameters in slop-intercept form
      y = m*x + c
      |m| = |sum(x^2) sum(x)|^-1  |sum(x*y)|
      |c| = |sum(x)   n     |     |sum(y)  |

      OR

      y = m*x + c  can be written in matrix form as follows

      => |y1| = |x1 1| |m|
         |y2| = |x2 1| |c|
             ...
         |yn| = |xn 1|

     => y = A * x
     Where y = [y1 y2 ... yn]^T
           x = [m c]^T
           A = [x1 1;x2 1;...;xn 1]
        A' * y = A'*A * x
        (A'*A)^-1 * A' * y = x
        Where 
        (A'*A)^-1 * A' = A^-1, is called the Moore-Penrose Pseudo Inverse



    Notes:
    Array is used Instead of Vector because it support element operations.
    include_directory (... include ${Eigen_INCLUDE_DIRS} ...)

*/

#include <iostream> // cout, ios, endl, std etc
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <random> // randome number generator

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_linear_least_square");
  ros::NodeHandle nh;
	cout.precision(5);
	//cout.setf(ios::fixed || ios::showpos);

  int n = (argc > 1) ? atof(argv[1]) : 6;

  std::default_random_engine rng;
  std::normal_distribution<double> dist(0, 0.01);
  auto grng = [&](double) { return dist(rng); };  // Lambda function

  // Vectors are stored as array for element-wise operations
  ArrayXd x = ArrayXd::LinSpaced(n, 1, n);
  ArrayXd y = 3.0 * x.pow(2.0) + 2.0 * x + ArrayXd::Constant(n, 1, 1) +
            ArrayXd::NullaryExpr(n, 1, grng);

  MatrixXd A = MatrixXd::Zero(3, 3);
  VectorXd b = VectorXd::Zero(3, 1);

  cout << "y = a0 + a1*x + a2*x^2" << endl;
  cout << "x:" << endl << x.transpose() << endl;
  cout << "y:" << endl << y.transpose() << endl;
  /*
    A(0, 0) = n;
    A(0, 1) = x.sum();
    A(0, 2) = x.pow(2).sum();
    A(1, 0) = A(0, 1);
    A(1, 1) = A(0, 2);
    A(1, 2) = x.pow(3).sum();
    A(2, 0) = A(1, 1);
    A(2, 1) = A(1, 2);
    A(2, 2) = x.pow(4).sum();
    b(0, 0) = y.sum();
    b(1, 0) = (x * y).sum();
    b(2, 0) = (x.pow(2) * y).sum();
  */
  for (size_t i = 0; i < n; i++) {
    double x1 = x[i], y1 = y[i], x2 = x1 * x1, x3 = x2 * x1, x4 = x3 * x1;
    A(0, 1) += x1;
    A(0, 2) += x2;
    A(1, 2) += x3;
    A(2, 2) += x4;
    b(0, 0) += y1;
    b(1, 0) += (x1 * y1);
    b(2, 0) += (x2 * y1);
  }
  A(0, 0) = n;
  A(1, 0) = A(0, 1);
  A(1, 1) = A(0, 2);
  A(2, 0) = A(1, 1);
  A(2, 1) = A(1, 2);

  VectorXd sol1 = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  VectorXd sol2 = A.colPivHouseholderQr().solve(b);
  VectorXd sol3 = (A.transpose() * A).ldlt().solve(A.transpose() * b); // issue!

  cout << "A:" << endl << A << endl;
  cout << "b:" << endl << b << endl;
  cout << "Actual: [a0,a1,a2]" << endl << "1,2,3" << endl;
  cout << "Sol1: [a0,a1,a2]" << endl << sol1.transpose() << endl;
  cout << "Sol2: [a0,a1,a2]" << endl << sol2.transpose() << endl;
  cout << "Sol3: [a0,a1,a2]" << endl << sol3.transpose() << endl;

  A.resize(n,2);
  b.resize(n,1);
  A.col(0) = x;
  A.col(1) = VectorXd::Constant(n, 1);
  b = y;
  cout << "A:" << endl << A << endl;
  cout << "b:" << endl << b << endl;
  cout << "Sol: [a0,a1,a2]" << endl << (A.transpose()*A).inverse()*A.transpose()*b << endl;



  // LSE for the third degree polynomial
	// y = 3x^2 + 2x + 1 + N(mu,std)
	dist = normal_distribution<double>(0, 0.01);
	x = ArrayXd::LinSpaced(n, 1.0, n); // Generate an array of n equal spaced numbers [1,n]
	y = 3 * x.pow(2) + 2 * x + ArrayXd::Constant(n, 1) + ArrayXd::NullaryExpr(n, 1, grng);
	cout<< "y = 3x^2 + 2x + 1 + N(mu,std)" << endl;
	cout << "x:" << x.transpose() << endl;
	cout << "y:" << y.transpose() << endl;
	A = MatrixXd::Zero(n, 3);
	b = VectorXd::Zero(n);
	A.col(0) = x.pow(2);
	A.col(1) = x;
	A.col(2) = ArrayXd::Constant(n, 1);
	b = y;
	cout << "A:" << endl << A << endl;
	cout << "b:" << endl << b << endl;
	cout << "A'*A:" << endl << A.transpose()*A << endl;
	cout << "(A'*A)^-1:" << endl << (A.transpose()*A).inverse() << endl;
	cout << "(A'*A)^-1*A':" << endl << (A.transpose()*A).inverse()*A.transpose() << endl;
	cout << "(A'*A)^-1*A'*b:" << endl << (A.transpose()*A).inverse()*A.transpose()*b << endl;

	/*
		LSE for the plane equation
		ax + by + cz + d = 0
		The plane parameters (a,b,c,d) can be estimated given points (x,y,z) using
		follwoing two approaches
		Ax = 0, use svd to solve for the last column (smallest eigen value) of v
		or 
		Ax = b (Non-homogenous equation) by reformulating by assuming c=1
		=> z = f(x,y) = -ax - by - d
		|-x1 -y1 -1| |a|   |z1|
		|-x2 -y2 -1| |b| = |z2|
		|...       | |d|   |. |
		|-xn -yn -1|       |zn|
		=> A*x = b => x = A^-1 * b = (A'*A)^-1*A' * b

		or by solving the following error function
		E = Sum(|f(x,y) - z|^2)
		  = Sum(|-ax-by-d - z|^2)
		=> dE/da = Sum(2*|-ax-by-d - z|*x)
		=> dE/db = Sum(2*|-ax-by-d - z|*y)
		=> dE/dd = Sum(2*|-ax-by-d - z|*1)
		Finding the minimum
		Sum(-ax^2-bxy -dx) = Sum(xz)
		Sum(-axy -by^2-dy) = Sum(yz)
		Sum(-ax  -by  -d ) = Sum(z)
		=>
		|a|   |-Sum(x^2) -Sum(xy)  -Sum(x)|^-1 |Sum(xz)|
		|b| = |-Sum(xy)  -Sum(y^2) -Sum(y)|    |Sum(yz)|
		|d|   |-Sum(x)   -Sum(y)   -n     |	   |Sum(z) |
	*/

	dist = normal_distribution<double>(0, 1); // change the distributation parameters
	// Generate random x,y points
	x = ArrayXd::NullaryExpr(n, 1, grng);
	y = ArrayXd::NullaryExpr(n, 1, grng);
	// Calculate the z value given the plane parameters and x,y values
	ArrayXd z = -3.0*x - 2.0*y - ArrayXd::Constant(n, 1) + ArrayXd::NullaryExpr(n,1,grng);
	cout<< "3x + 2y + z + d = N(0,1)" << endl;
	cout << "x:" << x.transpose() << endl;
	cout << "y:" << y.transpose() << endl;
	cout << "z:" << z.transpose() << endl;
	A = MatrixXd::Zero(n, 3);
	b = VectorXd::Zero(n);
	A.col(0) = -x;
	A.col(1) = -y;
	A.col(2) = ArrayXd::Constant(n, -1);
	b = z;
	cout << "A:" << endl << A << endl;
	cout << "b:" << endl << b << endl;
	cout << "(A'*A)^-1*A'*b:" << endl << (A.transpose()*A).inverse()*A.transpose()*b << endl;

	/*
		LSE for the 3D Circle/Sphere:
		The standard form of the circle is as follows
		(x-a)^2 + (y-b)^2 + (z-c)^2 = r^2
		The general form of the circle is as follows
		2ax + 2by + 2cz + (x^2 + y^2 + z^2) + d = 0  
		=> Ax + By + Cz + D - R = 0
		=> a=A/2, b=B/2, c=C/2, r=(D+(x^2+y^2+z^2))^0.5
		Where (a,b,c) represent the center point of the circle and r is the radius of the circle

		=> 
		|x1 y1 z1 1| |A|   |x1^2+y1^2+z1^2|
		|x2 y2 z2 1| |B| = |x2^2+y2^2+z2^2|
		|   ...    | |C|   |     ...      |
		|xn yn zn 1| |D|   |xn^2+yn^2+zn^2|

		=> A*x = b => x = A^-1 * b = (A'*A)^-1*A' * b
		Where
		a = A/2, b = B/2, c = C/2, r = (D+a^2+b^2+c^2)^0.5

		or by solving the following error function
		E = Sum(|f(a,b,c,r)|^2)		!!!!! Verify
		|A|   |Sum(x^2) Sum(xy)  Sum(xz)  Sum(x)|^-1 |Sum(xR)|
		|B| = |Sum(xy)  Sum(y^2) Sum(yz)  Sum(y)|    |Sum(yR)|
		|C| = |Sum(xz)  Sum(yz)  Sum(z^2) Sum(z)|    |Sum(yR)|
		|D|   |Sum(x)   Sum(y)   Sum(z)   n     |	 |Sum(R) |
	*/

	dist = normal_distribution<double>(0, 0.01); // change the distributation parameters
	// Generate random x,y,z points using (a=10,b=11,c=12, r=2.5)
	ArrayXd theta = ArrayXd::LinSpaced(10, 0, 2 * 3.141529);
	n = theta.size();
	x = 10. + 2.5*theta.cos();
	y = 11. + 2.5*theta.sin();
	z = 12. + ArrayXd::NullaryExpr(n, 1, grng);
	cout << "Ax + By + Cz + D - R = 0" << endl;
	cout << "x:" << x.transpose() << endl;
	cout << "y:" << y.transpose() << endl;
	cout << "z:" << z.transpose() << endl;
	A = MatrixXd::Zero(n, 4);
	b = VectorXd::Zero(n);
	A.col(0) = x;
	A.col(1) = y;
	A.col(2) = z;
	A.col(3) = ArrayXd::Constant(n, 1);
	b = x.pow(2) + y.pow(2) + z.pow(2);
	cout << "A:" << endl << A << endl;
	cout << "b:" << endl << b << endl;
	VectorXd sol = (A.transpose()*A).inverse()*A.transpose()*b;
	cout << "(A'*A)^-1*A'*b:" << endl << sol.transpose() << endl;
	sol.head(3) /= 2; // head, tail and segment are used for vectors
	// block, row, col, toprows, bottomrows, leftcols, rightcols, topLeftCorner, bottomLeftCorner, topRightCorner, bottomRightCorner are
	// used with matrix operations
	sol(3) = sqrt(sol(3) + sol(0)*sol(0) + sol(1)*sol(1) + sol(2)*sol(2));
	cout << "Params(xc,yc,zc,r):" << endl << sol.transpose() << endl;


  return 0;
}