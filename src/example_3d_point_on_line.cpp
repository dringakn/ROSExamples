/*
	Author: Dr. Ing. Ahmad Kamal Nasir
	Email: dringakn@gmail.com
	Description:

	Vector equation of the line using two points Ps and Pe:
		P(t) = Ps + t * v
		Ps = initial point
		v  = direction vector (Pe-Ps)

	Parametric equaiton of the line can be obtained from vector equation
		x = xs + t*a
		y = ys + t*b
		z = zs + t*c
		Note (a, b, c) are the (x, y, z) elements of the direction vector

	Symmetric equation of the line can be obtained from parametric equaiton by solving for t
		(x-xs)/a = (y-ys)/b = (z-zs)/c

	For a point P1 to be on the line the value of estimated using symmetric equation must be equal.
	OR
	The dot product of the direction vector and the point vector must be zero.

	Example:
		P1 = (2, 3, 4)
		Ps = (1, -2, -3)
		Pe = (4, -5, -1)
		=> v = (Pe-Ps) = (4-(1), -5-(-2), -1-(-3)) = (3, -3, 2) = (a, b, c)
		=> tx = (x1 - xs)/a = (2 - (1)) / 3 = 1/3
		=> ty = (y1 - ys)/b = (3 - (-2)) / -3 = -5/3
		=> tz = (z1 - zs)/c = (4 - (-3)) / 2 = 7/3
		Since tx != ty != tz, thus P1 is not on the line.

	Notes:
	https://gabrielgambetta.com/computer-graphics-from-scratch/02-basic-raytracing.html
*/

#include <eigen3/Eigen/Dense> //Matrix
#include <bits/stdc++.h>	  // C++

using namespace Eigen;

int main()
{
	VectorXd P1 = Vector3d(2, 3, 4);   // query point to be find on the ray.
	VectorXd Ps = Vector3d(1, -2, -3); // Ray start point
	VectorXd Pe = Vector3d(4, -5, -1); // Ray end point

	Vector3d u = P1 - Ps; // Point vector from ray starting point toward point

	Vector3d v = Pe - Ps;				   // Direction vector
	double tx = (P1.x() - Ps.x()) / v.x(); // t = (x - xs) / a
	double ty = (P1.y() - Ps.y()) / v.y(); // t = (y - ys) / b
	double tz = (P1.z() - Ps.z()) / v.z(); // t = (z - zs) / c
	std::cout << "Direction vector:\n"
			  << v << std::endl
			  << "Scaling factors: " << tx << ", " << ty << ", " << tz << std::endl;
	std::cout << "Dot product (u.v): " << u.dot(v) << std::endl;

	return 0;
}
