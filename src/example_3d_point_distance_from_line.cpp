/*
	Author: Dr. Ing. Ahmad Kamal Nasir
	Email: dringakn@gmail.com
	Description:

	Method-1: 
		dist(Pt, Ps, Pe) = |(Pt-Pe) x (Pt-Ps)| / |Pe-Ps|
		https://www.math.kit.edu/ianm2/lehre/am22016s/media/distances.pdf

	Method-2: 
	Distance of a point P1 from a line formed using two points (Ps, Pe):
		dist = ||u x v|| / ||v||
		||u x v|| = Magnitude of of the normal vector (cross-product of u and v)
		u = vector formed between specified point and start point of the line.
		v = direction vector of the line

	 Notes:
	 Vector equation of the line using two points Ps and Pe:
		r(t) = r0 + t * v
		r0 = initial point (Ps)
		v  = direction vector (Pe-Ps)

	Parametric equaiton of the line can be obtained from vector equation
		x = xs + t*a
		y = ys + t*b
		z = zs + t*c

	Symmetric equation of the line can be obtained from parametric equaiton by solving for t
		(x-xs)/a = (y-ys)/b = (z-zs)/c

	Example:
		P1 = (2, 3, 4)
		Ps = (1, -2, -3)
		Pe = (4, -5, -1)
		=> v = (Pe-Ps) = (4-(1), -5-(-2), -1-(-3)) = (3, -3, 2)
		=> ||v|| = sqrt(9+9+4) = sqrt(22)
		=> u = (P1-Ps) = (2-(1), 3-(-2), 4-(-3)) = (1, 5, 7)
		=> u x v = (31, 19, -18)
		=> ||u x v|| = sqrt(961+361+324) = sqrt(1646)
		=> dist = sqrt(1646) / sqrt(22) = 8.6498
*/

#include <eigen3/Eigen/Dense> //Matrix
#include <iostream>

using namespace std;
using namespace Eigen;

int main()
{
	// Vector3d P = Vector3d(2, 3, 4);	   // point whose shortest distance from the ray to be found.
	// Vector3d Ps = Vector3d(1, -2, -3); // Ray start point
	// Vector3d Pe = Vector3d(4, -5, -1); // Ray end point
	Vector3d P = Vector3d(10, 2, 0);	 // point whose shortest distance from the ray to be found.
	Vector3d Ps = Vector3d(0, 0, 0); // Ray start point
	Vector3d Pe = Vector3d(2, 0, 0); // Ray end point

	Vector3d v = Pe - Ps;												// Direction vector
	Vector3d u = P - Ps;												// Point vector from ray starting point toward point
	double distance = sqrt(u.cross(v).squaredNorm() / v.squaredNorm()); // sqrt is optimized

	cout << "Direction vector:\n"
		 << v << std::endl
		 << "Point vector from ray starting point:\n"
		 << u << std::endl;
	cout << "Distance of the point from ray:" << distance << std::endl;

	Vector3d P1 = P - Pe, P2 = P - Ps, P3 = Pe - Ps;
	distance = sqrt(P1.cross(P2).squaredNorm() / P3.squaredNorm());
	cout << "Distance of the point from ray:" << distance << std::endl;

	return 0;
}
