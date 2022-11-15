/*
	Author: Dr. Ing. Ahmad Kamal Nasir
	Email: dringakn@gmail.com
	Description:

	Given two u and v, find the quternion representaiton of the angle between them.
	d = u . v
	w = u x v
	Thus the quaternion can be represented in angle-axis form, as follows
	q = (cos(theta/2), w/|w| * sin(theta/2))
	which can be simplified to:
		q = (d + sqrt(d^2 + w^2), w)
		OR if u and v are both unit vectors
		q = (1 + d, w)

	Example:
		u = (1, -2, -3)
		v = (4, -5, -1)
		=> d = u . v = (1)(4) + (-2)(-5) + (-3)(-1) = 4+10+3 = 17
		=> w = u x v = (((2)-(15)), -((-1)-(-12)), ((-5)-(-8))) = (-13, -11, 3)
		=> q = (d + sqrt(d^2 + w^2), w).normalize()

	Notes:
		https://www.xarg.org/proof/quaternion-from-two-vectors/

		Using Lagrange identity:
			(uxv)^2 = |u|^2 * |v|^2 - (u.v)^2
			=> (uxv)^2 + (u.v)^2 = |u|^2 * |v|^2
			=> sqrt(w^2 + d^2) = |u||v|
		|u||v| = sqrt(w*w + d*d); Lagrange identity

		For non-unit vectors u and v
		u.v   = |u||v| * cos(theta) => cos(theta) = d/sqrt(w^2+d^2)
		|uxv| = |u||v| * sin(theta) => sin(theta) = w/sqrt(w^2+d^2)

		Half angle trignometric identies:
		cos(theta/2) = sqrt((1 + cos(theta))/2)
		sin(theta/2) = sqrt((1 - cos(theta))/2)
		sin(theta) = 2 * sin(theta/2) * cos(theta/2)
*/

#include <eigen3/Eigen/Dense> //Matrix
#include <bits/stdc++.h>	  // C++

using namespace Eigen;

int main()
{
	// Vector3d u = Vector3d(1, -2, -3); // First vector
	// Vector3d v = Vector3d(4, -5, -1); // Second vector
	Vector3d u = Vector3d(0, 0, 1); // First vector
	Vector3d v = Vector3d(1, 1, 2); // Second vector

	double d = u.dot(v);
	Vector3d w = u.cross(v);
	Vector4d q;							// q = (angle=d + sqrt(d^2 + w^2), axis=w)
	q.w() = d + sqrt(d * d + w.dot(w)); // angle
	q.x() = w.x();						// axis x-component
	q.y() = w.y();						// axis y-component
	q.z() = w.z();						// axis z-component
	q.normalize();						// Normalize before returning
	std::cout << "First vector:\n"
			  << u << std::endl
			  << "Second vector:\n"
			  << v << std::endl
			  << "Quaternion angle:\n"
			  << q << std::endl
			  << "dot product: " << d << std::endl
			  << "normalize cross product:\n"
			  << w.normalized() << std::endl
			  << "non-normalize cross product:\n"
			  << w << std::endl;

	return 0;
}
