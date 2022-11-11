/*
	Author: Dr. Ing. Ahmad Kamal Nasir
	Email: dringakn@gmail.com
	Description:
		Ray Tracing in 3D. The equation of line in 3D cann't be described by
		slope-intercept form. Furthermore, the line-equation in slope-intercept
		form has singularity (undefined) for vertical line.
		In 2D the implicith line equation can be derived from slope-intercept
		form as follows:
		y = m*x + d => y = (dy/dx)*x + d => dy*x - y*dx + d*dx = 0
		ax + by + c = 0, where a = dy, b = -dx, c = d*dx

		The 3D-equation of the plane in implicit form can be written as
		ax + by + cz + d = 0

		In order to avoid singularity, the equation of a line can be described implicitly
		using following parameteric equation:
		L(t) = (1-t)*S + t*E
		where
		S = Start point
		E = End point
		t = is a scaling factor between [0...1], if t>1 the point is on the
		line, outside the line-segement defined by the two points S, E, beyond
		E. if t<0, the point is beyond S.
		The above parameteric equation can be written as a fuction of each
		coordinate variables (x,y,z), i.e.
		x(t) = (1-t)*sx + t*ex
		y(t) = (1-t)*sy + t*ey
		z(t) = (1-t)*sz + t*ez

		Therefore, in order to detect if the plane intersect with the line we can solve
		the following set of linear equations
		a*x + b*y + c*z + d = 0		=>	a*x + b*y + c*z = -d
		(1-t)*sx + t*ex - x = 0		=>	x + t*(sx-ex)   = sx
		(1-t)*sy + t*ey - y = 0		=>	y + t*(sy-ey)   = sy
		(1-t)*sz + t*ez - z = 0		=>	z + t*(sz-ez)   = sz

		The above equations can be written in matrix form as follows

		|a b c 0    |   |x|   |-d|
		|1 0 0 sx-ex| * |y| = |sx|
		|0 1 0 sy-ey|   |z|   |sy|
		|0 0 1 sz-ez|   |t|   |sz|

		Example:
		2x + 2y + 2z -36 = 0
		S:(0,0,0), E:(2,4,3)
		=> (x,y,z,t) = (4,8,6,2)

		In order to detect if a point is within the triangle formed by three points,
		we could check the weights (a,b,c) of the following equation. If any one of the
		weight is negative, the point is outside of the triangle formed by point A,B,C

		a*Ax + b*Bx + c*Cx = x
		a*Ay + b*By + c*Cy = x
		a*Az + b*Bz + c*Cz = x

		   |Ax Bx Cx|   |a|   |x|
		=> |Ay By Cy| * |b| = |y|
		   |Az Bz Cz|   |c|   |z|

		point P(x,y,z) is outside of the triangle formed by points A,B,C only if min(a,b,c) < 0

		Example:
		A:(-2,2,2), B:(3,-3,-2), C:(4,1,-4), P(1.4,0.6,-1.2)
		=> (a,b,c) = (0.4, 0.2, 0.4)

		The same can be generalized to multiple points
		   |Ax Bx Cx Dx|   |a|   |x|
		=> |Ay By Cy Dy| * |b| = |y|
		   |Az Bz Cz Dz|   |c|   |z|
						   |d|

		point P(x,y,z) is outside of the polygon formed by points A,B,C,D only if min(a,b,c,d) < 0

		Example:
		A:(-2,2,2), B:(3,-3,-2), C:(4,1,-4), D:(3,3, -1), P(1.4,0.6,-1.2)
		=> (a,b,c) = (0, -0.0051, 0.2769, 0.1026)

		*/

#include <eigen3/Eigen/Dense>  //Matrix
#include <iostream>

using namespace std;
using namespace Eigen;

int main()
{
  // Intersection of a ray with a planer patch
  VectorXd Ps = Vector3d(0, 0, 0);			// Ray start point
  VectorXd Pe = Vector3d(2, 4, 3);			// Ray end point
  VectorXd Plane = Vector4d(2, 2, 2, -36);  // 3D plane
  VectorXd Pa = Vector3d(-2, 2, 2);			// Polygon point1
  VectorXd Pb = Vector3d(3, -3, -2);		// Polygon point2
  VectorXd Pc = Vector3d(4, 1, -4);			// Polygon point3
  VectorXd Pd = Vector3d(3, 3, -1);			// Polygon point4

  MatrixXd A = Matrix4d::Zero();

  // Set-up set of linear equations
  A << Plane(0), Plane(1), Plane(2), 0, 1, 0, 0, Ps(0) - Pe(0), 0, 1, .0, Ps(1) - Pe(1), 0, 0, 1, Ps(2) - Pe(2);
  VectorXd b = Vector4d(-Plane(3), Ps(0), Ps(1), Ps(2));
  cout << "A:" << endl << A << endl << "b:" << endl << b << endl;
  cout << "Sol:" << endl << A.inverse() * b << endl;

  // Second test
  A = Matrix3d::Zero();
  A.col(0) = Pa;
  A.col(1) = Pb;
  A.col(2) = Pc;
  b = Vector3d(1.4, 0.6, -1.2);

  cout << "A:" << endl << A << endl << "b:" << endl << b << endl;
  cout << "Sol:" << endl << A.inverse() * b << endl;
  return 0;
}
