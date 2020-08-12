/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Eigen Geometry module provides two different types of transformations.
    Abstract Transformation:
        Such as rotations(angle and axis, quaternions), translations, scaling.
        These transformations are not represented by matrices, never the less
        we can mix them with matrices and vectors in expressions and convert
        them to the matrices if required.
    Projective/Affine Transformation:
        These are represented by the "Transform" class. These are matrices.
        We can construct the Transform from abstract transformation as follows
            Transform t(AngleAxis(angle,axis))
    Note:
        Eigen default to column major storage.
        Projective transformation is the most general set of transformation.
        Affine transformation is a subset of the projective transformation.
        Euclidian transformation is a subset of the Affine transformation.
        Length, Angles and Areas are preserved in the Euclidian transformation.
        Parallel lines remain parallel in  Affine transformation.
    REFERENCE:
        https://eigen.tuxfamily.org/dox/GettingStarted.html
        https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
        Size can be 2,3,4 for fixed size square matrices or X for dynamic size
 */
#include <eigen3/Eigen/Dense>    //Matrix
#include <eigen3/Eigen/Geometry> //Transformation
#include <iostream>              //cout, std
#include <random> //default_random_engine, uniform_real_distribution
#include <vector> //vector

#define D2R(x) (x * M_PI / 180.0)
#define R2D(x) (x * 180.0 / M_PI)

using namespace std;
using namespace Eigen;

Matrix4f create_affine_matrix(float a, float b, float c, Vector3f trans) {
  Transform<float, 3, Eigen::Affine> t;
  t = Translation<float, 3>(trans);
  t.rotate(AngleAxis<float>(a, Vector3f::UnitX()));
  t.rotate(AngleAxis<float>(b, Vector3f::UnitY()));
  t.rotate(AngleAxis<float>(c, Vector3f::UnitZ()));
  // t = AngleAxis<float>(c, Vector3f::UnitZ());
  // t.prerotate(AngleAxis<float>(b, Vector3f::UnitY()));
  // t.prerotate(AngleAxis<float>(a, Vector3f::UnitX()));
  // t.pretranslate(trans);
  return t.matrix();
}
int main() {

  // Random numner
  default_random_engine rng;
  uniform_real_distribution<double> dist(-5, 5);

  // Sample Points
  double arrVertices[] = {-1.0, -1.0, -1.0, 1.0,  -1.0, -1.0, 1.0, 1.0,
                          -1.0, -1.0, 1.0,  -1.0, -1.0, -1.0, 1.0, 1.0,
                          -1.0, 1.0,  1.0,  1.0,  1.0,  -1.0, 1.0, 1.0};
  // Create a 3x8 matrices of 3D points
  MatrixXd mVertices = Map<Matrix<double, 3, 8>>(arrVertices);
  cout << "Vertices:" << endl;
  cout << mVertices << endl << "---" << endl;
  // Create a transform object
  Transform<double, 3, Projective> t(AngleAxisd(D2R(45), Vector3d::UnitX()));
  // Extract each column and convert it to a homegenous vector and transform it
  cout << t * mVertices.colwise().homogeneous() << endl << "---" << endl;

  // Create Transformation object
  Transform<double, 3, Projective> t1(
      AngleAxis<double>(D2R(30), Vector3d::UnitX()));
  Transform<double, 3, Projective> t2(
      AngleAxis<double>(D2R(10), Vector3d(0, 0, 0)));
  Transform<double, 3, Projective> t3(AngleAxisd::Identity());
  Transform<double, 3, Projective> t4 =
      Transform<double, 3, Projective>::Identity();
  t4.scale(1);
  // t4.rotate(Quaterniond::UnitRandom());
  t4.rotate(Quaterniond::Identity());
  t4.translate(Vector3d::Random());
  cout << "T4:" << endl << t4.matrix() << endl;

  cout << "Transformation:" << endl;
  cout << "T:" << endl << t2.translation() << endl;
  cout << "R:" << endl << t2.rotation() << endl;
  cout << "Linear:" << endl << t2.linear() << endl;
  cout << "Matrix:" << endl << t2.matrix() << endl;

  // 2D rotation from an angle
  Rotation2D<double> rot2 = Rotation2D<double>(D2R(30));
  cout << "Rotation2D:" << endl;
  cout << "R:" << endl << rot2.matrix() << endl;
  cout << "ANGLE:" << endl << rot2.angle() * R2D(1) << endl; // 0=x, 1=y, 2=z

  // 3D rotation from an angle-axis
  // AngleAxis<double> a = AngleAxisd::AngleAxis(D2R(10), Vector3d::UnitX());
  AngleAxis<double> a = AngleAxisd::Identity();
  a.angle() = D2R(10);
  a.axis() = Vector3d::UnitX();
  cout << "AngleAxis:" << endl;
  cout << "R:" << endl << a.toRotationMatrix() << endl;

  // 3D rotation from quaternion
  Quaterniond q = Quaterniond::Identity(); // UnitRandom
  cout << "Quaternion:" << endl;
  cout << "R:" << endl << q.toRotationMatrix() << endl;
  cout << "RPY:" << endl
       << q.matrix().eulerAngles(0, 1, 2) * R2D(1) << endl; // 0=x, 1=y, 2=z

  // ND Translation
  Translation<double, 3> tra3(1, 2, 4);
  cout << "Translation:" << endl;
  cout << "T:" << endl << tra3.vector() << endl;

  vector<Vector3d> pc1, pc2;
  const int N = 100;
  for (int i = 0; i < N; i++) {
    Vector3d p1, p2;
    p1 << dist(rng), dist(rng), dist(rng);
    // cout << p1 << endl << "---" << endl;
  }
  return 0;
}
