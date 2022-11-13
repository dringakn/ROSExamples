/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        Path smoothing using gradient ascent/descent.
        Find new intermediate points (Don't use start and end points for optimization)
            Minimize (Unsmooth - Smooth)    : (Xi - Yi)^2      --> Yi = Yi + alpha * (Xi - Yi)
        and Minimize (Smooth@i - Smooth@i-1): (Yi - Yi-1)^2    --> Yi = Yi + beta * (Yi - Yi-1)
        by combining first and second --> Yi = Yi + beta * (Yi+1 + Yi-1 - 2Yi)

        First minimize the coordinate of the original path and smoothed path. Then in the same iteration
        minimize the difference betwen the coordinate of smoothed path ath time step i and adjacent time
        steps (i-1, i+1).
    Reference:
    https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
*/

#include <eigen3/Eigen/Dense> //Linear algebra stuff
#include <bits/stdc++.h>      // C++ stuff

using namespace std;   // include standard c++ namespace
using namespace Eigen; // include eigne namespace

std::vector<VectorXd> smooth(std::vector<VectorXd> &path, double alpha, double beta, double tol = 1e-6)
{
    std::vector<VectorXd> smooth_path(path);
    double change, old;
    do
    {
        change = 0;
        for (int i = 1; i < smooth_path.size() - 1; ++i) // Skip first and last point
        {
            for (int j = 0; j < smooth_path[i].size(); ++j) // For each element of the vector
            {
                old = smooth_path[i][j];
                smooth_path[i][j] += alpha * (path[i][j] - smooth_path[i][j]);
                smooth_path[i][j] += beta * (smooth_path[i - 1][j] + smooth_path[i + 1][j] - 2 * smooth_path[i][j]);
                change += fabs(old - smooth_path[i][j]);
            }
        }
    } while (change >= tol);
    return smooth_path;
}

void printPath(std::vector<VectorXd> &path)
{
    for (auto it = path.begin(); it != path.end(); ++it)
        std::cout << "Pt: " << it->x() << ", " << it->y() << ", " << it->z() << std::endl;
}

int main()
{
    VectorXd Ps = Vector3d(0, 0, 1); // Path start point
    VectorXd P1 = Vector3d(0, 1, 1); // Path intermediate point
    VectorXd P2 = Vector3d(0, 2, 1); // Path intermediate point
    VectorXd P3 = Vector3d(1, 2, 2); // Path intermediate point
    VectorXd P4 = Vector3d(2, 2, 2); // Path intermediate point
    VectorXd P5 = Vector3d(3, 2, 2); // Path intermediate point
    VectorXd P6 = Vector3d(4, 2, 2); // Path intermediate point
    VectorXd P7 = Vector3d(4, 3, 3); // Path intermediate point
    VectorXd Pe = Vector3d(4, 4, 3); // Path end point

    std::vector<VectorXd> path, smooth_path;
    path.push_back(Ps);
    path.push_back(P1);
    path.push_back(P2);
    path.push_back(P3);
    path.push_back(P4);
    path.push_back(P5);
    path.push_back(P6);
    path.push_back(P7);
    path.push_back(Pe);


    std::cout << "Before optimization (Gradient descent):" << std::endl;
    printPath(path);

    double alpha = 0.5; // Weight for the data
    double beta = 0.4;  // Weight for the smoothing
    double tolerance = 10e-10;
    smooth_path = smooth(path, alpha, beta, tolerance);

    std::cout << "After optimization (Gradient descent):" << std::endl;
    printPath(smooth_path);

    return 0;
}