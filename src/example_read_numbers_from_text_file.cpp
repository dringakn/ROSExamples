#include <bits/stdc++.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

bool readPoseArrayFile(std::string fname, geometry_msgs::PoseArray &data)
{
    bool result = false;
    std::ifstream file(fname); // fstream
    std::string line;
    while (getline(file, line)) // iostream
    {
        if (line.size()) // Remove empty line
        {
            if (line[0] != '#') // Skip comment line starting with '#'
            {
                std::istringstream ss(line); // sstream
                geometry_msgs::Pose p;
                // Line format: x y z qx qy qz qw
                // Note: if q(x,y,z,w) are missing, by default filled with zero
                ss >> p.position.x >> p.position.y >> p.position.z >> p.orientation.x >> p.orientation.y >> p.orientation.z >> p.orientation.w;
                data.poses.push_back(p);
                result = true;
            }
        }
    }
    return result;
}

int main(int argc, char *argv[])
{
    std::string fname = "/home/ahmad/sample.txt";
    geometry_msgs::PoseArray data;
    if (readPoseArrayFile(fname, data))
    {
        std::cout << data << std::endl;
    }
    else
    {
        std::cout << "Error reading " << fname << std::endl;
    }
    return 0;
}