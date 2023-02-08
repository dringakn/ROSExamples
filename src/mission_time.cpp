#include <vector>
#include <cmath>
#include <iostream>

double total_mission_time(const std::vector<std::pair<double, double>> &waypoints,
                          double max_velocity, double max_acceleration)
{
    double total_time = 0;
    double distance, acc_time, acc_distance;
    double dx, dy;
    auto current_position = waypoints[0];
    auto next_position = current_position;
    for (int i = 1; i < waypoints.size(); i++)
    {
        // std::cout << next_position.first << ", "<< next_position.second << std::endl;
        next_position = waypoints[i];
        dx = next_position.first - current_position.first;
        dy = next_position.second - current_position.second;
        current_position = next_position;

        distance = sqrt(dx * dx + dy * dy);
        acc_time = max_velocity / max_acceleration;
        acc_distance = 0.5 * max_velocity * acc_time;
        if (distance < 2.0 * acc_distance)
        {
            total_time += (2.0 * sqrt(distance / max_acceleration));
        }
        else
        {
            total_time += ((2.0 * acc_time) + ((distance - (2.0 * acc_distance)) / max_velocity));
        }
    }
    return total_time;
}

int main(int argc, char const *argv[])
{
    std::vector<std::pair<double, double>> waypoints;
    waypoints.push_back(std::make_pair<double, double>(0, 0));
    waypoints.push_back(std::make_pair<double, double>(100, 0));
    waypoints.push_back(std::make_pair<double, double>(100, 100));
    waypoints.push_back(std::make_pair<double, double>(0, 100));
    waypoints.push_back(std::make_pair<double, double>(0, 0));
    double vmax = atof(argv[1]);
    double amax = atof(argv[2]);
    std::cout << "Total mission time: " << total_mission_time(waypoints, vmax, amax) << std::endl;
    return 0;
}
