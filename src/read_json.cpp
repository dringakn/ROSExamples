#include <iostream>
#include <fstream>
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

int main()
{
    json js = json::parse(std::ifstream("roi.json"));

    // string type = data.value("type", "not found");
    // cout << "Type : " << type << endl;

    auto features = js["features"];
    std::cout << "Features: " << features.size() << std::endl;
    for (auto &&f : features)
    {
        std::cout << "Type: " << f["geometry"]["type"] 
        << ", Description: " <<f["properties"]["description"]
        << ", Name: " <<f["properties"]["name"]
        << ", Coordinates: " << f["geometry"]["coordinates"][0].size() << std::endl;
    }

    // for (auto it = features.begin(); it != features.end(); ++it)
    //     cout << "it : " << * << endl;

    return 0;
}