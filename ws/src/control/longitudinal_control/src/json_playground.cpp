#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main() 
{
    std::ifstream file("/home/ros/ws/src/control/longitudinal_control/config/control_loop_config.json");

    if (!file.is_open()) 
    {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }
    json jsonData;
    file >> jsonData;

    file.close();

    std::cout << "Value of 'control_interval' in JSON: " << jsonData["control_interval"] << std::endl;
}