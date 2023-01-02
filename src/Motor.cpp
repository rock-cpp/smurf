#include "Motor.hpp"

smurf::Motor::Motor()
{}

smurf::Motor::Motor(configmaps::ConfigMap map)
{
    motorMap = map;
    motorMap["axis"] = 1; // TODO Why? This value is hardcoded in the current version of smurf (see SMURF::AddConfigMap)
    name = static_cast<std::string>(map["name"]);
}
