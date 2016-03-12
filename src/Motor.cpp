#include "Motor.hpp"

smurf::Motor::Motor(configmaps::ConfigMap map)
{
    motorMap = map;
    name = static_cast<std::string>(map["name"]);
}
