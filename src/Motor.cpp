#include "Motor.hpp"

smurf::Motor::Motor(configmaps::ConfigMap motorMap)
{
    motorMap = motorMap;
    name = static_cast<std::string>(motorMap["name"]);
}
