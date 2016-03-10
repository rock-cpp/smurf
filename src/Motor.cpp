#include "Motor.hpp"

smurf::Motor::Motor(configmaps::ConfigMap motorMap)
{
    marsMotor = mars::interfaces::MotorData();
    std::string prefix = "";
    marsMotor.fromConfigMap(&motorMap, prefix);
    name = static_cast<std::string>(motorMap["name"]);
}
