#include "Motor.hpp"

smurf::Motor::Motor(configmaps::ConfigMap motorData)
{
    marsMotor.fromConfigMap(motorData);
}
