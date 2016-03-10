#ifndef MOTOR_H
#define MOTOR_H
#include <configmaps/ConfigData.h>
#include <mars/interfaces/MotorData.h>

namespace smurf{
    
    class Motor
    {
    public:
        Motor(configmaps::ConfigMap motorMap);
        
    protected:
        mars::interfaces::MotorData marsMotor;
    };
    
}

#endif // MOTOR_H
