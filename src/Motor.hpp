#ifndef MOTOR_H
#define MOTOR_H
#include <configmaps/ConfigData.h>
#include <mars/interfaces/MotorData.h>

namespace smurf{
    
    class Motor
    {
    public:
        Motor(configmaps::ConfigMap motorMap);
        
        std::string getName(){ return this->name; }
        
    protected:
        mars::interfaces::MotorData marsMotor;
        std::string name;
    };
    
}

#endif // MOTOR_H
