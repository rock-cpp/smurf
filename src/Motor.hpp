#ifndef MOTOR_H
#define MOTOR_H
#include <configmaps/ConfigData.h>
#include <mars/interfaces/MotorData.h>

namespace smurf{
    
    class Motor
    {
        Motor(configmaps::ConfigMap motorData);
        
    protected:
        
        mars::interfaces::MotorData marsMotor;
        //std::string joint;
        //std::string name;
        //std::string type;
        //double p;
        //double i; 
        //double d; 
        //double maxEffort; 
        //double maxSpeed;
        //double maxValue;
        //double minValue;
    };
    
}

#endif // MOTOR_H
