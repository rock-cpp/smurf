#ifndef MOTOR_H
#define MOTOR_H
#include <configmaps/ConfigData.h>

namespace smurf{
    
    class Motor
    {
    public:
        Motor(configmaps::ConfigMap map);
        
        std::string getName(){ return this->name; }
        
        configmaps::ConfigMap getMotorMap(){ return this->motorMap; }
        
    protected:
        configmaps::ConfigMap motorMap;
        std::string name;
    };
    
}

#endif // MOTOR_H
