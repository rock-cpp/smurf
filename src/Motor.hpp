#ifndef MOTOR_H
#define MOTOR_H
#include <configmaps/ConfigData.h>

namespace smurf{

    class Motor
    {
    public:
        Motor();
        Motor(configmaps::ConfigMap map);

        std::string getName() const { return this->name; }

        configmaps::ConfigMap getMotorMap() const { return this->motorMap; }

        /**Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            throw std::runtime_error("Smurf::Motor::serialize not implemented");
        }

    protected:
        configmaps::ConfigMap motorMap;
        std::string name;
    };

}

#endif // MOTOR_H
