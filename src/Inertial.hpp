#ifndef INERTIAL_H
#define INERTIAL_H

#include <string>
#include <urdf_model/model.h>


namespace smurf{

    class Inertial
    {
    public:
        Inertial(){};
        Inertial(const urdf::Inertial& inertial);
        void setGroupId(const int id)
        {
            this->groupId = id;
        }
        int getGroupId() const
        {
            return this->groupId;
        }
        urdf::Inertial getUrdfInertial() const
        {
            return this->inertial;
        };
        void setName(std::string name)
        {
            this->name = name;
        };
        std::string getName() const
        {
            return this->name;
        };

        /**Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            throw std::runtime_error("Smurf::Inertial::serialize not implemented");
        }
    private:
        urdf::Inertial inertial;
        int groupId;
        std::string name;
    };

};


#endif // INERTIAL_H
