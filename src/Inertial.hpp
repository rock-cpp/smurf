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
        int getGroupId()
        {
            return this->groupId;
        }
    private:
        urdf::Inertial inertial;
        int groupId;
    };
    
};


#endif // INERTIAL_H
