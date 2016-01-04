#ifndef COLLIDABLE_H
#define COLLIDABLE_H

#include <string>
#include <urdf_model/model.h>


namespace smurf{
    
    class Collidable
    {
    public:
        Collidable(const std::string& name, const int& bitmask, const urdf::Collision& collision);
        std::string getName();
        int getBitmask();
        urdf::Collision getCollision();
        void setGroupId(const int id)
        {
            this->groupId = id;
        }
        int getGroupId()
        {
            return this->groupId;
        }
    private:
        std::string name;
        int bitmask;
        urdf::Collision collision;
        int groupId;
    };
    
};

#endif // COLLIDABLE_H
