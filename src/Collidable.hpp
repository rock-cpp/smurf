#ifndef COLLIDABLE_H
#define COLLIDABLE_H

#include <string>
#include <urdf_model/model.h>


namespace smurf{
    
    class Collidable
    {
    public:
        Collidable(const std::string& name, const int& bitmask, const urdf::Collision& collision);
        std::string getName() const 
        { 
            return this->name; 
        }
        int getBitmask() const
        { 
            return this->bitmask;  
        }
        urdf::Collision getCollision() const 
        { 
            return this->collision; 
        }
        void setGroupId(const int id) 
        { 
            this->groupId = id; 
          
        }
        int getGroupId() const 
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
