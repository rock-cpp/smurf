#ifndef COLLIDABLE_H
#define COLLIDABLE_H

#include <string>
#include <urdf_model/model.h>
#include <mars/interfaces/contact_params.h>

namespace smurf{
    
    class Collidable
    {
    public:
        Collidable(const std::string& name, const mars::interfaces::contact_params contact_params, const urdf::Collision& collision);
        std::string getName() const 
        { 
            return this->name; 
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
        mars::interfaces::contact_params getContactParams() const
        {
            return this->contact_params;
        }
    private:
        std::string name;
        urdf::Collision collision;
        int groupId;
        mars::interfaces::contact_params contact_params;
    };
    
};

#endif // COLLIDABLE_H
