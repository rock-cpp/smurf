#ifndef COLLIDABLE_H
#define COLLIDABLE_H

#include <string>
#include <urdf_model/model.h>


namespace smurf{
    
    class Collidable
    {
    public:
        Collidable(const std::string &name, const std::string &bitmask);
        std::string getName();
        std::string getBitmask();
    private:
        std::string name;
        std::string bitmask;
    };
    
};

#endif // COLLIDABLE_H
