#include <iostream>
#include "SmurfCollidableViz.hpp"
#include "SmurfCollidable.hpp"
#include <memory>
#include <osgViz/Object.h>

using namespace vizkit3d;

struct SmurfCollidableViz::Data {
    std::shared_ptr<smurf::Collidable> data;
    std::shared_ptr<smurf::Collidable> currentCollidable;
};

SmurfCollidableViz::SmurfCollidableViz()
    : p(new Data)
{
}

SmurfCollidableViz::~SmurfCollidableViz()
{
    delete p;
}

osg::ref_ptr<osg::Node> SmurfCollidableViz::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osgviz::Object();
}

void SmurfCollidableViz::updateMainNode ( osg::Node* node )
{
    if(p->currentCollidable != p->data)
    {
        p->currentCollidable = p->data;
        osgviz::Object* obj = dynamic_cast<osgviz::Object*>(node);
        if(obj)
        {
            obj->removeChildren(0, obj->getNumChildren());
            obj->addChild(new SmurfCollidable(p->data));
        }
    }
}

void SmurfCollidableViz::updateDataIntern(smurf::Collidable const& value)
{
//     std::cout << "UPDATE CALLLED" << std::endl;
    if(p->data && value != (*p->data.get()))
    {
        p->data.reset(new smurf::Collidable(value));
    }
    else if(!p->data)
    {
         p->data.reset(new smurf::Collidable(value));
    }

}
#ifndef USE_QT5
VizkitQtPlugin(SmurfCollidableViz)
#endif