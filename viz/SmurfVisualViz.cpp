/*
 * SmurfVisualViz.cpp
 * Copyright (C) 2017 rdominguez <rdominguez@INFUSE01-U>
 *
 * Distributed under terms of the MIT license.
 */
#include <iostream>
#include "SmurfVisualViz.hpp"
#include "SmurfVisual.hpp"
#include <memory>
#include <osgViz/Object.h>

using namespace vizkit3d;

struct SmurfVisualViz::Data {
    std::shared_ptr<smurf::Visual> data;
    std::shared_ptr<smurf::Visual> currentVisual;
};

SmurfVisualViz::SmurfVisualViz()
    : p(new Data)
{
}

SmurfVisualViz::~SmurfVisualViz()
{
    delete p;
}

osg::ref_ptr<osg::Node> SmurfVisualViz::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osgviz::Object();
}

void SmurfVisualViz::updateMainNode ( osg::Node* node )
{
    if(p->currentVisual != p->data)
    {
        p->currentVisual = p->data;
        osgviz::Object* obj = dynamic_cast<osgviz::Object*>(node);
        if(obj)
        {
            obj->removeChildren(0, obj->getNumChildren());
            obj->addChild(new SmurfVisual(p->data));
        }
    }
}

void SmurfVisualViz::updateDataIntern(smurf::Visual const& value)
{
//     std::cout << "UPDATE CALLLED" << std::endl;
    if(p->data && value != (*p->data.get()))
    {
        p->data.reset(new smurf::Visual(value));
    }
    else if(!p->data)
    {
         p->data.reset(new smurf::Visual(value));
    }
        
}

VizkitQtPlugin(SmurfVisualViz)
