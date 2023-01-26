#pragma once
#include <osgViz/Object.h>
#include <smurf/Collidable.hpp>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgDB/ReadFile>

namespace vizkit3d
{
    class SmurfCollidable : public osgviz::Object
    {
        public:
            SmurfCollidable(const std::shared_ptr<smurf::Collidable> collidable)
            {
                switch(collidable->geometry->type)
                {
                    case smurf::Geometry::BOX:
                        addBox(collidable);
                        break;
                    case smurf::Geometry::CYLINDER:
                        addCylinder(collidable);
                        break;
                    case smurf::Geometry::MESH:
                        addMesh(collidable);
                        //             addMesh(visual, frameId, uuid);
                        break;
                    case smurf::Geometry::SPHERE:
                        addSphere(collidable);
                        break;
                    default:
                        break;
                }
            }
            void addMesh(const std::shared_ptr<smurf::Collidable> collidable)
            {
                std::shared_ptr<smurf::Mesh> mesh = std::dynamic_pointer_cast<smurf::Mesh>(collidable->geometry);
                assert(mesh.get() != nullptr);
                std::cout << "MESH: " << mesh->filename << std::endl;
                osg::Node* meshNode = osgDB::readNodeFile(mesh->filename);
                addChild(meshNode);
            }
            void addBox(const std::shared_ptr<smurf::Collidable> collidable)
            {
                std::shared_ptr<smurf::Box> box = std::dynamic_pointer_cast<smurf::Box>(collidable->geometry);
                assert(box.get() != nullptr);
                osg::Box* boxNode = new osg::Box(osg::Vec3(0,0,0), box->size.x(), box->size.y(), box->size.z());
                osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(boxNode);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(boxDrawable);
                addChild(geode);
                //     ShapeDrawable* boxDrawable = new ShapeDrawable(boundingBox);
                //     Geode* geode = new Geode();
                //     geode->addDrawable(boxDrawable);
            }
            void addCylinder(const std::shared_ptr<smurf::Collidable> collidable)
            {
                std::shared_ptr<smurf::Cylinder> cylinder = std::dynamic_pointer_cast<smurf::Cylinder>(collidable->geometry);
                assert(cylinder.get() != nullptr);
                //x = length, y = radius, z = not used
                osg::Cylinder* cylinderNode = new osg::Cylinder(osg::Vec3(0,0,0), cylinder->radius, cylinder->length);
                osg::ShapeDrawable* cylinderDrawable = new osg::ShapeDrawable(cylinderNode);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(cylinderDrawable);
                addChild(geode);
            }
            void addSphere(const std::shared_ptr<smurf::Collidable> collidable)
            {
                std::shared_ptr<smurf::Sphere> sphere = std::dynamic_pointer_cast<smurf::Sphere>(collidable->geometry);
                assert(sphere.get() != nullptr);
                //x = length, y = radius, z = not used
                osg::Sphere* sphereNode = new osg::Sphere(osg::Vec3(0,0,0), sphere->radius);
                osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphereNode);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(sphereDrawable);
                addChild(geode);
            }
    };
}
