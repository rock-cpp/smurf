#pragma once
#include <osgViz/Object.h>
#include <smurf/Collidable.hpp>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgDB/ReadFile> 
#include <urdf_model/model.h>

namespace vizkit3d
{
    class SmurfCollidable : public osgviz::Object
    {
        public:
            SmurfCollidable(const std::shared_ptr<smurf::Collidable> collidable)
            {
                urdf::Collision collision = collidable->getCollision();
                switch(collision.geometry->type)
                {
                    case urdf::Geometry::BOX:
                        addBox(collidable);
                        break;
                    case urdf::Geometry::CYLINDER:
                        addCylinder(collidable);
                        break;
                    case urdf::Geometry::MESH:
                        addMesh(collidable);
                        //             addMesh(visual, frameId, uuid);
                        break;
                    case urdf::Geometry::SPHERE:
                        addSphere(collidable);
                        break;
                    default:
                        break;
                }
            }
            void addMesh(const std::shared_ptr<smurf::Collidable> collidable)
            {
                urdf::Collision collision = collidable->getCollision();
                boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh>(collision.geometry);
                assert(mesh.get() != nullptr);
                std::cout << "MESH: " << mesh->filename << std::endl;
                osg::Node* meshNode = osgDB::readNodeFile(mesh->filename); 
                addChild(meshNode);
            }
            void addBox(const std::shared_ptr<smurf::Collidable> collidable)
            {
                urdf::Collision collision = collidable->getCollision();
                boost::shared_ptr<urdf::Box> urdfBox = boost::dynamic_pointer_cast<urdf::Box>(collision.geometry);
                assert(urdfBox.get() != nullptr);
                osg::Box* box = new osg::Box(osg::Vec3(0,0,0), urdfBox->dim.x, urdfBox->dim.y, urdfBox->dim.z);
                osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(boxDrawable);
                addChild(geode);
                //     ShapeDrawable* boxDrawable = new ShapeDrawable(boundingBox);
                //     Geode* geode = new Geode();
                //     geode->addDrawable(boxDrawable);
            }
            void addCylinder(const std::shared_ptr<smurf::Collidable> collidable)
            {
                urdf::Collision collision = collidable->getCollision();
                boost::shared_ptr<urdf::Cylinder> urdfCylinder = boost::dynamic_pointer_cast<urdf::Cylinder>(collision.geometry);
                assert(urdfCylinder.get() != nullptr);
                //x = length, y = radius, z = not used
                osg::Cylinder* cylinder = new osg::Cylinder(osg::Vec3(0,0,0), urdfCylinder->radius, urdfCylinder->length);
                osg::ShapeDrawable* cylinderDrawable = new osg::ShapeDrawable(cylinder);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(cylinderDrawable);
                addChild(geode);
            }
            void addSphere(const std::shared_ptr<smurf::Collidable> collidable)
            {
                urdf::Collision collision = collidable->getCollision();
                boost::shared_ptr<urdf::Sphere> urdfSphere = boost::dynamic_pointer_cast<urdf::Sphere>(collision.geometry);
                assert(urdfSphere.get() != nullptr);
                //x = length, y = radius, z = not used
                osg::Sphere* sphere = new osg::Sphere(osg::Vec3(0,0,0), urdfSphere->radius);
                osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(sphereDrawable);
                addChild(geode);
            }
    };
}
