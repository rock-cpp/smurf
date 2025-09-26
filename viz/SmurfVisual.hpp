#pragma once
#include <osgViz/Object.h>
#include <smurf/Visual.hpp>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <urdf_model/model.h>

namespace vizkit3d
{
    class SmurfVisual : public osgviz::Object
    {
        public:
            SmurfVisual(const std::shared_ptr<smurf::Visual> visual)
            {
                switch(visual->geometry->type)
                {
                    case smurf::Geometry::BOX:
                        addBox(visual);
                        break;
                    case smurf::Geometry::CYLINDER:
                        addCylinder(visual);
                        break;
                    case smurf::Geometry::MESH:
                        addMesh(visual);
                        //             addMesh(visual, frameId, uuid);
                        break;
                    case smurf::Geometry::SPHERE:
                        addSphere(visual);
                        break;
                    default:
                        break;
                }
            }
            void addMesh(const std::shared_ptr<smurf::Visual> visual)
            {
                std::shared_ptr<smurf::Mesh> mesh = std::dynamic_pointer_cast<smurf::Mesh>(visual->geometry);
                assert(mesh.get() != nullptr);
                std::cout << "MESH: " << mesh->filename << std::endl;
                osg::Node* meshNode = osgDB::readNodeFile(mesh->filename);

                // TODO: move to SmurfVisual constructor, since this is generall functionality for all visuals
                // TODO: add texture to the meshNode

                // set Material
                osg::Vec4 ambientColor = cvtOSGColor(visual->material->ambientColor);
                osg::Vec4 specularColor = cvtOSGColor(visual->material->specularColor);
                osg::Vec4 diffuseColor = cvtOSGColor(visual->material->diffuseColor);
                float shininess = visual->material->shininess;

                // TODO: taken from mars/graphics/osg_material_manager/OsgMaterial
                osg::ref_ptr<osg::Material> osgMaterial = new osg::Material();;
                osgMaterial->setColorMode(osg::Material::OFF);
                osgMaterial->setAmbient(osg::Material::FRONT_AND_BACK, ambientColor);
                osgMaterial->setSpecular(osg::Material::FRONT_AND_BACK, specularColor);
                osgMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, diffuseColor);
                //material->setEmission(osg::Material::FRONT_AND_BACK, getColor("emissionColor"));
                osgMaterial->setShininess(osg::Material::FRONT_AND_BACK, shininess);
                //material->setTransparency(osg::Material::FRONT_AND_BACK, map.get("transparency", 0.0));

                meshNode->getOrCreateStateSet()->setAttributeAndModes(osgMaterial.get(), osg::StateAttribute::ON);
                addChild(meshNode);
            }
            void addBox(const std::shared_ptr<smurf::Visual> visual)
            {
                std::shared_ptr<smurf::Box> box = std::dynamic_pointer_cast<smurf::Box>(visual->geometry);
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
            void addCylinder(const std::shared_ptr<smurf::Visual> visual)
            {
                std::shared_ptr<smurf::Cylinder> cylinder = std::dynamic_pointer_cast<smurf::Cylinder>(visual->geometry);
                assert(cylinder.get() != nullptr);
                //x = length, y = radius, z = not used
                osg::Cylinder* cylinderNode = new osg::Cylinder(osg::Vec3(0,0,0), cylinder->radius, cylinder->length);
                osg::ShapeDrawable* cylinderDrawable = new osg::ShapeDrawable(cylinderNode);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(cylinderDrawable);
                addChild(geode);
            }
            void addSphere(const std::shared_ptr<smurf::Visual> visual)
            {
                std::shared_ptr<smurf::Sphere> sphere = std::dynamic_pointer_cast<smurf::Sphere>(visual->geometry);
                assert(sphere.get() != nullptr);
                //x = length, y = radius, z = not used
                osg::Sphere* sphereNode = new osg::Sphere(osg::Vec3(0,0,0), sphere->radius);
                osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphereNode);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(sphereDrawable);
                addChild(geode);
            }

            osg::Vec4 cvtOSGColor(smurf::Color color) {
                osg::Vec4 c(0, 0, 0, 1);
                c[0] = color.r;
                c[1] = color.g;
                c[2] = color.b;
                c[3] = color.a;
                return c;
            }
    };
}
