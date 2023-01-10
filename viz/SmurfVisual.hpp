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
                    case urdf::Geometry::BOX:
                        addBox(visual);
                        break;
                    case urdf::Geometry::CYLINDER:
                        addCylinder(visual);
                        break;
                    case urdf::Geometry::MESH:
                        addMesh(visual);
                        //             addMesh(visual, frameId, uuid);
                        break;
                    case urdf::Geometry::SPHERE:
                        addSphere(visual);
                        break;
                    default:
                        break;
                }
            }
            void addMesh(const std::shared_ptr<smurf::Visual> visual)
            {
                urdf::MeshSharedPtr mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
                assert(mesh.get() != nullptr);
                std::cout << "MESH: " << mesh->filename << std::endl;
                osg::Node* meshNode = osgDB::readNodeFile(mesh->filename);

                // TODO: move to SmurfVisual constructor, since this is generall functionality for all visuals
                // TODO: add texture to the meshNode

                // set Material
                smurf::Material material = visual->getMaterial();
                osg::Vec4 ambientColor = cvtOSGColor(material.getAmbientColor());
                osg::Vec4 specularColor = cvtOSGColor(material.getSpecularColor());
                osg::Vec4 diffuseColor = cvtOSGColor(material.getDiffuseColor());
                float shininess = material.getShininess();

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
                urdf::BoxSharedPtr urdfBox = urdf::dynamic_pointer_cast<urdf::Box>(visual->geometry);
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
            void addCylinder(const std::shared_ptr<smurf::Visual> visual)
            {
                urdf::CylinderSharedPtr urdfCylinder = urdf::dynamic_pointer_cast<urdf::Cylinder>(visual->geometry);
                assert(urdfCylinder.get() != nullptr);
                //x = length, y = radius, z = not used
                osg::Cylinder* cylinder = new osg::Cylinder(osg::Vec3(0,0,0), urdfCylinder->radius, urdfCylinder->length);
                osg::ShapeDrawable* cylinderDrawable = new osg::ShapeDrawable(cylinder);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(cylinderDrawable);
                addChild(geode);
            }
            void addSphere(const std::shared_ptr<smurf::Visual> visual)
            {
                urdf::SphereSharedPtr urdfSphere = urdf::dynamic_pointer_cast<urdf::Sphere>(visual->geometry);
                assert(urdfSphere.get() != nullptr);
                //x = length, y = radius, z = not used
                osg::Sphere* sphere = new osg::Sphere(osg::Vec3(0,0,0), urdfSphere->radius);
                osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere);
                osg::Geode* geode = new osg::Geode();
                geode->addDrawable(sphereDrawable);
                addChild(geode);
            }

            osg::Vec4 cvtOSGColor(urdf::Color color) {
                osg::Vec4 c(0, 0, 0, 1);
                c[0] = color.r;
                c[1] = color.g;
                c[2] = color.b;
                c[3] = color.a;
                return c;
            }
    };
}
