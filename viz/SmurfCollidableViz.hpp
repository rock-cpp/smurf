#pragma once
#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <smurf/Collidable.hpp>

namespace vizkit3d
{
    class SmurfCollidableViz
        : public vizkit3d::Vizkit3DPlugin<smurf::Collidable>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        SmurfCollidableViz();
        ~SmurfCollidableViz();

    Q_INVOKABLE void updateData(smurf::Collidable const &sample)
    {vizkit3d::Vizkit3DPlugin<smurf::Collidable>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(smurf::Collidable const& Collidable);

    private:
        struct Data;
        Data* p;
    };
    VizkitQtPluginHeaderDecls(SmurfCollidableViz)
}
