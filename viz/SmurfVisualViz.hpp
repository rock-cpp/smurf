#pragma once
#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <smurf/Visual.hpp>

namespace vizkit3d
{
    class SmurfVisualViz
        : public vizkit3d::Vizkit3DPlugin<smurf::Visual>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        SmurfVisualViz();
        ~SmurfVisualViz();

    Q_INVOKABLE void updateData(smurf::Visual const &sample)
    {vizkit3d::Vizkit3DPlugin<smurf::Visual>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(smurf::Visual const& Visual);

    private:
        struct Data;
        Data* p;
    };
}
