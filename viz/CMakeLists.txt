rock_vizkit_plugin(smurf-collidable-viz
    SmurfCollidableViz.cpp
    DEPS smurf
    DEPS_PKGCONFIG osgViz 
    MOC SmurfCollidableViz.hpp
    HEADERS SmurfCollidableViz.hpp)
rock_vizkit_plugin(smurf-visual-viz
    SmurfVisualViz.cpp
    DEPS smurf
    DEPS_PKGCONFIG osgViz 
    MOC SmurfVisualViz.hpp
    HEADERS SmurfVisualViz.hpp)
