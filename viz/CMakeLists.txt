pkg_check_modules(OSGVIZ osgViz)

rock_find_qt5(Widgets Core Gui REQUIRED)

if(OSGVIZ_FOUND)
rock_vizkit_plugin(smurf-collidable-viz
    SmurfCollidableViz.cpp
    DEPS smurf
    DEPS_PKGCONFIG osgViz
    MOC SmurfCollidableViz.hpp
    HEADERS SmurfCollidableViz.hpp
    LIBS Qt5::Core Qt::Gui)
rock_vizkit_plugin(smurf-visual-viz
    SmurfVisualViz.cpp
    DEPS smurf
    DEPS_PKGCONFIG osgViz
    MOC SmurfVisualViz.hpp
    HEADERS SmurfVisualViz.hpp
    LIBS Qt5::Core Qt5::Gui Qt5::Widgets)
else()
    message(STATUS "Could not find osgViz: vizkit plugins cannot be generated")
endif()
