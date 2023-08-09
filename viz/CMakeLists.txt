pkg_check_modules(OSGVIZ osgViz)

rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL Widgets Core Gui)

if(OSGVIZ_FOUND)
    if(ROCK_QT_VERSION_4)
        rock_vizkit_plugin(smurf_collidable-viz
            SmurfCollidableViz.cpp
            DEPS smurf
            DEPS_PKGCONFIG osgViz
            MOC SmurfCollidableViz.hpp
            HEADERS SmurfCollidableViz.hpp
            LIBS Qt4::QtCore Qt4::QtGui)
        rock_vizkit_plugin(smurf_visual-viz
            SmurfVisualViz.cpp
            DEPS smurf
            DEPS_PKGCONFIG osgViz
            MOC SmurfVisualViz.hpp
            HEADERS SmurfVisualViz.hpp
            LIBS Qt4::QtCore Qt4::QtGui)
    endif(ROCK_QT_VERSION_4)
    if(ROCK_QT_VERSION_5)
        rock_vizkit_plugin_qt5(smurf_collidable-viz-qt5
            SmurfCollidableViz.cpp
            DEPS smurf
            DEPS_PKGCONFIG osgViz
            MOC5 SmurfCollidableViz.hpp
            HEADERS SmurfCollidableViz.hpp
            LIBS Qt5::Core Qt5::Gui)
        rock_vizkit_plugin_qt5(smurf_visual-viz-qt5
            SmurfVisualViz.cpp
            DEPS smurf
            DEPS_PKGCONFIG osgViz
            MOC5 SmurfVisualViz.hpp
            HEADERS SmurfVisualViz.hpp
            LIBS Qt5::Core Qt5::Gui Qt5::Widgets)
    endif(ROCK_QT_VERSION_5)
else()
    message(STATUS "Could not find osgViz: vizkit plugins cannot be generated")
endif()
