pkg_check_modules(OSGVIZ osgViz)

if($ENV{USE_QT5})
    rock_find_qt5(Widgets Core Gui REQUIRED)
    set(QT_LIBS Qt5::Core Qt5::Gui)
endif()

if(OSGVIZ_FOUND)
    rock_vizkit_plugin(smurf_collidable-viz
        SmurfCollidableViz.cpp
        DEPS smurf
        DEPS_PKGCONFIG osgViz
        MOC SmurfCollidableViz.hpp
        HEADERS SmurfCollidableViz.hpp
        LIBS ${QT_LIBS}
    )
    rock_vizkit_plugin(smurf_visual-viz
        SmurfVisualViz.cpp
        DEPS smurf
        DEPS_PKGCONFIG osgViz
        MOC SmurfVisualViz.hpp
        HEADERS SmurfVisualViz.hpp
        LIBS ${QT_LIBS}
    )
else()
    message(STATUS "Could not find osgViz: vizkit plugins cannot be generated")
endif()
