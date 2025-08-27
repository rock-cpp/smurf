pkg_check_modules(OSGVIZ osgViz)

if(OSGVIZ_FOUND)
    rock_find_qt4(OPTIONAL)
    rock_find_qt5(OPTIONAL Widgets Core Gui)

    include(RockQt)

    rock_qt_vizkit_plugin(
        TARGETPREFIX smurf_collidable-viz
        QT4_SUFFIX ""
        MISSINGQTDEPS_NOBUILD
        SOURCES SmurfCollidableViz.cpp
        DEPS smurf
        DEPS_PKGCONFIG osgViz
        MOC SmurfCollidableViz.hpp
        HEADERS SmurfCollidableViz.hpp
        LIBS_QT4 Qt4::QtCore Qt4::QtGui
        LIBS_QT5 Qt5::Core Qt5::Gui)
    rock_qt_vizkit_plugin(
        TARGETPREFIX smurf_visual-viz
        QT4_SUFFIX ""
        MISSINGQTDEPS_NOBUILD
        SOURCES SmurfVisualViz.cpp
        DEPS smurf
        DEPS_PKGCONFIG osgViz
        MOC SmurfVisualViz.hpp
        HEADERS SmurfVisualViz.hpp
        LIBS_QT4 Qt4::QtCore Qt4::QtGui
        LIBS_QT5 Qt5::Core Qt5::Gui Qt5::Widgets)
else()
    message(STATUS "Could not find osgViz: vizkit plugins cannot be generated")
endif()
