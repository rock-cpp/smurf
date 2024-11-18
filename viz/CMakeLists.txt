pkg_check_modules(OSGVIZ osgViz)

if($ENV{USE_QT5})
    rock_find_qt5(Widgets Core Gui REQUIRED)
    set(QT_LIBS Qt5::Core Qt5::Gui)
endif()

if (ROCK_QT_VERSION_4)
	if (vizkit3d_FOUND AND OSGVIZ_PRIMITIVES_FOUND)
		rock_vizkit_plugin(smurf_collidable-viz
        SmurfCollidableViz.cpp
        DEPS smurf
        DEPS_PKGCONFIG osgViz
        MOC SmurfCollidableViz.hpp
        HEADERS SmurfCollidableViz.hpp
        LIBS ${QT_LIBS})

        rock_vizkit_plugin(smurf_visual-viz
        SmurfVisualViz.cpp
        DEPS smurf
        DEPS_PKGCONFIG osgViz
        MOC SmurfVisualViz.hpp
        HEADERS SmurfVisualViz.hpp
        LIBS ${QT_LIBS})
	else()  
		message(STATUS "osgviz or vizkit3d not found ... NOT building the smurf_collidable-viz and smurf_visual-viz plugins")
	endif()

endif()
if (ROCK_QT_VERSION_5)
        if (vizkit3d-qt5_FOUND AND OSGVIZ_PRIMITIVES_FOUND)
        rock_vizkit_plugin(smurf_collidable-viz-qt5
        SmurfCollidableViz.cpp
        DEPS smurf
        DEPS_PKGCONFIG osgViz
        MOC5 SmurfCollidableViz.hpp
        HEADERS SmurfCollidableViz.hpp
        LIBS ${QT_LIBS})
        
        rock_vizkit_plugin(smurf_visual-viz-qt5
        SmurfVisualViz.cpp
        DEPS smurf
        DEPS_PKGCONFIG osgViz  vizkit3d-viz-qt5
        MOC5 SmurfVisualViz.hpp
        HEADERS SmurfVisualViz.hpp
        LIBS ${QT_LIBS})
	 
	else()  
		message(STATUS "osgviz or vizkit3d not found ... NOT building the smurf_collidable-viz and smurf_visual-viz plugins")
	endif()
endif()

