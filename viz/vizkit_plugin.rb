Vizkit::UiLoader.register_3d_plugin('SmurfCollidableViz', 'tools_smurf', 'SmurfCollidableViz')
Vizkit::UiLoader.register_3d_plugin_for('SmurfCollidableViz', "smurf/Collidable", :updateData )
Vizkit::UiLoader.register_3d_plugin('SmurfVisualViz', 'tools_smurf', 'SmurfVisualViz')
Vizkit::UiLoader.register_3d_plugin_for('SmurfVisualViz', "smurf/Visual", :updateData )
