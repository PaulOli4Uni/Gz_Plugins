//Start by working with the .cc and .hh files
Change all 'CFile_' and 'HFile_' instances to the name of the said files (they should match)
The project folder name should also be similar to the file names.

In the Cmake list txt file: Change the 'CFile_' to the .cc file name (thus without extention) and change 'Folder_Name_' instances to the project folder name.

## Building Project

Make project in 'Gz_Plugins' folder. (.cc file, .hh file and a CMakeLists file)

~~~
cd Gz_Plugins/<Project Folder>
mkdir build
cd build
cmake ..
make
~~~

This will generate the 'ProjectName' library under `build`.

## Run

The plugin must be attached to an entity to be loaded. Therefore the plugin needs to be added to a entity of a .SDF file

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~
export GZ_SIM_SYSTEM_PLUGIN_PATH=<Plugin_location>/build/ 
^ Above line added to ~/.bashrc file to permanently include, for multiple plugins use:
export GZ_SIM_SYSTEM_PLUGIN_PATH=<Plugin_location1>/build/:<Plugin_location2>/build/  
~~~

Then load the example world:

    gz sim -v 3 hello_world_plugin.sdf

You should see green messages on the terminal like:

```
[Msg] Hello, world! Simulation is paused.
```

Toggle the play / pause buttons to see the message change.
