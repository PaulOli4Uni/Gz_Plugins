
## Position Controller
# General Info

Plugin is used to move a model to a desired location and orientation. Individual pose messages can be sent, or a file can be loaded with a sequence of pose instructions.


Model will keep moving in a direction until it reaches the desired pose or new pose instructions are received 
(Pose instructions have to be valid for new movement to occur)


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
cd Gz_Plugins/<Project Folder>
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build 
^ Above line added to ~/.bashrc file to permanently include 
~~~

## SDF File Plugin options
# Needed


# Optional
<pose_topic> Changes topic name to which Pose information should be sent to. If not present the topic will pulblish to "/model/<model_name>/pos_contr"
Format > Text < 

Sending message to subscriber topic from terminal
gz topic -t /<topic_name> -m gz.msgs.StringMsg -p 'data:"Text"'

<file_topic> Changes topic name to which File_with_Pose.txt information should be sent to. If not present the topic will pulblish to "/model/<model_name>/file_pos_contr"
Format > Text < 



-Use the following offsets to place the centre of the model
<xyz offset>  XYZ offset of pose. (Meters)
    Format > 0.0 0.0 0.0 < 
<rpy offset> Roll, Pitch, Yaw offset. (Radians)
    Format > 0.0 0.0 0.0 < 


## Terminal messages that might     be needed (Spesific to testing)
For both messages 'box' can be substituted to a different model name

-Send a Pose message
gz topic -t /box -m gz.msgs.StringMsg -p 'data:"0,0,2,0,0,0,1"'

-Get Pose of the model 
gz model -m "box" -p

-Send file with Pose messages
gz topic -t /file -m gz.msgs.StringMsg -p 'data:"/home/paul/Gz_Plugins/Position_Control/pose_file2.txt"'


## Terminal messages if no topic name provided (Generic)
gz topic -t /box/pos_contr -m gz.msgs.StringMsg -p 'data:"0,0,2,170,50,170,1"'
gz topic -t /box/file_pos_contr -m gz.msgs.StringMsg -p 'data:"filename"'

## SDF File -> Checklist

Ensure that link pose is set to <0 0 0 0 0 0> relative to the model pose
Ensure that mass is set to 1kg and Ix , Iy and Iz is also 1 (kg.m2)



## Other Messages
--Record Video
gz service -s /boxes_full_2d --timeout 2000 --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --req 'start:true, save_filename:"name.mp4"'
gz service -s /boxes_full_2d --timeout 2000 --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --req 'start:false'
--Run/Pause Simulation
gz service -s /world/boundingbox_sensor/control/state --reqtype  gz.msgs.WorldControlState --reptype gz.msgs.Boolean --timeout 2000 --req 'world_control: {pause: true}'



