# Logical camera plugin

The outputs of the logical camera is madel names and their poses with respect to the camera fram, a detailed description of how this plugin works in gazebo could be found [here](https://gazebosim.org/tutorials?tut=logical_camera_sensor&cat=sensors).

The implementation of this plugin is simple to get logical images from gazebo simulator and then wrap the data into ros topic. The topic is hardcoded for now under "/gazebo/logical_camera".
