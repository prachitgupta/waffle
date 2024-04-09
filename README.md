# waffle
deep_yard navigation
Steps

add these files in your package
you might have to add some dependencies in your package.xml from mine

some changes need to be made in cmake.txt for custom message Tupla
after building sourcing ws
export GAZEBO_MODEL_PATH="path_YOUR_package"/models/trees:$GAZEBO_MODEL_PATH

roslaunch waffle world.launch  => launches trees
roslaunch waffle spawn.launch => spawn waffle

depth control
rosrun waffle only_depth.py
rosrun waffle lower_control.py

ml predictions and visualization
rosrun waffle ml.pt
ToDo : add custom loss and class defination in script and validate predictions

combined
rosrun waffle navigation.py
rosrun waffle lower_control.py

