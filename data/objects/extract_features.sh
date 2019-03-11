. /opt/ros/indigo/setup.bash
. ~/catkin_ws/devel/setup.bash

rm -rf combined_labels
mkdir combined_labels
rosrun perception extract_features_main blue_cup.bag blue_cup
rosrun perception extract_features_main expo.bag expo
rosrun perception extract_features_main galaxy_nexus.bag galaxy_nexus
rosrun perception extract_features_main orange_dog_toy.bag orange_dog_toy
rosrun perception extract_features_main red_cup.bag red_cup
rosrun perception extract_features_main turtle.bag turtle
rosrun perception extract_features_main yellow_dog_toy.bag yellow_dog_toy
mv *_label.bag combined_labels
