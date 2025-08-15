# Line by line ROS2 commands needed to Launch node, build, etc.

1. source src/docker_humble_jetson/attach.sh \
    a) If you are doing a clean build on a new docker you need to do this first otherwise the custom msg won't build. If you see an error message, it's normal and you can ignore it. 

3. colcon build --packages-select custom_msg \
    a) build custom msg first 

4. colcon build \
    a) build the project \
    b) for clean build do: rm -rf build/ install/ log/ \
    c) NEVER build in the src/ folder. The docker automatically goes into src/ so don't forget to do a cd .. to go back one. 

5. source src/docker_humble_jetson/attach.sh \
    a) Source again for ROS. Not neceassrily needed but sometimes it is and just good practice to avoid debugging for no reason 
   
6. ros2 launch avionics_nexus launch.py \
    a) launch avionics pipeline. Also launches the python node. \
    b) This is a specific command for the current code. General syntax is: ros2 launch *node_name* *launch_file*  

7. docker exec -it elec_humble_desktop bash \
    a) If you want multiple terminals, allows you to attach them to the same docker. 

8. ros2 topic pub /EL/servo_req custom_msg/msg/ServoRequest "{id: 1, increment: 50.0, zero_in: false}" \
    a) simulate publisher message from CS to see if your system works 

9. ros2 topic echo /EL/mass_topic \
    a_ allows you to see what the message received by the RP from the avionics 
