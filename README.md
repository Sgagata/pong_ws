Steps to compile and run pong game (taken from the assignment descriprion and modifed)
As instructed by TA we aslo left in the .vscode folder so that the includes problems
could be resolved faster
The launch files are in bundle_launch directory, on the same level as src

1. Unzip in an empty directory <pong_ws>
2. Adjust SDL library path in src/pong_pkg/CMakeLists.txt
3. Go to <pong_ws>
4. colcon build
In case the build fails follow steps below:
-------------------------------------------------------------------------------------
5. In the same teminal:
cd src
git clone https://github.com/ros-perception/vision_opencv.git
colcon build --packages-select cv_bridge --cmake-force-configure
6. Delete the build folder, either manualy or in the same terminal:
cd ..
rm -r build
7. And build again
colcon build
--------------------------------------------------------------------------------------- 
The all the nodes for each assigmnent can be run all together using a launch file
8. In a new terminal
. <pong_ws>/install/setup.bash
cd <pong_ws>/bundle_launch/launch
9. Pick a correct launch file:
    6.2: camera_launch.launch.py
    6.3: pong_core_launch.launch.py
    6.4: viz_launch.launch.py
    6.5: keyboard_pong_launch.launch.py
    Set focus on the key_input window and control left bar with W, S keys; control right bar with I, K keys

For the 6.6 there are two options: both players use light, left player uses light and right one keyboard  (I, K keys)
light: pong_complete_launch.lauch.py
mixed: pong_play_launch.lauch.py
10. In the same terminal
ros2 launch <launch_file>