# Keyboard_input node

## Installation, building the node
Building the node goes just as any other ROS2 node:

* Copy this directory into the (top level) ``src`` directory of your ROS2 workspace
* Go to the top level ROS2 workspace
* type ``colcon build --packages-select keyboard_input`` to build the node

## Running the node
Running the node goes just as any other ROS2 node:

* In a new terminal, source the workspace directory: go to the top level ROS2 workspace and type ``. install/local_setup.sh``
* Run the node with ``ros2 run keyboard_input keyboard_input``

## Notes
* Every time a key is pressed, this node outputs a message on the ``/keyboard_input/key`` topic, containing the ASCII code of the key you pressed. If you hold the key for a longer time, it starts repeating sending messages (just like you're used to). See also below.
No message is sent when the key is released (instead; the stream of repeating messages simply stops).
* Pressing Ctrl-C stops the node.
* To test how the node works, consider running ``ros2 topic echo /keyboard_input/key`` in a separate terminal to see the sent messages.
* To find out which value is sent by a specific key, run the above command.
* Arrow keys behave differently: they consist of three consecutive messages. For example, the UP arrow key sends the values (27, 91, 65). Note that these values are not unique to the UP arrow key. Specifically, these values are also sent by the ESC button (27), the '[' (91) and the shift-A (65). It is hard to distinguish between these, therefore it is advised to *not* use the arrow keys.
* The same holds for various other `weird' keys, such as Home, Insert, PgUp etc. Just try which keys work and which do not work.


* For playing Pong, it may be annoying that, after the first key-press, it takes a while (500 ms by default) before the key starts repeating itself. Fortunately, this delay (as well as the repeating rate, i.e., how many messages are sent when the key repeats) can be adjusted.
  * In a terminal, type ``xset r rate <delay_ms> <rate>``, e.g., ``xset r rate 50 40`` to set the repeat-delay to 50 ms and a repeat rate of 40 characters per second. To reset to the default values, type ``xset r rate 500 33``.
  *  This is a system-wide setting, so if you adjust it, this also holds for e.g. Visual Studio Code.
  * After a reboot the customized settings are forgotten and you're back to 500 ms and 33 characters/seconds.
  * The default Terminal program is immune for this setting. This means that:
    1. You can not use the Terminal program to run the keyboard node in (well, it works but you do not benefit from the improved response after using ``xset``).
    2. You thus should use a different terminal program. Install `xterm` (``sudo apt install xterm``) and use that one instead (start it from the start menu or t ype ``xterm`` in the default Terminal program).
    3. You *can* use the Terminal program to reset the delay and rate to the default values with ``xset`` (which may be hard to do from another program because if the delay is <150 ms, it is hard to type a single character).

* Using this node in a Launch file is tricky. The problem is that any key presses done in the Terminal where you started the launch file are *not* sent to the keyboard_input node (instead, they are just discarded). A solution (found on  https://answers.ros.org/question/398388/capture-stdin-using-ros2-launch/) is to make the specific node run in a separate terminal. Conveniently, this can be an `xterm`, so that part of the responsiveness problem is automatically  solved as well. This can be done by adding `prefix='xterm -e',` to the arguments of the Node function in the launch file. See also the launch file in the provided node.
  * You need to give focus to the xterm window before it receives key events.
  * In order to close the node, either:
      * Press Ctrl-C inside the xterm window. This sends a polite close request to the keyboard_input program, which then removes the created node and topic. 
      * Or, Press Ctrl-C in the Launch window. This does *not* send a polite close request to the keyboard_input program. Instead, it immediately destroys it, so the keyboard_input program will have no time to remove the node and topic. They are automatically removed by the ROS2 daemon after about 20 seconds.
      
      