## Webots + NAO Robot + ROS

1. Download Webots from this link:
   https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb

2. Follow the installation guide for Webots at https://cyberbotics.com/doc/guide/installation-procedure

3. Install webots_ros by using the command:
$ sudo apt-get install ros-noetic-webots-ros

4. Modify one of the launch files from webots_ros package:
$ cd /opt/ros/noetic/share/webots_ros/launch/ $ sudo cp complete_test.launch complete_test.launch.bkup 
$ sudo gedit complete_test.launch

5. In this launch file, replace the world directory with the one from world folder 
NAO robot demo:
<arg name="world" value="/usr/local/webots/projects/robots/softbank/nao/worlds/nao_demo.wbt"/>

6.Download nao_demo_python.py from this repository and replace the original:
$ cd /usr/local/webots/projects/robots/softbank/nao/controllers/nao_demo_python/
$ sudo cp nao_demo_python.py nao_demo_python.py.bkup

Example Command Format for Moving a File:
$ sudo cp /path/to/source_directory/nao_demo_python.py /usr/local/webots/projects/robots/softbank/nao/controllers/nao_demo_python/

7.Set the WEBOTS_HOME environment variable:
$ export WEBOTS_HOME=/usr/local/webots

8.Launch the simulation:
$ roslaunch webots_ros complete_test.launch

9.In the simulator, select NAO from the robot description, then switch the controller to nao_demo_python.
Your robot is now running a simple generic code integrated with ROS.




