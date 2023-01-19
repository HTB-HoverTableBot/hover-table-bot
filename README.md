# HTB - HoverTableBot

## Melodic branch
Please check your ROS version

It's born from [roboka robot](https://github.com/alex-makarov/robaka-ros) with some [changes](https://github.com/HTB-HoverTableBot/hoverboard-driver).

## Files
Common ROS packages for HTB, useable for both simulation and real robot operation.

- htb_description: Robot description (URDF)
- htb_base : Reference to the hardware driver for communicating with the onboard MCU.
- htb_gazebo : Gazebo plugin definitions and extensions to the robot URDF.
- htb_navigation: Different slam samples
  - hector_slam_test.launch: [hector_slam](http://wiki.ros.org/hector_slam) sample with rplidar

## Extra information
- [Wiki](https://github.com/HTB-HoverTableBot/hover-table-bot/wiki) for more information.
- [webpage](https://htb-hovertablebot.github.io/)
- [Blog entry](https://martinnievas.com/myblog/2021-04-29-hoverboard-robot/) with some construction details.
