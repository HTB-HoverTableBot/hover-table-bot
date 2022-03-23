# HTB - HoverTableBot

![hover-table-bot](https://user-images.githubusercontent.com/24465803/154068083-3606a0f5-c1e1-4f49-b3c1-c46fb2b3c105.jpg)

It's born from [roboka robot](https://github.com/alex-makarov/robaka-ros) with some [changes](https://github.com/HTB-HoverTableBot/hoverboard-driver).


[webpage](https://htb-hovertablebot.github.io/) is live!

Check out the [Wiki](https://github.com/HTB-HoverTableBot/hover-table-bot/wiki) for more information.
[Blog entry](https://martinnievas.com/myblog/2021-04-29-hoverboard-robot/) with some construction details.


## Files
Common ROS packages for HTB, useable for both simulation and real robot operation.

- htb_description: Robot description (URDF)
- htb_base : Reference to the hardware driver for communicating with the onboard MCU.
- htb_gazebo : Gazebo plugin definitions and extensions to the robot URDF.
- htb_navigation: Different slam samples
  - hector_slam_test.launch: [hector_slam](http://wiki.ros.org/hector_slam) sample with rplidar
