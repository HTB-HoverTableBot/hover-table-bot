# HTB - HoverTableBot

![hover-table-bot](https://user-images.githubusercontent.com/24465803/131233565-71bd79fa-e4e6-45d6-9c2b-892c30f467ae.png)

It's born from [roboka robot](https://github.com/alex-makarov/robaka-ros) with some [changes](https://github.com/MartinNievas/hoverboard-driver/tree/devel).


Check out the [Wiki](https://github.com/HTB-HoverTableBot/hover-table-bot/wiki) for more information.
[Blog entry](https://martinnievas.com/myblog/2021-04-29-hoverboard-robot/) with some construction details.


## Files
Common ROS packages for HTB, useable for both simulation and real robot operation.

- htb_description: Robot description (URDF)
- htb_base : Reference to the hardware driver for communicating with the onboard MCU.
- htb_gazebo : Gazebo plugin definitions and extensions to the robot URDF.
