Readme
=============

This repository consists of custom [Player](http://en.wikipedia.org/wiki/Player_Project) plugins, released under the GNU Lesser General Public License (LGPL). Plugins extend the functionality and capability of Player.

Player is a free robotics framework that is often used in robotics and sensor systems research. It uses a server-client model. Sensor 'drivers' are loaded by the server and then are read and manuipulated by the client program. Why all this hassle? Well, by doing so it lends itself to [distributed control](http://en.wikipedia.org/wiki/Distributed_control_system) schemes. Perhaps a full blown PC is too big / draws too much power for your robot to have onboard? Moreover, it makes the top level (client) program hardware-independent. That's achieved by one of the several levels of abstractions in Player. For more information on Player, see the [The Player Project](http://playerstage.sourceforge.net/) website.

Another huge benefit of using a robotics framework is that they often come with robot simulators. The simulators for Player are [Stage](http://playerstage.sourceforge.net/index.php?src=stage) and [Gazebo](http://gazebosim.org/).

About these plugins
------------

Many of these Player plugins were developed during the course of my [Masters project](http://robotang.co.nz/miscellaneous/my-masters/). I have also written a brief description about each of the plugins in this repository on my website, which can be found [here](http://robotang.co.nz/projects/robotics/custom-player-plugins/).

Misc
------------

Having trouble installing Player? Building large open-source projects (such as Player) from source can be challenging. Fortunately, there are several guides on the web how to do this. I have also written a guide too. You can find my [Install Player-Stage-Gazebo guide](http://robotang.co.nz/how-to/various/install-psg/) on my website.

Also be sure to check out my [website](http://robotang.co.nz) for other cool projects.

Robert Tang <opensource@robotang.co.nz>
