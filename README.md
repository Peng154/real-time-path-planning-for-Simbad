# real-time-path-planning-for-Simbad
- the project is based on Simbad Platform(so you should import the simbad-1.4.jar first),actually it use the RT-RRT* algorithm to reach the goal of realtime path planning, also provide simple RRT demo and simple RRT* algorithm demo.
- there is still some problem existing during path planning when the moving obstacle is extremely close to the robot（when the obstacles block the root of the rrt tree!）. I think this situation may be related to the reason that the robot can not predict the moving status of the moving_obstacles.

![Simulation Result](https://raw.githubusercontent.com/Peng154/real-time-path-planning-for-Simbad/master/rt_rrt_star.gif)

# Reference
[1. Wikipedia RRT Explaination] (https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)

[2. rt_rrt* paper] (https://mediatech.aalto.fi/~phamalainen/FutureGameAnimation/p113-naderi.pdf)

[3. simbad paltform doc] (http://simbad.sourceforge.net/doc.php)
