# Simulation for TIAGo robot tracking a moving object.
Assignment of SofAR class from Robotics Engineering course at Università degli Studi di Genova. The assignment regards the development of a software architecture in ROS 2 where, in a simulation launched on Webots, TIAGo robot (made by [@palrobotics](https://pal-robotics.com/robots/tiago)) follows a moving item seen by his RGB-D camera.

To complete the task, we decided to have a different approach to the problem. Using libraries of computer vision and simple PID controllers for the robot, we implemented a reasonable architecture where TIAGo follows a target and tries to keep it at the center of its vision.

## Installing the package.

## Our approach to the problem.
First of all, we decided to split the problem in main aspects, trying to schedule the macro problem in some smaller problems.
1. Finding an initial compliance __simulation__ where TIAGo can move in a space.
2. Creating a package to __get the rectangle coordinates (and its centroid's ones)__ on the image where the object seems to be. This means getting the dinstances too.
3. __Controlling the robot behaviour__ (mobile base and head) to respect the requirements we gave him (distance less than a treshold) with a PID controller.
4. Adding a simple module of __obstacle avoidance__ which can even be improved.

Now we will explain how we manage to develop each point. 

#
