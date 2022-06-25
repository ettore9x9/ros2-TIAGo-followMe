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


### Finding a simulation.
_Packages_: [__tiagosim__](https://github.com/ettore9x9/SOFAR_TIAGo/tree/main/tiagosim)

First of all we had to find a simulation in ROS2 with TIAGo where we could move him in the space. We found a good solution looking at the TIAGo Iron docs on ROS2 [here](https://cyberbotics.com/doc/guide/tiago-iron). 
The simulation is a room containing different obstacles to train the robot.

### Creating the computer vision and image processing module.
_Packages_: [__cv_basics__](https://github.com/ettore9x9/SOFAR_TIAGo/tree/main/cv_basics), [__tiagosim__](https://github.com/ettore9x9/SOFAR_TIAGo/tree/main/tiagosim).

We wanted this assignment not to work with just only one kind of item, but everything that could be seen and recognised by the [openCV](https://opencv.org/) library in python. As we wanted to make a forward step in our development, we decided to use training models of human people (which can be found on GitHub). Please, remember that every model you gave him as input in the `webcam_sub.py` can be changeable. This means that every traing model .xml can work! 

We used primarly two different models: 
- Frontal Human Face `./src/cv_basics/cv_basics/haarcascade_frontalface_alt2.xml`
- Profile Human Face `./src/cv_basics/cv_basics/haarcascade_profile.xml`

As we wanted the possibility to switch betweeen profile and frontal face when the robot was moving, which makes the robot recognization of the face way smoother:
```python
faces = self.face_cascade_front.detectMultiScale(gray, 1.1, 4)
if len(faces) == 0 :
      faces = self.face_cascade_profile.detectMultiScale(gray, 1.1, 4)
```
Once we have computed the centroid of the rectangle where the face is detected, we decided to publish a _PointStamped_ message with the centroid and it's relative image timestamp (header). This is important because when `depth_finder.py` node will receive the depth information, it will have to combine the image to the right depth (f_datadepth is higher than f_datacentroid because of the image processing), which is our case.
