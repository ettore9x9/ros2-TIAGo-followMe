# Simulation for TIAGo robot tracking a moving object.
Assignment of SofAR class from Robotics Engineering course at Università degli Studi di Genova. The assignment regards the development of a software architecture in ROS 2 where, in a simulation launched on Webots, TIAGo robot (made by [@palrobotics](https://pal-robotics.com/robots/tiago)) follows a moving item seen by his RGB-D camera.

To complete the task, we decided to have a different approach to the problem. Using libraries of computer vision and simple PID controllers for the robot, we implemented a reasonable architecture where TIAGo follows a target and tries to keep it at the center of its vision.

## Installing the package.

First of all the simulation runs on ROS2 version foxy. To install the ROS2 look at the following [link](https://docs.ros.org/en/foxy/Installation.html). Then you have to build your workspace which can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

The approach to the project was computer vision like, so we had to decide wheter library of open source cv use, we decided to use [OpenCV](https://opencv.org/). To install OpenCV:
```bash
sudo apt update
sudo apt install python3-opencv
```
To develop a good and simple PID controller, we decided to use this [library](https://pypi.org/project/simple-pid/). To install it:
```bash
sudo apt update
sudo apt install python3-pip
pip install simple-pid
```
The simulation enviroment we decided to use is Webots for ROS2. Please keep in mind Webots works only on amd64 architectures. To install it:
```bash
wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
sudo apt-get update
deb https://cyberbotics.com/debian/ binary-amd64/
sudo apt-get update
sudo apt-get install webots
```

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
_Packages_: [__cv_basics__](https://github.com/ettore9x9/SOFAR_TIAGo/tree/main/cv_basics), [__tiagosim__](https://github.com/ettore9x9/SOFAR_TIAGo/tree/main/tiagosim), [__vision_opencv__](https://github.com/ettore9x9/SOFAR_TIAGo/tree/main/vision_opencv).

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

### Controlling the robot behaviour (PID control) with a simple idea of obstacle avoidance.

_Packages related_: [__tiagosim__](https://github.com/ettore9x9/SOFAR_TIAGo/tree/main/tiagosim), [Obstacle_Avoidance_ROS2](https://github.com/ettore9x9/SOFAR_TIAGo/tree/main/Obstacle_Avoidance_ROS2).

We want to keep the robot following a target. To do that, the best way to achieve the goal is controlling the robot behaviour with simple tasks. These tasks are:

- Keeping the centroid of the face on the center of the image (Setpoint = 2m).
- Keeping a fixed distance between the robot and the human (Setpoint = 320 px).

We configured the PID controllers with the PID library of python. As we had parameters to be configured and an important command (desired velocity) to be sent to the robot, we thought it was a good idea to use a _request-process-reply_ design pattern as it follows:

<p align="center">
<img src="https://s3.us-west-2.amazonaws.com/secure.notion-static.com/65081900-5dbe-4623-81b6-4ef7e01f2f54/Schermata_2022-07-02_alle_11.20.21.png?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Content-Sha256=UNSIGNED-PAYLOAD&X-Amz-Credential=AKIAT73L2G45EIPT3X45%2F20220702%2Fus-west-2%2Fs3%2Faws4_request&X-Amz-Date=20220702T092125Z&X-Amz-Expires=86400&X-Amz-Signature=1339dab22668388e15a69609b70ae79cf98d2d795d9d6a08332ec28ddb59bf74&X-Amz-SignedHeaders=host&response-content-disposition=filename%20%3D%22Schermata%25202022-07-02%2520alle%252011.20.21.png%22&x-id=GetObject" width="400" height="410" />
</p>

This becomes helpful when approaching different ideas for the obstacle avoidance, as it is an independent node where the code can be modified as the developer wishes. The pid controllers are two, one for the differential wheels mobility of the base and the other one for the distance, so for the linear mobility. We decided to use as gains (__Proportional, Integrative and Derivative__) the following values:
- Linear velocity control: K_p = -1, K_i = 0, K_d = -2.
- Angulaar velocity control: K_p = 0.004, K_i = 0, K_d = 0.0008.

All the values listed were assigned in an hempirical approach.

## UML Graphs.
