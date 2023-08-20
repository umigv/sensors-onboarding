# Sensors Onboarding
Welcome to the sensors subteam! We work on integrating, configuring, and fusing all the different sensors of the robot together so that it can localize itself and understand its surroundings. You can think of sensors as the backbone or glue of the team—we help connect subsystems together and provide the crucial information necessary for them to function. In this onboarding project, you will get your local environment setup so you can work with our software stack, then you will get to work with real sensors on a project that may find its way onto our actual robot!

## Setup
Please follow the [environment setup instructions](https://github.com/umigv/environment) to get your local environment up and running, then clone this repository into the `ws/src` folder as follows:
```sh
cd [your environment folder]/ws/src
git clone --recursive https://github.com/umigv/sensors-onboarding.git
```

## Learning ROS
If you don't already know ROS or haven't used ROS2 yet, go through the beginner ROS tutorials linked [here](https://docs.ros.org/en/humble/Tutorials.html). You don't have to work through all of these, especially not step-by-step, but a once-over of the materials will help you understand a lot of the concepts that we will be working with in ROS like nodes, topics, publishers and subscribers, and more. The beginner tutorials (CLI Tools and Client Libraries) should be all you need to get started. You can skip **Beginner: CLI Tools/Configuring environment** since that has already been done for you in the Docker container. The ROS tutorial packages for **Beginner: Client Libraries** have already been cloned for you in this repository under `ros_tutorials`, so you don't need to clone them yourself. When you get to the Client Libraries section, you only need to familiarize yourself with the **C++/Python (will decide later)** documentation, since that is the language we have used for our packages. However, if you prefer the other language, feel free to learn how to use it as well and write your packages in the langauge you want.

## Hands-On Project
### Modifying an IMU Driver
Now that you're familiar with ROS, let's start working with some real sensors! In this project, you will be adding some features to a ROS2 driver for an IMU sensor. You will have the opportunity to test your modified driver on real hardware and hopefully gain  a good understanding of how we can use ROS to interact with sensors!

#### Overview
To start, open the `bno055` folder. This is a modified version of [this package](https://github.com/flynneva/bno055) that has had some features removed in order to serve for this project. If you ever get really stuck and can't figure something out, you can look at the original package for guidance, but before resorting to that please ask your fellow members or team leads for help, as they may be able to assist you better in actually learning rather than just copying the answer. You will be adding/fixing the following features to the driver:
- Accounting for gravity in the acceleration output as per [this ROS standard](https://www.ros.org/reps/rep-0145.html#data-sources)
    - This is necessary for many packages that work with IMUs (e.g. SLAM, localization) to operate properly and was a feature that was missing from previous drivers we've used on this team
- Add logging
- Account for measurements made by another sensor

#### Guidance
This section will walk you through the structure of the package and provide guidance for the various steps of the project for your reference. You can (and are encouraged) to try to tackle these challenges on your own first, but feel free to reference this section and your peers for help if you ever you get stuck.

##### Structure
- A sample launch file to start the driver is in `launch/bno055.launch`
- Other import package files include `setup.cfg`, `setup.py`, and `requirements.txt`. These files tell `colcon` where to find the code and how to build the package
- Main driver and communication code is housed in the `bno055` folder
    - Node definition and general runtime logic in `bno055.py`
    - Direct sensor interface code defined in `sensor/SensorService.py`
    - Helper functions for interfacing with sensor using different connection protocols defined in `connectors`
    - Other code defines parameters for the node and error handling logic
- We will mostly be making changes to ``bno055.py` and `sensor/SensorService.py`, but it is good to get familiar with the rest of the code as well to really understand how the drivers we use interact with the real hardware

##### Understanding Sensor Communication
In order to communicate with actual hardware sensors, there are multiple communications protocols that are commonly used. These protocols define how hardware components "talk" over wires, describing the exact timing, order, and meaning of the electrical signals sent over the set of wires connecting the devices. All together, these wires form a unit called a "bus", and include both signal and power wires. The protocols we will learn about here are digital, serial protocols. Digital means that messages are sent as a series of discrete values, in this case pulses on or off. Serial means that messages are sent one bit at a time, meaning that less wires are needed to transfer messages. For more information, check out [this page](https://learn.sparkfun.com/tutorials/serial-communication/all). This specific IMU driver has options to use both the [I2C](https://learn.sparkfun.com/tutorials/i2c/all) and [UART](https://www.embedded.com/understanding-the-uart/) protocols and provides abstractions to use either depending on your configuration. To actually connect to the sensors, a USB adapter is used to convert to the desired protocol. We won't get super deep into either of these protocols or the underlying serial communications details, but it's good to be aware of them and have a basic idea of what's going on under the hood.

##### The Project
For this project, we will work through adding some features to this IMU driver and fixing some bugs with it so you can gain a better understanding of how we work with sensors and how to debug issues with them. You will also get the opportunity to test your code on real hardware and see the impact in real time! After familiarizing yourself with the structure of the code, we will get started with the first task: accounting for gravity in the output acceleration.

###### Task 1: Gravity
ROS passes values like the acceleration of an object in a specific format called a messsage, with each message having a rigidly defined structure. The structure for an IMU acceleration message can be found [here](https://www.ros.org/reps/rep-0145.html).

### Todo
- Finish writing guide
    - Guide members to "discover" issues on their own by testing the sensor and finding issues
- Modify driver to remove features

### Ideas
- Add new feature to IMU driver
    - Could take some feature out like accounting for gravity and ask new members to add in
- Integrate encoders with IMU for localization
- Build towards teamwide goal like Jason mentioned
    - http://zlethic.com/diy-3dlidar/
    - https://github.com/Microsoft/AI-Robot-Challenge-Lab

## Timeline
- Week 1: Introductions, environment setup
- Week 2: Finish environment setup, start on ROS tutorials
- Week 3: Continue with ROS tutorials, introduce main onboarding project
- Week 4: Work time for both tutorials and onboarding project
- Week 5: Wrap up onboarding project and introduce robot goals and projects