# Sensors Onboarding 2024

Welcome to the sensors subteam! We work on integrating, configuring, and fusing all the different sensors of the robot together so that it can localize itself and understand its surroundings. You can think of sensors as the backbone or glue of the team—we help connect subsystems together and provide the crucial information necessary for them to function. In this onboarding project, you will set up your local environment so you can work with our software stack, learn ROS (the set of libraries and tools we use to run our robot), and work on a project that involves real sensors, giving you a hands-on experience of what we do on the team.

## Environment Setup
You will be using pre-configured Ubuntu 22.04 virtual machines for your ROS environment. First, you need to download the virtualization software required to run these virtual machines, VMware Workstation Pro for Windows and VMware Fusion Pro for Mac. 

### Windows Instructions
- Download and run the installer found in the folder at the following link: [Workstation Pro Installer](https://drive.google.com/drive/u/1/folders/1B4brWb8zgHHhMUmMF9D1ZJPqDI5njiof)
    - Note: if you get the error "Can't scan for viruses", thats normal in this case, just press "Download anyway"
- When prompted, select “Install Windows Hypervisor Platform (WHP) automatically”
- Leave all other settings as the default
- Download and unzip the virtual machine found in the folder at the following link: [Virtual Machine](https://drive.google.com/drive/u/1/folders/1MHMFtIlwFGJZG5i-9k_x_0I_77PrtI__)
  - Note: the virtual machine itself will be a folder named "ARV ROS2 Humble AMD64 VM", this will take a while to download, so please be patient
- Launch the VMware Workstation Pro application
- Select "Use VMware Workstation 17 for Personal Use"
- Select "Open a Virtual Machine"
- In the file explorer, navigate inside the unzipped virtual machine folder you downloaded and select the "VMware virtual machine configuration" file found inside
- This will open the pre-configured virtual machine, where a ROS workspace is already installed and ready to go
- The password to login is "arv123"

### Mac Instructions
- Download and run the installer found in the folder at the following link: [Fusion Pro Installer](https://drive.google.com/drive/u/1/folders/17qI6loxY2wwvchc0UcinaLhVEvrYvMe1)
- Download and unzip the virtual machine found in the folder at the following link: [Virtual Machine](https://drive.google.com/drive/u/1/folders/1AtrTcFdAR8XJhJ1agBp9nuoueDqO_DnV)
- Launch the VMware Fusion Pro application
- When prompted for a license key, select "I want to license VMware Fusion 13 Pro for Personal Use"
- In the upper menu, select File->Open and Run, and select the virtual machine you downloaded
- This will open the pre-configured virtual machine, where a ROS workspace is already installed and ready to go
- The password to login is "arv123"

### VMware vs Docker

We strongly encourage using **VMware** for your ROS2 development environment. VMware tends to provide better performance and hardware compatibility, especially when dealing with USB passthrough for sensors. However, Docker is still a viable option if you prefer containerized environments. -- I have been using Docker containers and haven't had issues so far. For those interested in Docker, check out the [Docker setup guide](https://docs.docker.com/get-started/).

---


## Learning ROS

If you aren't familiar with ROS2, start by reviewing the beginner ROS2 tutorials linked [here](https://docs.ros.org/en/humble/Tutorials.html). You don't need to go through every single tutorial step-by-step, but focusing on topics like **CLI Tools** and **Client Libraries** will help you grasp essential concepts like nodes, topics, publishers and subscribers, services, and parameters. For this onboarding, you can skip the **CLI Tools/Configuring environment** tutorial since those steps have been pre-configured in your VM or Docker container. You will only need to familiarize yourself with either the Python or C++ client libraries, depending on your preference, but you are encouraged to learn both for future flexibility.



## Hands-On Project - Modifying an IMU Driver

Now that you're familiar with ROS, let's start working with some real sensors! In this project, you will be adding some features to a ROS2 driver for an IMU sensor. You will have the opportunity to test your modified driver on real hardware and hopefully gain  a good understanding of how we can use ROS to interact with sensors! Make sure to check out the [tips](#tips) section below for some best practices.

### Overview

To start, open the `bno055` folder. This is a modified version of [this package](https://github.com/flynneva/bno055) that has had some features removed in order to serve for this project. If you ever get really stuck and can't figure something out, you can look at the original package for guidance, but before resorting to that please ask your fellow members or team leads for help, as they may be able to assist you better in actually learning rather than just copying the answer. You will be adding/fixing the following features to the driver:

- Accounting for gravity in the acceleration output as per [this ROS standard](https://www.ros.org/reps/rep-0145.html#data-sources)
  - This is necessary for many packages that work with IMUs (e.g. SLAM, localization) to operate properly and was a feature that was missing from previous drivers we've used on this team
- Add additional logging

### Guidance

This section will walk you through the structure of the package and provide guidance for the various steps of the project for your reference. You can (and are encouraged) to try to tackle these challenges on your own first, but feel free to reference this section and your peers for help if you ever you get stuck.

#### Structure

- A sample launch file to start the driver is in `launch/bno055.launch`
- Other import package files include `setup.cfg`, `setup.py`, and `requirements.txt`. These files tell `colcon` where to find the code and how to build the package
- Main driver and communication code is housed in the `bno055` folder
  - Node definition and general runtime logic in `bno055.py`
  - Direct sensor interface code defined in `sensor/SensorService.py`
  - Helper functions for interfacing with sensor using different connection protocols defined in `connectors`
  - Other code defines parameters for the node and error handling logic
- We will mostly be making changes to `bno055.py` and `sensor/SensorService.py`, but it is good to get familiar with the rest of the code as well to really understand how the drivers we use interact with the real hardware

#### Understanding Sensor Communication

In order to communicate with actual hardware sensors, there are multiple communications protocols that are commonly used. These protocols define how hardware components "talk" over wires, describing the exact timing, order, and meaning of the electrical signals sent over the set of wires connecting the devices. All together, these wires form a unit called a "bus", and include both signal and power wires. The protocols we will learn about here are digital, serial protocols. Digital means that messages are sent as a series of discrete values, in this case pulses on or off. Serial means that messages are sent one bit at a time, meaning that less wires are needed to transfer messages. For more information, check out [this page](https://learn.sparkfun.com/tutorials/serial-communication/all). This specific IMU driver has options to use both the [I2C](https://learn.sparkfun.com/tutorials/i2c/all) and [UART](https://www.embedded.com/understanding-the-uart/) protocols and provides abstractions to use either depending on your configuration. To actually connect to the sensors, a USB adapter is used to convert to the desired protocol. We won't get super deep into either of these protocols or the underlying serial communications details, but it's good to be aware of them and have a basic idea of what's going on under the hood.

#### The Project

For this project, we will work through adding some features to this IMU driver and fixing some bugs with it so you can gain a better understanding of how we work with sensors and how to debug issues with them. You will also get the opportunity to test your code on real hardware and see the impact in real time! After familiarizing yourself with the structure of the code, we will get started with the first task: accounting for gravity in the output acceleration.

##### Task 1: Gravity

ROS passes values like the acceleration of an object in a specific format called a messsage, with each message having a rigidly defined structure. The structure for an IMU acceleration message can be found [here](https://www.ros.org/reps/rep-0145.html). The important part here can be found under `Data Sources`, where it says that accelerometers should report a vector corresponding to the acceleration due to gravity in the z axis while the device is at rest. This is not currently accounted for in our BNO055 driver's IMU output, but we would like to account for it because certain SLAM algorithms expect the absolute gravity reference for localization. Therefore, your first task is to modify the IMU driver to add the acceleration due to gravity to the IMU output. There are already gravity registers available for you to use in `bno055/registers.py`. Try to figure out how to read the values from these registers and incorporate them into the IMU message output published on `/bno055/imu`.

Note: this driver already does correctly incorporate gravity into `/bno055/imu_raw`. It does this through a direct interface with a sensor register that does this for you, but for the purpose of this exercise you can just ignore that part and add the gravity accleration to `/bno055/imu` directly. We won't actually be using this code, but it's good to get a feel for how we can interface with the IMU ourselves and add new/missing features to pre-existing drivers.

##### Task 1 Hints

- If you launch the IMU driver with an IMU plugged in using `ros2 launch bno055 bno055.launch.py` and then view the output on the IMU topic using `ros2 topic echo /bno055/imu`, you can see under the `linear_acceleration` section that all values are near 0 when the sensor is at rest, meaning that gravity is not factored into the data. If you run `ros2 topic echo /bno055/imu_raw`, you can see our desired output, which includes the absolute gravity acceleration while the sensor is at rest.
- Ignore the lines in `sensor/SensorService.py` that refer to `imu_msg_raw`. We are only interested in modifying `imu_msg` in this exercise. When using this driver on our actual robot, we will probably just use the `imu_raw` topic when we need it, but for this exercise we want to emulate its functionality ourselves.
- A very useful resource to learn about a particular sensor is its datasheet. When working with a new sensor, it's always a good idea to find its datasheet and reference it to learn about how a sensor represents and reports the data it collects. In this case, the IMU sensor we are using is a BNO055 from Bosch, and we can find its datasheet [here](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf). An important area of interest is section **3.6.5 Output data registers** starting on page 34, where the registers that our data is stored in are listed along with their properties and sizes.
- Notice that in line 148 of `sensor/SensorService.py`, the driver calls `self.con.receive()` with a register argument of `registers.BNO055_ACCEL_DATA_X_LSB_ADDR` and a length argument of `45`. This line is calling a library function to read 45 bytes of data from the IMU registers starting at the memory address specified by `registers.BNO055_ACCEL_DATA_X_LSB_ADDR`.
  - You can think of the IMU registers as a linear array of memory. If you check `registers.py`, you can see the layout of these registers in memory. For example, `BNO055_ADDRESS_A` is at memory address `0x28`, `BNO055_ADDRESS_B` is at address `0x29`, and so on. You can think of these addresses as indices into the register memory array. They are encoded in hexadecimal, which is why they each start with `0x` and have letters as well as numbers. If you are interested in learning more, feel free to google hexadecimal notation and why computers use it, but for now the exact reason or meaning of the numbers isn't important. The driver we are using already has the memory addresses labeled, so we don't have to worry much about their actual values.
- Each register contains a value that is 1 byte in size, so by reading 45 registers, the driver is actually reading 45 different registers in a row into one buffer!
  - The memory address (or "index") given by `registers.BNO055_ACCEL_DATA_X_LSB_ADDR` is just a starting point, and the driver is actually reading all the registers in a row from `BNO055_ACCEL_DATA_X_LSB_ADDR` to `BNO055_TEMP_ADDR`.
  - This driver code is not documented very well to represent this, and one interesting exercise would be to redesign the code and data structures so that this behavior is more clear, either through changing the buffer structure to incorporate names or tags somehow or adding comments describing each memory location. When writing your own code in the future, be sure to think about how to make it readable and understandable by other people!
- In the code following the `receive()` call, the driver transforms individual bytes or pairs of bytes into the necessary forms needed for each part of the standard IMU message specified [here](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg). For example, by cross-referencing the code in `sensor/SensorService.py` with the register listings in `registers.py`, we can see that lines 208 and 209 of `sensor/SensorService.py` take the data from registers `LINEAR_ACCEL_DATA_X_LSB` and `LINEAR_ACCEL_DATA_X_MSB`, pack them into a float, scale them by an appropriate value so they are represented in the correct units, and set `imu_msg.linear_acceleration.x` to the resulting value.
  - A lot of important documentation on ROS packages can be found [here](https://index.ros.org/), including the IMU message documentation linked above. You will want to reference this site often, and especially with the sections on [sensor messages](https://index.ros.org/p/sensor_msgs/) and [geometry messages](https://index.ros.org/p/geometry_msgs/) for this subteam.
- We can see in `registers.py` that there are registers that just hold the gravity data. How can we access the data from these registers and add them to the linear acceleration values in `imu_msg`?
  - Hint: you don't need to call `self.con.receive()` again—the data we want is already in `buf`.

##### Task 2: Linear Velocity and Filters

This task is a bit more free-form, so feel free to implement it however you like and play around with the features you add. The goal of this task is to get you more familiar with ROS topics, messages, subscribers, and logging. We would like to add some additional logging to the IMU driver when certain events occur. The exact events you log will be up to you, but here are some ideas to get you started (these are ordered in terms of expected difficulty but this may not be accurate):

- Periodically log the current velocity (not acceleration!) and orientation of the IMU
- Log a warning whenever the IMU experiences a sudden rapid acceleration (e.g. a collision)
- Apply simple filters such as a [low-pass filter](https://en.wikipedia.org/wiki/Low-pass_filter) or some other filter to try reducing drift
Extra Challenge
- If you're up for a challenge try to compute velocity using various numerical integration algorithms such as [Verlet](https://en.wikipedia.org/wiki/Verlet_integration). Think about what registers you need to compute the velocity via Verlet Integration.
- If you're up for more challenges, you can try to filter out the noise/IMU drift via a [Kalman Filter](https://en.wikipedia.org/wiki/Verlet_integration)
  - I can provide tutorials if you are interested in implementing Kalman Filters. Fun Fact: A simplified version of Kalman Filters is Project 1 of ROB 101! 

We will do this by adding a new ROS subscriber to the driver that will subscribe to the `/bno055/imu` or `/bno055/imu_raw` topic (your choice, if you haven't implemented task 1, `imu_raw` includes gravity in the acceleration message which may be useful to you if you aren't familiar with [quaternions](https://wiki.ogre3d.org/Quaternion+and+Rotation+Primer), which is the format for orientation in ROS, although it is good to learn about quaternions for future work in sensors and robotics). This subscriber will gather the data it needs to log what you choose and then use the ROS logging API to print messages when needed.

##### Task 2 Hints

- You will need to set up a new subscriber and intialize it in the `main` function of `bno055.py`. Check out [this tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node) on how to do so if you are need a refresher.
  - I recommend creating your subscriber in a separate file and importing it into `bno055.py` in order to keep your concerns separate. Analyze how `bno055.py` imports modules like `SensorService` for an idea of how to do this.
    - Note: adding another subscriber to the driver that subscribes to the driver's output is definitely not best practice, I just chose this because it's simpler and easier to implement for newcomers to ROS. The real best way to do this would be to create and launch a completely separate node that subscribes to `/bno055/imu` or `/bno055/imu_raw` and monitors it, and you are welcome and even encouraged to do the exercise this way in order to get more practice with ROS nodes.
- Information on ROS topics gets sent as individual messages with instantaneous data describing the robot's state at a certain interval (rate). This driver actually does not have a fixed rate of publishing, which makes it more difficult to do  calculations based on the rate at which messages are received. Think about how you can get around this limitation (hint: ROS allows you to get the current time with `my_node.get_clock().now()`, where `my_node` is the ROS node object you create)
- Think about what data you need to store about past IMU messages in order to log the events you want. What data structures will you need to store them? How often do you need to do this?
- For help with logging in ROS, check out [this article](http://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html).

## Tips

- Make sure you always run `colcon` commands from the root directory of your workspace, e.g. `~/ws`. Don't run them from subdirectories like `~/ws/src`!
- To test with RViz, make sure you set the fixed frame to `bno055` in the RViz settings
- If you are on Mac, you probably won't be able to test your code by plugging in an IMU directly since Mac doesn't allow USB passthrough to Docker containers eassily. When you need to test your code, push your code to a fork and test it on a team computer running Linux as detailed in the following section

## Running on Team Computers

If you are on Mac or just want to test with a more robust native Linux/ROS setup, follow the instructions below to get your code onto a team computer:

- Run everything below **outside of your ROS container** but in your Unix environment (e.g. inside WSL on Windows, inside your reguolar terminal on Mac)
- If you don't already have SSH keys set one up by following [these instructions](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)
- On Github, fork the `sensors-onboarding` repo by clicking the Fork button in the upper right corner
- Once you've forked the repository, click the green Code dropdown and copy the **SSH link** to your forked repository (should look like `git@github.com:username/sensors-onboarding`)
- Open your `sensors-onboarding` repo on your local machine and enter the following commands:
  - `git remote remove origin`
  - `git remote add origin [link to your fork here]`
  - `git remote add upstream https://github.com/umigv/sensors-onboarding.git`
- Fetch the new origin data with `git fetch origin`
- Set your `main` branch to track your fork instead of the original repository with `git branch main -u origin/main`
- Push your code with `git push`
- On the team laptop, pull your code into the workspace inside the folder with your name in `~/sensors` (e.g. `cd ~/sensors/[your name here]/ws/src`) with `git clone [link to your fork here]`
- You can use the ROS helper functions I've written as follows:
  - `rosup [your name here]` - installs the dependencies needed for your ROS workspace (will need a password that one of the leads can tell you)
  - `rosbuild [your name here]` - builds your ROS workspace
  - `rossrc [your name here]` - sources `install/setup.bash` in your ROS workspace
- For more information on forks, check [this page](https://docs.github.com/en/get-started/quickstart/fork-a-repo)

## Timeline

- Week 1: Introductions, environment setup
- Week 2: Finish environment setup, start on ROS tutorials
- Week 3: Continue with ROS tutorials, introduce main onboarding project
- Week 4: Work time for both tutorials and onboarding project
- Week 5: Wrap up onboarding project and introduce robot goals and projects
