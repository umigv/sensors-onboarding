# Sensors Onboarding

Welcome to the sensors subteam! We work on integrating, configuring, and fusing all the different sensors of the robot together so that it can localize itself and understand its surroundings. You can think of sensors as the backbone or glue of the team—we help connect subsystems together and provide the crucial information necessary for them to function. In this onboarding project, you will get your local environment setup so you can work with our software stack, learn ROS (the set of libraries and tools we use to run our robot), and get to work with real sensors on a project that should help give you an idea of what you will be doing on the team.

## Setup

Please follow the [environment setup instructions](https://github.com/umigv/environment) to get your local environment up and running, then clone this repository into the `ws/src` folder as follows:

```sh
cd [your environment folder]/ws/src
git clone --recursive https://github.com/umigv/sensors-onboarding.git
```

## Learning ROS

If you don't already know ROS or haven't used ROS 2 yet, go through the beginner ROS 2 tutorials linked [here](https://docs.ros.org/en/humble/Tutorials.html). You don't have to work through all of these (especially not step-by-step), but a once-over of the materials will help you understand a lot of the concepts that we will be working with in ROS like nodes, topics, publishers and subscribers, services, parameters, and more. The beginner tutorials (**CLI Tools** and **Client Libraries**) should be all you need to get started. You can skip **Beginner: CLI Tools/Configuring environment** since those steps have already been done for you in the Docker container. The ROS tutorial packages for **Beginner: Client Libraries** have also already been cloned for you in this repository under `ros_tutorials`, so you don't need to clone them yourself. When you get to the **Client Libraries** section, you only need to familiarize yourself with either the C++ or Python documentation (not both) based on your personal preference. However, you are welcome and encouraged to learn how to use the other language as well, since you are likely to use packages written in both languages.

### Tips

- Make sure you always run `colcon` commands from the root directory of your workspace, e.g. `~/ws`. Don't run them from subdirectories like `~/ws/src`!

## Timeline

- Week 1: Introductions, environment setup
- Week 2: Finish environment setup, start on ROS tutorials
- Week 3: Continue with ROS tutorials, introduce main onboarding project
- Week 4: Work time for both tutorials and onboarding project
- Week 5: Wrap up onboarding project and introduce robot goals and projects
