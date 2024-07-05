# Introduction

Hello everyone, I'm Harrison. Today I'm looking forward to writing this tutorial. Following this tutorial, you can build a simple autonomous driving system step by step. I will explain the code logic used in the project as detailed as possible, and will talk about many professional terms that you have heard of, such as network communication, thread pool, path finding algorithm and PID controller, distribution, etc. Even if you don't have a C++ foundation, I believe you can benefit from it. Friends who study together are welcome to give me suggestions. Okay, let's get started.

# Autonomous driving system architecture

First, let's sort out the architecture of the autonomous driving system technology and understand what kind of business scenarios our project is designed to simulate. The autonomous driving system is installed on the vehicle, and it has three core functions: perception, planning, and control.

1. **Perception**: Collect environmental data through sensors (such as cameras, radars, lidars, etc.), detect and classify objects, and combine various types of data to model the environment.

2. **Planning**: Based on the perception information, evaluate behavioral options to make decisions and plan safe and effective paths.

3. **Control**: Execute the planned path and behavior decisions, and operate the vehicle through the vehicle control system (such as steering wheel, accelerator, brake).

Simply put, perception is responsible for collecting and preliminarily processing data, planning is responsible for path planning based on perception information, and control is responsible for the actual operation of the vehicle. Each function requires countless people to invest countless efforts to develop, and the simple implementation in the teaching project can be said to be a starting point. Due to the lack of necessary hardware resources, we use the **point cloud type** built into **ROS2** to simulate the data from the lidar. Each driverless car usually communicates with a central server (also called a cloud server) over the network. We will complete the development of a simple server to simulate the existence of this central server. After the tutorial officially starts, we will introduce their implementation process in detail.

# Why choose ROS2 (Robot Operating System 2)?

**ROS2** is very suitable for beginners to get started with autonomous driving, because it is easy to run a demo, and most startups also use **ROS2** to run experiments in the early stages. In addition, many existing frameworks on the market are also based on **ROS2** (such as Baidu's Apollo).

From the perspective of finding a job, being familiar with **ROS2** is a very important skill for developing robots and autonomous driving.

# Environment configuration

We developed in **Ubuntu 22.04.4 LTS** and **ROS2** environment. Friends who want to know how to use the **ROS2** basic framework can refer to the following (I will introduce what we need to use again during the explanation, so friends who don’t know **ROS2** don’t need to worry):

[ROS2 Basic-Download link is also inside](https://github.com/Erio-Harrison/ros2_basic)

[ROS2 from the Ground Up: Part 1](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-1-an-introduction-to-the-robot-operating-system-4c2065c5e032)

[ROS2 from the Ground Up: Part 2](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-2-publishers-and-subscribers-8deda54927c7)

[ROS2 from the Ground Up: Part 3](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-3-a-hands-on-guide-to-creating-custom-messages-and-turtlebot3-service-96a68df2e670)