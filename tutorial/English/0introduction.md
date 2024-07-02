# Introduction

Hello everyone, I'm Harrison, and today I'm excited to start writing this educational project. Following this tutorial, you can build a simple autonomous driving system step by step. I will explain the code logic used in the project as thoroughly as possible, covering many professional terms you might have heard before, such as network communication, thread pools, path-finding algorithms, PID controllers, distributed systems, and more. Even if you don't have a C++ background, I believe you can benefit from this. I welcome feedback from fellow learners. Alright, let's begin.

# Simple Autonomous Driving System

First, let's review the architecture of autonomous driving system technology to understand what kind of business scenario our project aims to simulate. An autonomous driving system is installed in vehicles, and its core functions are threefold: Perception, Planning, and Control.

1. Perception: Collect environmental data through sensors (such as cameras, radar, lidar, etc.), detect and classify objects, and model the environment by combining various data.

2. Planning: Based on perception information, evaluate behavioral options to make decisions, and plan safe and effective paths.

3. Control: Execute the planned path and behavioral decisions by operating the vehicle through the vehicle control system (such as steering wheel, accelerator, brake).

In simple terms, perception is responsible for collecting and initially processing data, planning is responsible for path planning based on perception information, and control is responsible for the actual operation of the vehicle. Each function requires countless people to invest immense effort in development. The simple implementation in this educational project can be seen as a starting point for further exploration. Due to the lack of necessary hardware resources, we will use ROS2's built-in point cloud type to simulate data from lidar. Each autonomous vehicle typically communicates with a central server (also known as a cloud server) over a network; we will develop a simple server to simulate the presence of this central server. The tutorial will introduce their implementation processes in detail once we officially begin.

# Why Choose ROS2?

ROS2 is very suitable for beginners to get started with autonomous driving because it's easy to run a demo, and many startup companies also use ROS2 for experiments in their early stages. Additionally, many existing frameworks in the market are modified versions based on ROS2 (such as Baidu's Apollo).
From a job-seeking perspective, familiarity with ROS2 is a crucial skill for robotics and autonomous driving-related development.

# Environment Setup

We are developing under the `Ubuntu 22.04.4 LTS` and `ROS2 (Robot Operating System 2)` environment. For those who want to learn how to use the ROS2 basic framework, you can refer to the following resources (I will introduce the relevant content we use again in the explanation process, so friends who are not familiar with ROS2 don't need to feel anxious):

[ROS2 Basic](https://github.com/Erio-Harrison/ros2_basic)

[ROS2 from the Ground Up: Part 1](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-1-an-introduction-to-the-robot-operating-system-4c2065c5e032)

[ROS2 from the Ground Up: Part 2](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-2-publishers-and-subscribers-8deda54927c7)

[ROS2 from the Ground Up: Part 3](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-3-a-hands-on-guide-to-creating-custom-messages-and-turtlebot3-service-96a68df2e670)