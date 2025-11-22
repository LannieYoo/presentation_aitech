# CST8504 ROS2 Vision Presentation Site

Project title

ROS2 vision with hand control for turtlesim

Course

CST8504 Robot Programming with ROS2

Team

Jiaxing YI  041127158

Hye Ran Yoo  041145212

Joseph Weng  041076091

Peng Wang  041107730

Target

Short presentation site for future AISD students who will do the ROS2 and vision labs

Total menus

5 pages

Page list

1 Intro

2 ROS2 and system setup

3 Camera and hand control for turtlesim

4 Network and performance issues

5 Conclusion and advice

## Global layout idea for Cursor

Main navigation

Intro

ROS2 and setup

Camera and turtlesim

Network and performance

Conclusion and advice

Design ideas

Simple responsive layout

Top navigation with the 5 menu items

Each page has

* Page title
* Presenter name and student number
* Short text blocks
* A small list of key points

Optional elements

* Small simple diagrams using div blocks
* Very short code blocks for main ros2 commands

## Page 1  Intro

Menu title

Intro

Route

/

Presenter

Jiaxing YI  041127158

Page title

ROS2 vision project overview

Purpose of this page

Welcome the audience

Explain the goal of the assignment

Give a high level roadmap of the next pages

Section 1  Context of the course

We study ROS2 in the AISD program.

The labs and the assignment ask us to connect ROS2 with vision.

First we work with turtlesim and hand tracking.

Later students can extend the same idea to the Create3 robot.

Section 2  Goal of our project

Use a camera and MediaPipe hands to control turtlesim.

Understand how ROS2 nodes, topics, publishers, and subscribers work together.

Learn from real environment problems such as wifi setup, camera support, and network delay.

Prepare useful tips for future students who will do the same labs.

Section 3  Simple overview of the system

A ROS2 node reads camera frames.

MediaPipe hands detects hand landmarks.

Another ROS2 node sends motion commands to turtlesim.

All messages move over topics inside ROS2.

Short roadmap for the next pages

Page 2  ROS2 and basic setup problems

Page 3  Camera and hand control flow

Page 4  Network and performance issues in the lab

Page 5  What we learned and advice for AISD classmates

## Page 2  ROS2 and system setup

Menu title

ROS2 and system setup

Route

/ros2-setup

Presenter

Hye Ran Yoo  041145212

Page title

First steps with ROS2 and system setup

Key story focus

First contact with ROS2 concepts.

Connection between local laptop, remote server, and robot.

Wifi and basic setup problems with the loaner Linux server.

Section 1  First contact with ROS2

ROS2 was a new concept for our team.

We needed to understand that ROS2 runs as nodes that talk over topics.

Our laptops use an ssh client such as Putty to connect to a Linux server.

The server runs the ROS2 nodes that control turtlesim, and later can control Create3.

Short explanation of core terms

Node  a running program in ROS2

Publisher  a node that sends messages

Subscriber  a node that receives messages

Topic  a channel that connects publishers and subscribers

Section 2  Loaner Linux server and wifi problem

At the start we received a Linux loaner server for the assignment.

Wifi was not working on that machine.

Because of that it was hard to install packages and run system updates.

We spent time to set up wifi first before we could do ROS2 work.

After wifi setup we could finally run basic ROS2 commands.

Example setup steps

Get the loaner server from the lab.

Test wifi and see that there is no connection.

Configure wifi and confirm internet access.

Run simple ros2 commands such as

* ros2 node list
* ros2 topic list

Section 3  Our strategy for this assignment

Focus on getting a stable environment before writing complex code.

Use a simple but working hand controlled turtlesim as our main demo.

Document each problem and solution so that future students can follow the same steps.

Keep the code small and clear instead of adding many features.

Section 4  Understanding the ROS2 structure

Laptop runs Putty or another ssh client.

Putty connects to the loaner Linux server.

The loaner server runs ROS2 nodes for camera, MediaPipe hands, and turtlesim control.

Messages move over topics between these nodes.

Suggested layout for Cursor

Left side  short explanation paragraphs.

Right side  a simple block diagram with

Laptop terminal

Loaner server

ROS2 nodes

Turtlesim

## Page 3  Camera and hand control for turtlesim

Menu title

Camera and turtlesim

Route

/camera-turtlesim

Presenter

Joseph Weng  041076091

Page title

Camera and hand control flow

Key story focus

Camera detection problems in VMware.

Switch from virtual machine to loaner laptop for vision.

Creating move and hands python files.

Connecting MediaPipe hands output to turtlesim motion.

Section 1  Camera problem on VMware

At first we tried to use a virtual machine on a personal laptop.

Inside VMware the camera was not detected in a stable way.

We spent a lot of time trying to make the camera work.

Because the camera was not reliable it was hard to test MediaPipe.

Section 2  Switch to loaner laptop for camera and vision

We changed the plan and used the loaner laptop directly for vision.

On the loaner laptop the camera worked much better.

MediaPipe hands could detect the hand landmarks in real time.

This change saved time and made the vision part possible.

Section 3  Move and hands python files

One python file sends move commands to turtlesim.

It publishes linear and angular velocity to a turtlesim command topic.

Another python file reads camera frames from the camera.

It uses MediaPipe hands to track the hand.

It maps hand positions or gestures to motion commands.

Section 4  Full flow for hand controlled turtle

Camera captures the hand.

MediaPipe hands finds key points in the image.

A ROS2 node converts the hand data to velocity messages.

Turtlesim subscriber receives the messages and moves the turtle.

Simple example commands for the page

ros2 run aisd_vision hands

ros2 run aisd_motion move

ros2 run turtlesim turtlesim_node

Suggested layout for Cursor

Show a vertical flow diagram

Camera

MediaPipe hands

ROS2 topic

Turtlesim

Next to the diagram show short text blocks with simple explanations.

## Page 4  Network and performance issues

Menu title

Network and performance

Route

/network-performance

Presenter

Peng Wang  041107730

Page title

Network delays and lab environment issues

Key story focus

Slow response when viewing from a different device over the network.

Many students on the same lab network.

Difference between running commands from Putty and running directly on the loaner laptop.

Practical tips for future students.

Section 1  Slow camera and turtle motion over the network

In some tests we used the loaner laptop as the main ROS2 machine.

We connected to it from Putty on a personal laptop.

In this case camera frames and turtle motion felt slow.

Hand movement and turtle reaction had clear delay.

Section 2  Lab network congestion

During lab time many students used the same network.

Sometimes the ssh connection was unstable.

This caused extra lag and even disconnects.

It made the demo less smooth and harder to debug.

Section 3  Running directly on the loaner laptop

When we ran the terminals directly on the loaner laptop the system was much faster.

The turtle reaction to hand movement was more real time.

The main downside was that we had to type long commands directly on that machine.

Long ros2 launch and ros2 run commands were easy to mistype.

Section 4  Practical tips for future students

If the network is slow, try to run the main ROS2 commands directly on the machine with the camera.

Prepare a text file with the common ros2 commands so you can copy and paste.

Test camera support early, not just on the demo day.

Do a short dry run in the lab before the real demo.

Suggested layout for Cursor

One section for problems.

One section for solutions.

Use simple cards for each tip so the site can show them as small boxes.

## Page 5  Conclusion and advice

Menu title

Conclusion and advice

Route

/conclusion

Presenter

Jiaxing YI  041127158

Page title

What we learned and advice for future students

Key story focus

ROS2 is new but not impossible.

Main difficulty is environment setup more than pure code.

Summary of what helped our team.

Final advice in student voice.

Section 1  Summary of learning

ROS2 concepts such as node, topic, publisher, and subscriber became clear after practice.

The hardest part was not the logic of turtlesim control.

The biggest blockers were wifi, camera detection, and network delay.

Once the environment worked, the remaining steps were manageable.

Section 2  What helped our team

Step by step setup

Fix wifi first.

Check basic ROS2 commands.

Test camera without MediaPipe.

Then add MediaPipe hands.

Clear mental model

Laptop or loaner laptop.

ROS2 nodes.

Topics.

Turtlesim.

Later the same pattern can be used for Create3.

Section 3  Why this project matters for AISD

This project helped us think like AI software developers.

We learned to work with a distributed system laptop, server, and robot.

We practiced debugging real environment problems, not only code bugs.

We became more comfortable with Linux, terminals, and ROS2 tools.

These skills are important for future AI and robotics projects in the program.

Section 4  Advice for future students

Simple advice list

Do not panic when ROS2 feels new.

Draw a small diagram of nodes and topics before coding.

Test each part in isolation camera, MediaPipe hands, turtlesim.

Avoid last minute debugging on the real demo day.

If possible, avoid virtual machines for camera based work.

Section 5  Closing for presentation

Our team used ROS2 and vision to move a simple turtle.

We met real problems with wifi, camera, and network speed.

By solving them we learned how powerful and flexible ROS2 can be.

We hope this talk makes your own assignment smoother and less stressful.

Suggested layout for Cursor

Simple text blocks for summary.

A highlight area with three or four advice cards.

A final short closing sentence for future AISD students.
