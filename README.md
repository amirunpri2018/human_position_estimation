# HARN

## Disclosure

This is a final year BSc research project carried at the University of Leeds and whose content is open-source, so you are allowed to use the idea, source code and images behind it under the GPL3 license conditions. However appropriate accreditation has to be given when these are used, by referencing the original author and the institution as follows:

**LaTeX Reference:**
```latex
@misc{itaouil18fyp,
  author         = {Ilyass Taouil},
  title          = {{3D Human Position Estimation}},
  year           = {2018},
  url            = {{https://github.com/itaouil/human_position_estimation.git}},
  note           = {School of Computing, The University of Leeds}
}
```

## Introduction

This repository hosts my final year project, which is a ROS package that estimates 3D human position in the map. The estimation procedure is broken into three steps, a first raw coloured image conversion, a detection on the OpenCV image and a final distance and 3D pose computation, where by **pose is meant the actual x, y and z coordinates of the detection in the map**.

The project makes use of the SSD deep learning model to perform robust human detection on the RGB data and whose installation and set-up instructions are given in sections below. Moreover, depth data coming from the RGB-D sensor are used to computed the distance and ultimately the 3D position of the detection or detections.

**If you need a package that needs to estimate people position and you have access to RGB-D sensory data then this package is for you.**

## Things To Know

This package was developed using Python3, OpenCV on the ROS Indigo distro running on Ubuntu 14.04 (LTS). The robot used was [TIAGO](http://tiago.pal-robotics.com/) by PAL Robotics.

The estimation procedure is really efficient as it takes only 46 milliseconds for each logic run. However, because ROS services were used due to RGB-depth synchronisation issues it is not a constant frame by frame detection, but still it is probably good for your type of usage.

The final result works well, but it can be further improved. So if you are willing to contribute to the project in any form such as update the documentation, improve the codebase, fix bugs or even publish the package to ROS and maintain it, please do contact me at **itaouil95@gmail.com** or just send a pull request. You can also find the wish-list at the end of the REAMDE file with possible fixes to start from.

## General Installation

In order to use the 

## Run It

**Detailed** instructions on how to install, configure and run anything required by the project is available in the [Wiki](https://github.com/itaouil/HARN/wiki). of the project.
