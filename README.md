# 3D Human Position Estimation

## Disclosure

This is a final year BSc research project carried at the University of Leeds and whose content is open-source, so you are allowed to use all content behind it under the GPL3 license conditions. However appropriate accreditation has to be given when these are used, by referencing the original author and the institution as follows:

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

This repository hosts my final year project, which is a **ROS package** that estimates 3D human position in the map. The estimation procedure is broken into three steps:

1. ROS RGB image conversion
2. Person detection on the converted image (OpenCV format)
3. Distance and 3D position computation

The conversion step makes use of the ROS Indigo CvBridge interface. Person detection is done using the **deep learning MobileNet SSD model**. Distance and 3D position computation make use of the **RGB-D** (such as Kinect), therefore it is **essential** that such sensor is available.

The package is extremely efficient as it finished the estimation process in under **47 milliseconds**.

## What Do I Need To Run The Project?

The project does not have many software dependencies as it only relies on the ROS middleware and the OpenCV library. Moreover, this project assumes that you have access to a camera system for RGB data and a depth sensor for depth information. In case you don't, this can be easily fixed by buying a Kinect camera for 20$. Apart from that you are set to start using the package.

## How Can I Run It?

All information regarding the installation process can be found in the [Wiki](https://github.com/itaouil/HARN/wiki). Enjoy!

## I Want To Contribute

The final result works well, but it can be further improved. So if you are willing to contribute to the project in any form such as update the documentation, improve the code-base, fix bugs or even publish the package to ROS and maintain it, please do contact me at **itaouil95@gmail.com** or send over a pull request. You can find a detailed wish-list in the [Wiki](https://github.com/itaouil/HARN/wiki).
