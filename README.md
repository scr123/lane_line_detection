# Lane Line Detection

Lane Line Detection is a class using traditional image processing techniques to extract lane lines from video frames,
and provide a polynomial representation with associated meta-data for each lane line.

This package includes a ROS interface with dyanmic_reconfigure and rviz support.

## Release Notes
* Complete:
    * Basic lane line detection outputting an object list
    * ROS Interface:
        * Dynamic Reconfigure for image processing tuning
        * Rviz for visualizing lane lines and their respective meta-data
* TODO:
    * Provide facility for mapping camera pixels to meters
    * CUDA support for OpenCV and Eigen library calls
    * Deep Learning for:
        * Lane line region of interest (ROI)
        * Classification of lane line types (solid, striped, etc...)
    * Add more LaneLineDetection GTests, and have them all pass

## ROS Interface
### Subscribers
* image_sub - sensor_msgs::Image - subscribes to raw image

### Publishers
* image_pub - sensor_msgs::Image - publishes processed image
* lane_lines_pub - LaneLines - publishes all lane line information

### Messages
* LaneLines.msg - LaneLine vector
* LaneLine.msg  - lane line representation and meta-data

### Parameters
* param.yaml - contains all ROS parameters
* calibration.yaml - contains intrinsic camera calibration information

### Dynamic Reconfigure
* Image Region of Interest (ROI)
* Yellow Color Mask
* White Color Mask
* Gaussian Blur Weights
* Canny Edge Detection Weights
* Probabilistic Hough Line Transform Weights

## Getting Started

These instructions will help you generate the necessary documentation for using this package, and list the required dependencies.

### Documentation

The documentation for this project is Doxygen based. To generate, execute the following commands:

```
cd <path>/lane_line_detection
doxygen Doxyfile
```

### Dependencies

The follwing dependencies are required, and can be installed accordingly.

```
sudo apt install doxygen
sudo apt install libgtest-dev
sudo apt install build-essential
sudo apt install python-catkin-tools
sudo apt install ros-melodic-desktop-full
sudo apt install ros-melodic-rviz-visual-tools

```
## Running the tests

To run the unit tests for this package, use the following command:

```
catkin build lane_line_detection --no-deps --catkin-make-args run_tests
```

### Break down into end to end tests

The unit test for this package extracts lane lines from test images validating functionality.

```
lane_line_detection_test.cpp
```

## Built With

* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html) - Build tool used for compiling this project
* [cv_bridge](http://wiki.ros.org/cv_bridge) - Packge for converting between ROS and OpenCV images
* [Eigen](https://eigen.tuxfamily.org/dox/index.html) - Open source matrix and vector math library
* [Google Test](https://github.com/google/googletest) - Unit testing framework
* [OpenCV 3.2](https://opencv.org/opencv-3-2) - Open source computer vision library
* [ros_melodic](http://wiki.ros.org/melodic) - Open source meta-operating system
* [rviz_visual_tools](http://wiki.ros.org/rviz_visual_tools) - Wrapper package for rviz


## Authors

* **Sean Crutchlow**

## License

This project is licensed under the MIT License - see the LICENSE file for details
