# occlusion_render
A utility for rendering the view from the Sawyer robot's camera to detect occlusions, developed by [Edward Ahn](https://github.com/edhyah).

## Details
This package implements a ROS node that uses joint angles recorded by RethinkRobotic's Sawyer robot to render the camera view offscreen. This image can be used to automatically detect instances when the arm occludes the end effector (see the package [occlusion_detect](https://github.com/r-pad/occlusion_detect)).

## Installation Requirements
This project depends on the following:

* ROS Indigo
* [offscreen_render](https://github.com/personalrobotics/offscreen_render)
* [sawyer_robot](https://github.com/RethinkRobotics/sawyer_robot)

## Usage
```python
roslaunch occlusion_render render.launch
```

## Sample Results
The results below show the rendering produced by this package on the left, and the actual image taken by the camera on the right.

![alt text](results/result1.png?raw=true "Sample Result 1")
![alt text](results/result2.png?raw=true "Sample Result 2")
![alt text](results/result3.png?raw=true "Sample Result 3")

## Future Work
* We want to use a depth map produced by [range_image_registration](https://github.com/personalrobotics/range_image_registration) to display depth information on the rendered image.
* We intend to develop the package [occlusion_detect](https://github.com/r-pad/occlusion_detect), which will subscribe to the rendered images to produce a binary output determining whether the end effector is too occluded in an image to be useful for other purposes.
