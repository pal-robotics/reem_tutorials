# reem_tutorials

Basic tutorials to get familiar with the REEM robot in a simulated environment.

## Requirements

The following repositories are required to run the examples in simulation:

* reem_simulation (https://github.com/pal-robotics/reem_simulation)
* pal_face_detector_opencv (https://github.com/pal-robotics/pal_face_detector_opencv)

## How to run

### Look to point

In order to run launch REEM in simulation as follows:

```
roslaunch reem_gazebo reem_gazebo.launch world:=simple_office_people
```

and then run the example:

```
rosrun reem_tutorials look_to_point
``` 

a window showing the images from one of the frontal cameras of REEM will come up. You may click in the image to make REEM to look towards that direction.

### Face detections

This example shows how to subscribe to a _pal_detection_msgs::FaceDetections_ topic and show detected faces.

You may use the following simulation:

```
roslaunch reem_gazebo reem_gazebo.launch world:=meeting_a_female
```

Then launch the face detector:

```
roslaunch pal_face_detector_opencv detector.launch
``` 

and finally the example:

```
rosrun reem_tutorials face_detection
```

