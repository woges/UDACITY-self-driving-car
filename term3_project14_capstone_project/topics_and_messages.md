# Topics and messages

A cheatsheet of topics and messages formats, to avoid needing to look for them each time.


## Messages

### styx_msgs/Lane

Getting message format:

```
$ rosmsg info styx_msgs/Lane
```

Message format:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
styx_msgs/Waypoint[] waypoints
  geometry_msgs/PoseStamped pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  geometry_msgs/TwistStamped twist
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
```

It is used on:

- `/base_waypoints` --> List of waypoints read from csv file.
-  `/final_waypoints` --> Subset of `/base_waypoints` that we need to generate in `waypoint_updater.py`. The first waypoint is the one in `/base_waypoints` which is closest to the car.

Example of data, from:

`$ rostopic echo /base_waypoints`

The result is:

```
header:
  seq: 182
  stamp:
    secs: 0
    nsecs:         0
  frame_id: /world
waypoints:
  -
    pose:
      header:
        seq: 0
        stamp:
          secs: 0
          nsecs:         0
        frame_id: ''
      pose:
        position:
          x: 909.48
          y: 1128.67
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    twist:
      header:
        seq: 0
        stamp:
          secs: 0
          nsecs:         0
        frame_id: ''
      twist:
        linear:
          x: 11.1112
          y: 0.0
          z: 0.0
        angular:
          x: 0.0
          y: 0.0
          z: 0.0
  -
    pose:
      header:
        seq: 0
        stamp:
          secs: 0
          nsecs:         0
        frame_id: ''
      pose:
        position:
          x: 909.486
          y: 1128.67
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    twist:
      header:
        seq: 0
        stamp:
          secs: 0
          nsecs:         0
        frame_id: ''
      twist:
        linear:
          x: 11.1112
          y: 0.0
          z: 0.0
        angular:
          x: 0.0
          y: 0.0
          z: 0.0
...
```

It is **very** long; not great for printing...

### geometry_msgs/PoseStamped

To get message format:

```
$ rosmsg info geometry_msgs/PoseStamped
```

Message format is:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

Used in the following topics:

- `/current_pose` --> current pose, as provided by the car/simulator.

An example

```
$ rostopic echo /current_pose
```

The result is:

```
header:
  seq: 670
  stamp:
    secs: 1504453486
    nsecs: 724562883
  frame_id: world
pose:
  position:
    x: 1131.22
    y: 1183.27
    z: 0.1069651
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0436201197059
    w: 0.999048189607
---
header:
  seq: 671
  stamp:
    secs: 1504453486
    nsecs: 727682113
  frame_id: world
pose:
  position:
    x: 1131.22
    y: 1183.27
    z: 0.1069651
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0436201197059
    w: 0.999048189607
---
...
```

Each one starting with `header:` is a different message; but there are lots of them per-second.