### Terminal 1
```bash
ros2 launch drims2_description ur10e_start.launch.py fake:=true
```

### Terminal 2 
```bash
ros2 launch drims2_dice_simulator spawn_dice.launch.py face_up:='5' position:='[-0.1, 0.0, 0.85]'
```

### Terminal 3
```bash
ros2 launch drims2_homework demo_node_start.launch.py
```
### Terminal 4
```bash
ros2 launch depthai_examples rgb_stereo_node.launch.py
```
### Terminal 5
Service: 
```bash
ros2 service call /dice_identification drims2_msgs/DiceIdentification
```


### Record Bag
```bash
ros2 bag record -o  bags/<bag-name> -a
```


ros2 action send_goal /move_to_pose drims2_msgs/action/MoveToPose "pose_target:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: 'checkerboard'
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
cartesian_motion: false"
