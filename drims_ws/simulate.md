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
