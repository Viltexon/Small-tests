# Small-tests
Just learning some basic ROS concepts

## world_setup package

1. Create catkin workspace:
```
mkdir -p catkin_ws/src
cd catkin_ws/
catkin_make
```
2. Add `world_setup` package inside `catkin_ws/src` folder.
3. Source `setup.bash`:
```
source devel/setup.bash
```
4. Start the simulation:
```
roslaunch world_setup demo.launch
```
5. Start Rviz:
```
roslaunch world_setup rviz.launch
```

## test_world package

1. Add `test_world` package inside `catkin_ws/src` folder.
2. Build catkin workspace:
```
catkin_make
```
3. Source `setup.bash`:
```
source devel/setup.bash
```
4. Start the simulation:
```
roslaunch test_world demo.launch
```
