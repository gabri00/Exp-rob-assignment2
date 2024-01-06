# Assignment 2

## Group members

| Name Surname          | ID       |
| --------------------- | -------- |
| [Gabriele Nicchiarelli](https://github.com/gabri00) | S4822677 |
| [Ivan Terrile](https://github.com/Ivanterry00)         | S4851947 |
| [Miriam Anna Ruggero](https://github.com/Miryru)   | S4881702 |
| [Davide Pisano](https://github.com/DavidePisano)        | S4363394 |

## Preliminary operations

You'll need to install some additional ROS packages:

```bash
cd ~/<ros_workspace>/src
git clone https://github.com/CarmineD8/aruco_ros.git
git clone https://github.com/CarmineD8/SLAM_packages.git # Remember to switch to noetic branch
git clone https://github.com/KCL-Planning/ROSPlan.git
```

For *ROSPlan* follow the instruction in their [readme](https://github.com/KCL-Planning/ROSPlan/blob/master/README.md) to properly install the dependencies.

Install dependencies:

```bash
cd ~/<ros_workspace>
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd ~/<ros_workspace>
catkin_make
catkin_make --only-pkg-with-deps my_rosplan_interface
```

## Run Gazebo simulation

Run the simulation:

```bash
roslaunch assignment_pkg simulation.launch
```

In another terminal, after launching the simulation, build and dispatch the plan:
```bash
cd ~/<ros_workspace>/src/Exp-rob-assignment2/
./rosplan_services.bash
```

To check the generated plan:
```bash
rostopic echo /rosplan_planner_interface/planner_output -p
```

## Description of the packages

...

#### Rqt graph

![Gazebo rqt](media/rqt_graph.png)

#### Flowchart

![Flowchart](media/flowchart.png)

#### Video demo

https://github.com/gabri00/Exp-rob-assignment2/assets/31885249/597f6e62-6481-4e97-b565-68672145cb75

## Further improvements

1. 
2. 
3. 

## References

- [Husarion ROSbot](https://husarion.com/manuals/rosbot/)

- [ROSbot GitHub repository](https://github.com/husarion/rosbot_ros/)

- [Course professor](https://github.com/CarmineD8/)
