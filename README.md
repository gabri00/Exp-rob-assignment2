# Assignment 2

## Comandi utili per lanciare tutte le parti di rosplan in sequenza

Prima bisogna creare il file domain.pddl e problem.pddl
Poi bisogna creare un file launch con tutto il necessario e lanciarlo:
```bash
roslaunch rosplan_planning_system file_name.launch
```

Poi bisogna lanciare i seguenti comandi su un altro terminale:
```bash
rosservice call /rosplan_problem_interface/problem_generation_server
rosservice call /rosplan_planner_interface/planning_server
rosservice call /rosplan_parsing_interface/parse_plan
rosservice call /rosplan_plan_dispatcher/dispatch_plan
```

Clone the following packages:
```bash
git clone https://github.com/CarmineD8/my_rosplan_interface.git
git clone https://github.com/CarmineD8/rt2_packages.git
git clone https://github.com/CarmineD8/planning.git
```

Come pubblicare il goal da terminale per la move_base:
```bash
rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'map'
    pose:
      position:
        x: 0.0
        y: 1.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0" 
```

## Group members

| Name Surname          | ID       |
| --------------------- | -------- |
| [Gabriele Nicchiarelli](https://github.com/gabri00) | S4822677 |
| [Ivan Terrile](https://github.com/Ivanterry00)         | S4851947 |
| [Miriam Anna Ruggero](https://github.com/Miryru)   | S4881702 |
| [Davide Pisano](https://github.com/DavidePisano)        | S4363394 |

## Preliminary operations

Install dependencies:

```bash
cd ~/<your_workspace>
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd ~/<your_workspace>
catkin_make
```

## Run on the Rosbot

#### Step 1: Connect to the local network

| Network name | Network password |
| ------------ | ---------------- |
| TP_LINK      | 03694008         |

#### Step 2: Add the ROS master URI and user's IP address

Add the following lines at the bottom of the `~/.bashrc` file:

```bash
export ROS_MASTER_URI=https://192.168.1.10x:11311
export ROS_IP=<YOUR_IP_ADDRESS>
```

#### Step 3: Connect to the robot via SSH

In the local terminal:

```bash
ssh husarion@192.168.1.10x
# x is the identifier number of the robot (written on the bot)
```

**Password**: husarion

#### Step 4: Start the drivers

In the Rosbot terminal:

```bash
roslaunch tutorial_pkg all.launch
```

#### Step 5: Start the simulation

From the local terminal:

```bash
roslaunch rosbot_gazebo real_rosbot.launch
```
