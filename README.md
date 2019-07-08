# f110_planning_example
A* and Dijkstra's toy example in racecar simulator

![](https://github.com/hzheng40/f110_planning_example/blob/master/astar.gif?raw=true)

## Quick start

To run the example, make sure you already have mLab's racecar simulator working first.
Then clone this repo to your ```catkin_ws/src```:

```
git clone https://github.com/hzheng40/f110_planning_example.git planning_example
```

To run the example:

```
roslaunch planning_example planning_toy.launch
```

In RVIZ, use the ```2D Pose Estimate``` tool to set the start point on the map. Then use the ```2D Nav Goal``` tool to set the goal point on the map. Once the algorithm runs for the first time, changing either the start point or the goal point will trigger the planning.

The green cubes in the visualization represents the expansion frontier. The red cubes represents the visited set. And the blue cube represents the current node. The gree path represents the path found by the algorithm.

The default algorithm used is A*, to change to Dijkstra's, change the variable ```self.use_astar``` to ```False``` in ```scripts/planning_toy.py```.
