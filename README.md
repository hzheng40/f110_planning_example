# f110_planning_example
A* and Dijkstra's toy example in racecar simulator

![](https://github.com/hzheng40/f110_planning_example/blob/master/astar.gif?raw=true)

## Quick start

To run the example, make sure you already have mLab's racecar simulator working first.
Then:

```
roslaunch planning_example planning_toy.launch
```

The default algorithm used is A*, to change to Dijkstra's, change the variable ```self.use_astar``` to ```False``` in ```scripts/planning_toy.py```.
