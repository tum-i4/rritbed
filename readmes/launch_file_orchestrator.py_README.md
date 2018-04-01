# Launch File Orchestrator
Create randomised ROS launch files.

## How to use
- See --help
- Make sure the generator definitions file exists at "~/ros/gens". Create it by running "generators.py <FILE_PATH>", found in "catkin_ws/src/turtlesim_expl/src/generator/".

## Constraints
- The number, type and layout of generators per client can't be chosen, and neither can the min/max number of generators, pysims, ...
- Rosbag recording can't be disabled and will always record to "~/ros/bags/recording-all..."

## Pitfalls
Duplicate VIN (--allow-duplicate-vin) can't be labelled.

## Missing functionality
See constraints, pitfalls.