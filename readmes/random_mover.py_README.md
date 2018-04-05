# RandomMoveStrategy
Send random movement commands (in form of geometry_msgs/Twist) to a ROS topic.

## How to use
- See --help
- Easiest to use in a launch file - simply subordinate to the same element as the corresponding py_turtlesim, it will automatically publish to the correct topic.

## Constraints
- Can't change the output topic or publish rate
- "return" intelligence is rather primitive

## Pitfalls
None

## Missing functionality
Smarter intelligence
