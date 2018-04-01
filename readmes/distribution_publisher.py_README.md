# Distribution Publisher
Generate data based on a distribution or file and publish it to ROS.

## How to use
- See --help
- Gen mode: Optionally supply args to the generator with "gen gaussian (a) (b)"

## Constraints
- Make sure the argument order is correct (argparse ensures this anyways, but don't be confused): <General args (id, intrusion)> <Mode> <Mode args>

## Pitfall
- The supplied generator arguments (see How to use) are only used if the number of arguments is correct - no sanity check!

## Missing functionality
None