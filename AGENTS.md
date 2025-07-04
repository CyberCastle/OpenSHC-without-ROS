# AGENT Instructions

This file defines the guidelines for contributing to OpenSHC without ROS.

## Objective

This library provides locomotion control for a hexapod robot based on the Arduino Giga R1. The robot body forms a hexagon with legs spaced 60° apart, each leg having three joints for 3DOF. It includes inverse kinematics using DH parameters and Jacobians, orientation and pose control, gait planning and error handling.

## Code Style

-   Use C++17.
-   Four-space indentation with no tabs.
-   Place the opening brace on the same line as the declaration.
-   Document public functions using Doxygen-style comments (`/** ... */`) and in English.

## Testing

-   Run the unit tests before submitting changes.
-   Install the Eigen dependency by running `tests/setup.sh` if needed.
-   Build the tests with `make` inside the `tests` directory.

```bash
cd tests
./setup.sh
make
```

Each test executable can be run individually.

## Physical characteristics of the robot

These are the characteristics of a real robot, used to test this library.

-   robot height: 120 mm
-   robot weight: 6.5 Kg
-   body hexagon radius: 200 mm.
-   coxa length: 50 mm
-   coxa weight: 54 g
-   femur length: 101 mm
-   femur weight: 150 g
-   tibia length: 208 mm
-   tibia weight: 200 g

**Note:** Physically, if the robot has all servo angles at 0°, the femur remains horizontal, in line with the coxa. The tibia, on the other hand, remains vertical, perpendicular to the ground. This allows the robot to stand stably by default when all angles are at 0°.
 
## Commit Messages

-   Use imperative mood in English. Example: "Add new gait option".
-   Keep the summary under 72 characters.

## Pull Requests

-   Include a summary of the changes made.
-   Mention any known limitations or additional steps.
