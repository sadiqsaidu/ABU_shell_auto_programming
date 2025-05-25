# Shell Simulation

## Overview

Shell Simulation is a ROS 2 project developed for the Shell Eco-marathon Autonomous Programming Competition 2025. This project simulates an autonomous vehicle in the CARLA simulator and demonstrates key elements of autonomous driving:

- **Path Planning:** A planning node publishes a series of predefined waypoints that the vehicle must navigate. As the vehicle reaches each waypoint (within a configurable threshold), the next target is published.
- **Perception:** A perception node processes depth images from a dedicated camera. It analyzes a specific region of interest in the image to detect obstacles in front of the vehicle.
- **Vehicle Control:** A control node receives data from odometry, speed sensors, and the current target waypoint, then computes simple control commands (steering, throttle, and braking) to follow the route while respecting speed limits and avoiding obstacles.

The objective is to optimize energy efficiency by covering as much distance as possible using minimal energy, while following simulated traffic rules and avoiding obstacles.

## Requirements

- **Operating System:** Ubuntu 22.04 (Jammy Jellyfish)
- **ROS 2 Distribution:** Humble
- **CARLA Simulator:** 0.9.15 (with CARLA ROS Bridge)
- **Dependencies:** 
  - Standard ROS 2 packages (e.g., rclpy, nav_msgs, geometry_msgs)
  - OpenCV and cv_bridge for image processing
  - NumPy for numerical operations

## Installation

1. **Setup your ROS 2 workspace:**

   ```bash
   # Clone or copy the project into your ROS 2 workspace's src folder.
   git clone <repository_url>  # Replace <repository_url> with the project repository URL
   ```

2. **Build the workspace:**

   ```bash
   # From the workspace root directory:
   colcon build
   ```

3. **Source the workspace:**

   ```bash
   # Source the install setup script:
   source install/setup.bash
   ```

## Running the Simulation

The project is structured into multiple ROS 2 nodes. To start the simulation, use the provided launch file:

```bash
ros2 launch shell_simulation shell_simulation.launch
```

This command will start the following nodes:
- **Planning Node:** Publishes sequential waypoints for navigation.
- **Control Node:** Listens to odometry, speed, and waypoint data to issue control commands.
- **Perception Node:** Detects obstacles by processing depth images and publishes alerts.

The simulation will run until either all goal points are reached or the maximum runtime is exceeded. During the run, performance data (e.g., energy efficiency) is recorded for later analysis.

## Competition Objectives

In the competition, your solution must maximize energy efficiency (km/kWh) by:
- Optimizing the path to cover maximum distance while consuming minimal energy.
- Accurately detecting and avoiding obstacles.
- Adhering to traffic rules (such as speed limits and lane discipline) to avoid penalties.

Refinements in the planning, control, and perception aspects of this project can greatly improve overall performance.

## Additional Information

- **Testing:** Before each submission, verify that your code compiles and that the vehicle performs safely in the simulation.
- **Submission:** Ensure your ROS packages install correctly. The final submission should include a package named `shell_simulation` with a launch file `shell_simulation.launch` that runs without additional parameters.
- **Performance Records:** After each attempt, you may review the ROS bag files to analyze vehicle behavior and energy use.

## License

[Specify license information here.]

## Acknowledgments

This project is developed for the Shell Eco-marathon Autonomous Programming Competition 2025 in partnership with the Southwest Research Institute.
