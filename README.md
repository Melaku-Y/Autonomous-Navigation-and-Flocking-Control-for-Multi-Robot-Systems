# Autonomous-Navigation-and-Flocking-Control-for-Multi-Robot-Systems

This MATLAB project provides a simulation environment for studying autonomous navigation and multi-robot coordination. It implements the Artificial Potential Field method for single-robot navigation and consensus-based algorithms for multi-robot flocking and formation control.


## Features

-   **Artificial Potential Fields:** Simulate a single robot navigating towards a goal while avoiding obstacles.
-   **Consensus Algorithms:** Demonstrate how a group of robots can agree on a common position using graph-based control.
-   **Flocking Simulation:** Implements Craig Reynolds's three classic rules for flocking:
    -   **Alignment:** Steer towards the average heading of local flockmates.
    -   **Cohesion:** Steer to move toward the average position of local flockmates.
    -   **Separation:** Steer to avoid crowding local flockmates.
-   **Variable Communication:** Model different network topologies (fully connected, ring, path, directed graphs) and simulate the effect of a limited communication radius.
-   **Robot Models:** Supports both single integrator (velocity control) and double integrator (acceleration control) dynamics.
-   **Rich Visualization:** Generates plots for robot trajectories, final positions, and the evolution of position/velocity over time.

## Getting Started

Follow these instructions to get the simulation running on your local machine.

### Prerequisites

-   MATLAB

### Installation

1.  Clone this repository to your local machine:
    ```sh
    https://github.com/Melaku-Y/Autonomous-Navigation-and-Flocking-Control-for-Multi-Robot-Systems.git
    ```
2.  Open MATLAB and navigate to the cloned repository's directory.

## How to Run the Simulations

This project is divided into two main scripts.

### 1. Consensus for Single Integrator Robots

This simulation demonstrates how robots reach a consensus on their position based on a defined communication graph.

**To Run:**
1.  Open the `Consensus_simple_integrator.m` file (the first script provided).
2.  Select the scenario you want to simulate by setting the `question` variable:
3.  Run the script. The simulation will display the robot trajectories in real-time and generate final plots.

**Key Parameters to Modify:**
-   `N`: The number of robots.
-   `graph_type`: The communication structure to use.
-   `alpha`, `beta`: Control gains for consensus and repulsive forces.

### 2. Flocking for Double Integrator Robots

This simulation implements the three rules of flocking (alignment, cohesion, separation) for robots with more complex, acceleration-based dynamics.

**To Run:**
1.  Open the `Flocking_ALI_MESIHU.m` file (the second script provided).
2.  Adjust the simulation parameters at the top of the file to observe different behaviors.
3.  Run the script. You will see the robots moving as a flock and avoiding each other. Final plots showing trajectories and velocity/position profiles will be generated.

**Key Parameters to Modify:**
-   `N`: The number of robots.
-   `alpha`: Gain for the **alignment** force.
-   `beta`: Gain for the **cohesion** force.
-   `gamma`: Gain for the **separation** force.
-   `rho`: The **communication radius**. Robots will only interact with neighbors within this distance.

