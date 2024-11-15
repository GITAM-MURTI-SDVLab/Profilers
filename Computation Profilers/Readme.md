# Computation Profilers

## Overview
This folder contains the necessary scripts and instructions for running computational profilers that monitor CPU load, RAM usage, and computation time during algorithm execution in a ROS environment.

## Workflow Explanation
The workflow for using the computation profilers involves the following steps:
1. **Python Node Creation**: A Python script is used to create a ROS node that performs the monitoring.
2. **Launch File**: The node is launched using a ROS launch file, which initiates the monitoring process.
3. **Clock Topic Detection**: The node waits for the `/clock` topic to appear. Once detected, the monitoring begins, capturing:
   - **CPU Load (%)**
   - **RAM Usage (MB)**
   - **Computation Time (ms)**

## How to Use the Profiler
1. **Navigate to the folder**:
   ```bash
   cd path/to/Computation/profilers

