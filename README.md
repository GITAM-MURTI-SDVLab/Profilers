# Project Overview: Computation Profilers and Mapping Accuracy

## Introduction
This repository is dedicated to showcasing the use of computation profilers and mapping accuracy methods. These tools are essential for understanding and optimizing the performance of algorithms in Unmanned Aerial Vehicles (UAVs) and Autonomous Ground Vehicles (AGVs). Profiling helps ensure real-time efficiency, contributing to better system reliability and energy management.

## Importance of Computation Profilers
Computation profilers provide critical insights into different performance metrics, which are indispensable for optimizing the functionality of UAVs and AGVs. Below, we break down the key profilers included in this repository:

### 1. **CPU Load (%)**
- **Definition**: CPU load refers to the percentage of total processing power being used by an algorithm during its execution.
- **Importance**:
  - Monitoring CPU load helps in understanding how much of the available processing power an algorithm consumes.
  - High CPU load can indicate potential bottlenecks or inefficiencies, which can lead to performance degradation, especially in real-time applications.
- **Application in UAVs and AGVs**:
  - For UAVs, excessive CPU load can limit the ability to process new sensor data, affecting tasks like navigation and obstacle avoidance.
  - In AGVs, balancing CPU load ensures smooth and uninterrupted path planning and decision-making in dynamic environments.
- **Tooling**: Profiling tools such as `htop`, `top`, or custom scripts in Python can be used to monitor CPU usage.

### 2. **RAM Usage (MB)**
- **Definition**: RAM usage measures the amount of memory an algorithm consumes while running.
- **Importance**:
  - High RAM usage can cause a system to rely on disk swapping if it exceeds available physical memory, significantly slowing down operations.
  - Memory management is critical for embedded systems in UAVs and AGVs where resources are often limited.
- **Application in UAVs and AGVs**:
  - UAVs require efficient memory usage to avoid performance issues during flight, which could affect mission success and safety.
  - In AGVs, maintaining low and predictable RAM usage ensures consistent response times in operations such as warehouse navigation or industrial automation.
- **Tooling**: Tools like `memory_profiler` in Python or Linux commands like `vmstat` can help monitor RAM usage effectively.

### 3. **Computation Time (ms)**
- **Definition**: Computation time refers to the time taken for an algorithm to process a given task or complete a cycle of its main loop.
- **Importance**:
  - Measuring computation time helps identify delays in the system and ensures algorithms meet real-time processing requirements.
  - Low computation time is crucial for ensuring quick decision-making in UAVs and AGVs.
- **Application in UAVs and AGVs**:
  - UAVs rely on minimal computation delays to adapt to sudden changes in their environment, such as avoiding unexpected obstacles.
  - AGVs use real-time processing to navigate efficiently and interact with their surroundings without lag, ensuring smooth operations.
- **Tooling**: Profilers such as `cProfile`, `timeit`, or custom timing scripts can be used to measure computation time with high precision.

### 4. **Map Accuracy (%)**
- **Definition**: Map accuracy evaluates how closely the generated map aligns with a known ground truth or reference.
- **Importance**:
  - High map accuracy is essential for reliable navigation and localization in both UAVs and AGVs.
  - Discrepancies in mapping can lead to poor path planning, navigation errors, or collision risks.
- **Application in UAVs and AGVs**:
  - UAVs use accurate mapping for complex missions such as search and rescue, where navigating through variable terrain is crucial.
  - AGVs depend on accurate maps for effective route planning and obstacle avoidance in warehouses or other operational environments.
- **Tooling**: Techniques involving point cloud comparison, using tools like Open3D or the `point cloud library (PCL)`, can help in evaluating and quantifying map accuracy.

## Real-Time Impact
### How These Profilers Enhance UAV and AGV Systems
- **UAVs**: The ability to monitor CPU load, RAM usage, and computation time helps ensure smooth, energy-efficient flight operations. Accurate map generation allows UAVs to adapt in real-time to changes in the environment.
- **AGVs**: Ensuring computational efficiency and accurate mapping leads to optimized path planning, energy savings, and reliable obstacle avoidance, directly impacting productivity and safety.

## Folder Structure
- **Computation profilers**: Detailed instructions and examples on how to use various profilers for performance analysis.
- **Mapping Accuracy**: Guides for testing and evaluating mapping accuracy, ensuring the reliability of navigation systems.

Explore the subfolders for step-by-step instructions on how to use these tools effectively for your UAV and AGV applications.
