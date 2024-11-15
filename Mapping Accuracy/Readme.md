# Mapping Accuracy

## Overview
This folder provides tools and instructions for evaluating mapping accuracy for both 2D and 3D SLAM methods. Accurate mapping is essential for reliable autonomous navigation in various environments. This document covers how RMSE is used to assess map accuracy and provides step-by-step workflows for both 2D and 3D SLAM evaluations.

## What is RMSE and Why Use It?
### RMSE Overview
**Root Mean Square Error (RMSE)** is a statistical measure that evaluates the average deviation between predicted values (e.g., SLAM-generated maps) and actual values (e.g., ground truth maps). It serves as a reliable metric to quantify the accuracy of SLAM algorithms.

### Importance of RMSE
- **Quantitative Assessment**: Provides a numerical value summarizing the map's accuracy, where a lower RMSE indicates better performance.
- **Cross-Method Comparison**: Enables uniform comparison between different SLAM algorithms, facilitating the identification of the most effective method.
- **Error Analysis**: Accounts for both the magnitude and distribution of errors, helping to pinpoint specific areas of map deviation.

---

## Workflow for 2D SLAM Accuracy Evaluation

### Step-by-Step Process:
1. **Collect Ground Truth Data**:
   - Obtain accurate reference maps of the environment for comparison.
2. **Run 2D SLAM Algorithm**:
   - Execute the SLAM algorithm to create a map of the environment.
   - Use tools like GMapping, Cartographer, or Hector SLAM.
3. **Export Generated Map**:
   - Save the generated map in a suitable format (e.g., image or occupancy grid).
4. **Align Maps**:
   - Align the SLAM-generated map with the ground truth to ensure point-to-point correspondence.
5. **Compute RMSE**:
   - Use a Python script or analysis tool to calculate the RMSE between the SLAM-generated map and the ground truth.
   - The RMSE value quantifies the deviation and provides an accuracy metric.
6. **Analyze Results**:
   - Assess the RMSE output and visualize overlays of the maps to identify areas of high and low accuracy.

### Key Points:
- Ensure ground truth data is accurate and corresponds to the same environment conditions as the SLAM-generated map.

---

## Workflow for 3D SLAM Accuracy Evaluation

### Step-by-Step Process:
1. **Run 3D SLAM Algorithm**:
   - Execute the chosen 3D SLAM algorithm (e.g., LIO-SAM, FLOAM, ISC-LOAM) to create a 3D map.
2. **Visualize in RViz**:
   - View the generated map in RViz to verify completeness and initial alignment.
3. **Export to PCD File**:
   - Convert the RViz visualization into a PCD (Point Cloud Data) file.
   - Use `rosbag` and ROS tools to record data and export the map.
4. **Collect or Use Reference PCD File**:
   - Obtain a ground truth PCD file or another SLAM-generated PCD file for comparison.
5. **Align Point Clouds**:
   - Align the two PCD files using a point cloud registration method, such as ICP (Iterative Closest Point).
6. **Compute RMSE**:
   - Use a Python script or 3D processing software like Open3D or PCL (Point Cloud Library) to compute the RMSE between corresponding points in the PCD files.
7. **Analyze Results**:
   - Examine the RMSE value and overlay the point clouds visually to confirm accuracy and detect areas of high deviation.

### Key Points:
- Ensure the point clouds are aligned properly before computing RMSE to avoid skewed results.
- Visual inspection of the point cloud overlay is essential to validate numerical results.

---

## Summary
Both 2D and 3D SLAM workflows involve running SLAM algorithms, obtaining or exporting maps, aligning with reference data, and calculating RMSE for quantitative analysis. These workflows help in understanding the performance and reliability of SLAM methods for different applications.

