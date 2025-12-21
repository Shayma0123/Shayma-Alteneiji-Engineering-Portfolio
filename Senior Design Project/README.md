<h3> Senior Design Project </h3>
<h4> Radar-Camera Data Fusion System Using the EKF for Target Tracking 
</h4>
<p>
Designed and implemented a multi-sensor fusion system to track moving objects by combining radar and camera measurements. The EKF integrates asynchronous radar data (position, velocity) with camera YOLO object detection outputs (centroid detections), while a Global Nearest Neighbor (GNN) algorithm handles measurement-to-track assignment. Fusion results are benchmarked using ArUco markers to extract the ground truth states. 
Applications include <strong>Advanced Driver Assistance Systems (ADAS)</strong>.
</p>
<strong> Skills Demonstrated:</strong> 
<li> EKF derivation & implementation </li>
<li> Sensor calibration & interfacing </li>
<li> MATLAB pipelines </li>
<li> Guidance & integration of team contributions </li>


<p align="center">
  <img src="Senior Design Project/Fusion_System_Higher_Level_Block_Diagram .png" alt="System Block Diagram" width="600"><br>
  <em>System Block Diagram for the Radar–Camera Fusion System</em>
</p>

<strong> Contribution</strong> 
<li> Designed and implemented EKF fusion from scratch, handled calibration, MATLAB implementation, interfaced radar and camera sensors, integrated teammates’ modules, and guided system-level testing. </li>

<p><strong> Demo Video Link</strong></p>
  [![Demo Video](https://img.youtube.com/vi/dQw4w9WgXcQ/0.jpg)](https://www.youtube.com/watch?v=H9NKajhr28o)
  <em>Screenshot from the Demonstration Video </em>
</p>
<strong> Key Notes / Highlights</strong> 
<li> RMSE for radar-only tracking reached as low as 0.5 m.</li>
<li> Asynchronous fusion handles out-of-sequence measurements. </li>
<li> The camera-based fog classifier is implemented but could not be tested, and the YOLO dataset could not be integrated due to reprojection error originating from setup error. </li>
<li> All MATLAB code is documented for clarity; scripts are modular.</li>

