# ROSBot Autonomous Package Pickup and Delivery

## 📌 Overview
This project implements a fully autonomous **pickup and delivery system** using the **ROSBot Pro 3.0** platform and **ROS 2 Humble**.  
The robot navigates an unknown environment, detects **ArUco markers** as pickup and delivery points, plans routes, and executes deliveries without human intervention.

## 🔑 Key Features
- **ArUco Marker Detection**: Identifies pickup (even IDs) and delivery (odd IDs) markers in real time.  
- **Frontier-Based Exploration**: Enables the robot to autonomously map and explore unknown environments.  
- **Task Planning**: Implements a queue-based planner to dynamically assign and reorder delivery tasks.  
- **Visual Servoing**: Achieves precise alignment (~2 cm accuracy) using a mounted pointer and visual feedback.  
- **Hybrid Architecture**: Combines reactive, deliberative, and executive layers for robust task execution.  

## 🛠️ Tools & Technologies
- ROS 2 Humble  
- ROSBot Pro 3.0  
- OpenCV (ArUco)  
- SLAM Toolbox, Nav2  
- Python (ROS 2 nodes)

## 📊 Results
- Completed **multiple pickup–delivery cycles** in both simulation and real-world tests.  
- Achieved an average task completion time of **~128 seconds** per delivery pair.  
- Demonstrated **100% marker detection accuracy** under normal conditions.  
- Robust visual servoing with smooth alignment and safety timeout (10s).

## 🚀 Future Improvements
- Advanced task scheduling with route optimization.  
- Integration of physical contact sensors for confirmation of deliveries.  
- Enhanced obstacle avoidance in dynamic environments.  
- Improved sim-to-real transfer using Gazebo/Webots simulations.
