# Mini-Project-Path-Planning-for-Autonomous-Vehicle-using-CARLA

## 📌 Project Overview  
This project focuses on implementing **path planning for autonomous vehicles** in a **static environment** using the **CARLA simulator**. The goal is to optimize vehicle autonomy by ensuring smooth navigation, obstacle avoidance, and traffic signal handling. The **A* algorithm** is employed to compute the shortest and most efficient path while ensuring smooth vehicle turns when obstacles are detected.

## 🚀 Key Features  
- 🛣️ **Shortest Path Calculation:** Implements the **A* algorithm** for optimized routing.  
- 🚧 **Obstacle Avoidance:** Enables smooth lane shifting when static obstacles are detected.  
- 🚦 **Traffic Signal Compliance:** The vehicle stops at red lights and resumes movement on green.  
- 🏎️ **CARLA Integration:** Utilizes CARLA’s built-in modules for lane detection, vehicle velocity monitoring, and path-following.  

## 📑 System Design  
### 🔹 Algorithms Used  
1. **A* Algorithm:**  
   - Uses cost function and heuristic function to optimize path planning.  
   - Provides better performance compared to **Dijkstra's Algorithm** by reducing unnecessary computations.  
2. **Dijkstra’s Algorithm (Evaluated but not used in final implementation):**  
   - Computes shortest paths but lacks heuristic guidance, making it slower.  

### 🔹 CARLA Integration  
- **Sensors & Actuators:** Uses CARLA’s sensor suite (LiDAR, cameras, GPS) for realistic simulation.  
- **Waypoint Navigation:** Computes waypoints and directs the vehicle along the optimal route.  

## 🛠️ Implementation Details  
- **Programming Language:** Python  
- **Simulator:** [CARLA](https://carla.org/)  
- **Path Planning:** A* Algorithm  
- **Traffic Signal Handling:** Integrated using CARLA’s Traffic Manager  

## 📊 Results & Observations  
- ✅ The vehicle follows an optimal path while avoiding static obstacles.  
- ✅ Smooth lane shifts occur when obstructions are detected.  
- ✅ The vehicle obeys traffic signals, stopping at red lights and resuming on green.  
- ✅ Improved efficiency over Dijkstra’s Algorithm due to heuristic-based optimization.  

## 🔮 Future Scope  
- 🔄 **Dynamic Path Planning:** Incorporating moving obstacles and pedestrians.  
- 🤖 **Reinforcement Learning:** Using RL techniques to improve decision-making.  
- 🚦 **Improved Obstacle Handling:** Enhancing lane-change logic for better real-world adaptability.  


💡 _Feel free to fork, contribute, and improve the project!_ 🚀 
