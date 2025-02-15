# RRP Robot PID Controller

This project implements a **PID-based controller** for an **RRP robotic arm** in **ROS**. The controller adjusts joint efforts using real-time feedback from `/rrp/joint_states`, ensuring smooth and accurate movement through defined waypoints.

## Features
- **PID Control**: Implements proportional-integral-derivative control for precise movement.
- **ROS Integration**: Uses publishers and subscribers to interact with the robot.
- **Data Logging**: Saves joint trajectories to CSV for analysis.
- **Docker Support**: Runs the simulation in a **Docker container** for easy setup.

---

## Installation & Setup

### Prerequisites
Ensure you have:
- **ROS Noetic/Melodic** installed
- **Gazebo** for simulation
- **Docker** (optional for containerized execution)

### Clone the Repository
```bash
git clone https://github.com/yourusername/rrp_robot_controller.git
cd rrp_robot_controller
```

### Build & Run (Without Docker)
1. **Source ROS**:
   ```bash
   source /opt/ros/noetic/setup.bash
   ```
2. **Build the workspace**:
   ```bash
   catkin_make
   source devel/setup.bash
   ```
3. **Launch the Simulation**:
   ```bash
   roslaunch rrp_bot rrp_simulation.launch
   ```
4. **Run the PID Controller**:
   ```bash
   rosrun rrp_bot rrp_controller.py
   ```

---

## Running with Docker

### Build the Docker Image
```bash
docker build -t rrp_bot .
```

### Run the Container
```bash
docker run --rm -it --net=host --privileged rrp_bot
```

### Launch ROS Inside Docker
```bash
source /opt/ros/noetic/setup.bash
roslaunch rrp_bot rrp_simulation.launch
```

---

## Usage

### Define Setpoints
Modify `positions` in **`rrp_controller.py`** to set custom waypoints:
```python
positions = [
    (0.0, 0.77, 0.34),
    (-0.345, 0.425, 0.24),
    (0.67, -0.245, 0.14),
    (0.77, 0.0, 0.39)
]
```

### Monitor Performance
Check the frequency of control commands:
```bash
rostopic hz /rrp/joint1_effort_controller/command
```

### Logs & Data
- **Trajectory data** is saved in `trajectory1.csv`, `trajectory2.csv`, and `trajectory3.csv`.
- **Errors are published to** `/rrp/joint_error`.

---
