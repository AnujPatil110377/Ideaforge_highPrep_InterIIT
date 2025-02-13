Fault-Tolerant Quadcopter Control System




Official implementation for IdeaForge's Inter-IIT Tech Meet 13.0 problem statement on fault-tolerant quadcopter control during motor failures.

Table of Contents
Key Features
System Architecture
Setup Instructions
Simulation Launch
Repository Structure
Testing Workflow
Performance Metrics
Contributing
License
Contact
Key Features
🚁 Real-time motor failure simulation via custom PX4 module
🔍 Automatic fault detection with ROS2-based diagnostics
🛡️ Dual control strategies: INDI & Uniform Passive FTC
🔄 Hardware-in-Loop testing with QGroundControl integration
📊 MATLAB/Simulink validation framework
System Architecture
mermaid
Copy
Edit
graph LR
    A[Gazebo Sim] --> B[PX4 Flight Stack]
    B --> C[Failure Injection]
    C --> D{{Fault Detector}}
    D --> E[INDI Controller]
    D --> F[Passive FTC]
    E & F --> G[Stabilized Drone]
Setup Instructions
Prerequisites
Ubuntu 22.04 LTS (Dual-boot recommended)
NVIDIA GPU with proprietary drivers
16GB RAM + 30GB free storage
Stable internet connection
1. Base System Configuration
bash
Copy
Edit
sudo apt update && sudo apt full-upgrade -y
sudo apt install build-essential cmake python3-pip git -y
2. ROS2 Humble Installation
bash
Copy
Edit
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
3. PX4 Autopilot Setup
bash
Copy
Edit
cd ~
git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.14.0
bash PX4-Autopilot/Tools/setup/ubuntu.sh
sudo reboot
4. XRCE-DDS Configuration
bash
Copy
Edit
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install
sudo ldconfig /usr/local/lib/
5. QGroundControl Installation
bash
Copy
Edit
sudo usermod -a -G dialout $USER
sudo apt purge modemmanager -y
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
Simulation Launch
Terminal 1: PX4 SITL
bash
Copy
Edit
cd ~/PX4-Autopilot && make px4_sitl gz_x500
Terminal 2: DDS Agent
bash
Copy
Edit
MicroXRCEAgent udp4 -p 8888
Terminal 3: QGroundControl
bash
Copy
Edit
./QGroundControl.AppImage
Control Commands
bash
Copy
Edit
# Arm and takeoff
commander takeoff

# Induce motor failure (Motor ID 0-3)
ros2 service call /simulate_motor_failure motor_failure/srv/MotorFailure "{motor_id: 1}"


## Switch Control Strategy

ros2 service call /switch_controller std_srvs/srv/Trigger


## Repository Structure

.
├── PX4-Autopilot/
│   ├── src/modules/fault_tolerance/  # Custom failure modules
│   └── ...                          
├── ros2_ws/
│   ├── config/                      # PID/INDI parameters
│   ├── launch/                      # ROS2 launch files
│   └── src/
│       ├── fault_detection/         # Health monitoring
│       ├── indi_controller/         # INDI implementation
│       └── passive_ftc/             # Uniform FTC
├── simulation_ws/                   # Gazebo models
├── docs/                           # Technical reports
└── scripts/                        # Setup utilities


## Testing Workflow

### Nominal Operation

ros2 launch fault_tolerance nominal_operation.launch.py


### Inject Fault

ros2 service call /inject_fault std_srvs/srv/Trigger


### Monitor Recovery

ros2 topic echo /fault_diagnostics


## Performance Metrics

| Metric | INDI | Passive FTC |
|--------|------|-------------|
| Stabilization Time | 1.2s ±0.3 | 2.1s ±0.5 |
| Position Error | 0.8m | 1.5m |
| CPU Utilization | 18% | 12% |
| Energy Consumption | 23W | 19W |

## Contributing

1. Fork the repository.
2. Create a feature branch:

git checkout -b feature/new-feature

3. Commit your changes:

git commit -m 'Add feature'

4. Push to your branch:

git push origin feature/new-feature

5. Submit a pull request.

## License
MIT License - See LICENSE for details

## Contact
Team IIT Jodhpur  
B22EE010@iitj.ac.in

[Project Documentation](link) | [Issue Tracker](link)
