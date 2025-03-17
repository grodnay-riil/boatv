# ✨ VERA® Robotics Project ✨

This repository contains the proprietary ROS 2-based robotics environment, **VERA®**, including hardware abstraction layers, simulation support, behavior algorithms, and user interfaces for our platform range.

## 📂 Project Structure

- `vera_hardware/`: Hardware abstraction layer.
- `vera_simulation/`: Gazebo simulation support.
- `vera_behavior/`: Robot behavior algorithms.
- `vera_ui/`: User interfaces.
- `vera_descriptions/`: Robot descriptions and models.
- `third_party/`: External ROS packages and dependencies.
- `scripts/`: Helper scripts (building, running, deploying).
- `cyclonedds.xml`: DDS configuration file.
- `Dockerfile` : Docker images building script
- `dockercompose.yaml` : How to build and run services based on docker images 

## 🚀 Getting Started

First, clone the repository along with its submodules:

```bash
git clone https://github.com/RIILTECH/vera.git --recurse-submodules
```

### 1. Environment Setup
I assume you already have ```docker``` and ```vscode``` installed on host.
Optional for faster docker builds (if you remove you'll have to build gain)
```bash
sudo apt-get install apt-cacher-ng -y
```
Source the setup script to configure your environment:

```bash
source scripts/setup.bash
```

### 2. Build and Run

Build the Docker environment:

```bash
build_docker.bash
```

Launch the development container:

```bash
run_dev.bash
```

Join a running development container:

```bash
join_dev.bash
```

Build your workspace using colcon:

```bash
colcon build --symlink-install
source install/setup.bash
```
or use the alias ```sbuild```.

You can now launch nodes and packages within the development container on the host.

For the robot, deploy the code to the robot (or use alias ```autodeploy``` to deploy continuously):

```bash
deploy.bash
```
Ssh to the robot alias ```ssrobot``` and repeat the build over there:

### 3. Launching the complete system

*Outside of the container* set the robot model environment variable:

```bash
export VERA_MODEL=stingray
```

Launch simulation

```bash
run\_sim.bash
```

or run the real thing:
```bash
run\_hardware.bash
```
## 📜 License

This project is proprietary. © 2024 Skana Robotics Ltd. All rights reserved.

## 👋 Maintainers

For questions or feedback, please contact:

- [alex@riil.tech](mailto\:alex@riil.tech)
- [grodnay@riil.tech](mailto\:grodnay@riil.tech)

---

