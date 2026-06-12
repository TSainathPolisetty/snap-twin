# SO-101 Digital Twin: Standalone Simulation Setup

This project provides a **standalone Digital Twin** for the SO-101 robotic arm. It uses a custom Python-based WebSocket bridge to stream high-fidelity 3D data and coordinate transforms directly to Foxglove, bypassing the need for a full ROS 2 or Gazebo installation.

This setup is designed for embedded systems engineers who need a lightweight, high-performance visualization dashboard without the overhead of a full robotics stack.

---

## Prerequisites

### Hardware
- Rubik Pi (or any Ubuntu-based SBC)

### Network
- Ensure the Pi and your laptop are on the same local network

### Software
- On the Pi: [Miniconda](https://docs.conda.io/en/latest/miniconda.html) or Anaconda
- On your Laptop: [Foxglove Desktop](https://foxglove.dev/download)

---

## From-Scratch Installation

Follow these steps to set up the environment and all required dependencies on a fresh Ubuntu system.

### 1. System Utilities

Install `tmux` to manage multiple server processes and `build-essential` for compiling math libraries.

```bash
sudo apt update && sudo apt install -y tmux build-essential
```

### 2. Conda Environment
Create an isolated environment to manage specific versions of Python and its dependencies.

```bash
# Create the environment with Python 3.10
conda create -n snap-twin python=3.10 -y

# Activate the environment
conda activate snap-twin
```

### 3. Python Packages
Install the core stack for 3D kinematics and communication:

```bash
pip install foxglove-sdk yourdfpy scipy numpy pyyaml
```

#### Package Overview

- `foxglove-sdk` — Handles the WebSocket communication protocol  
- `yourdfpy` — Loads URDF models and calculates forward kinematics  
- `scipy` — Manages 3D rotation math (quaternions)  
- `numpy` — Numerical computation support  
- `pyyaml` — Required to parse URDF and robot configuration files  

---

## Project Structure

Ensure your `test-base` folder is organized as follows:

| File | Purpose |
|------|----------|
| `starts_twin.sh` | Automation bootstrap script. Starts tmux and launches servers. |
| `viz_server.py` | Logic engine. Calculates joint positions and streams to port 8765. |
| `mesh_server.py` | Asset server. Serves `.stl` files over HTTP on port 8000. |
| `final_browser.urdf` | Optimized URDF with absolute `http` paths for Foxglove. |
| `assets/` | Directory containing all `.stl` mesh files. |

## Launching the Twin

### 1. SSH into your Pi

```bash
ssh ubuntu@192.168.0.101
```

---

### 2. Run the Bootstrap Script

```bash
cd ~/demo/snap-twin/test-base
chmod +x starts_twin.sh
./starts_twin.sh
```

This automatically starts a `tmux` session with two panes:

- **Left Pane** — Asset Server (port 8000)
- **Right Pane** — Logic Bridge (port 8765)


## Foxglove Configuration

1. Open Foxglove Desktop on your laptop.
2. Click **Open Connection** and select **Foxglove WebSocket**.
3. Enter:

   ```
   ws://192.168.0.101:8765
   ```

4. In the **3D Panel** settings:

   - **URDF Source**: URL  
   - **URL**:

     ```
     http://192.168.0.101:8000/final_browser.urdf
     ```

   - **Fixed Frame**: `world`

The fixed frame must be set to `world` to display the table and floor correctly.
