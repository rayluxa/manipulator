# Robot Arm Simulation — PyBullet

A simulation of a Kuka IIWA robotic arm performing pick and place tasks,
built as a stepping stone toward a real physical robot arm controlled
by voice commands and computer vision.

## What it does

- Loads a Kuka IIWA 7-DOF robot arm in a PyBullet physics simulation
- Uses Inverse Kinematics (IK) to move the end-effector to target positions
- Performs two pick and place sequences:
  - Standard lateral placement
  - 180° base rotation placement

## Requirements

- Python 3.8+
- pybullet
- pybullet_data (included with pybullet)

## Installation

pip install pybullet

## Run

python simulation.py

## Controls (PyBullet viewer)

- Ctrl + Left Mouse — rotate camera
- Ctrl + Right Mouse — zoom
- Ctrl + Middle Mouse — pan

## Roadmap

- [x] Add virtual camera to the simulation
- [x] Detect cube position using segmentation
- [ ] Add voice command parsing (Whisper)
- [ ] Connect to physical robot arm
