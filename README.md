# Simulation of Robotic Manipulation with Dual-Hand Robots

This project demonstrates the simulation of dual-hand robotic manipulation using the PyBullet physics engine. The system addresses challenges such as inverse kinematics, joint coordination, and stable grasping for effective object manipulation.

## Features
- **Inverse Kinematics**: Computes joint angles for precise end-effector positioning.
- **Joint Coordination**: Ensures smooth arm and finger movements.
- **Stable Grasping**: Implements controlled finger closure for secure object handling.

## Methodology
1. **Initialization**: Sets up the robot, environment, and object parameters in PyBullet.
2. **Control Strategies**: Utilizes position control for seamless motion and interpolation.
3. **Simulation Execution**: Monitors object manipulation for robust performance.

## Results
The project achieves:
- Accurate positioning of the robotic end-effector.
- Effective grasping and manipulation of objects.
- Smooth and coordinated joint movements.

## Requirements
- Python 3.x
- gym 0.26.2
- PyBullet 3.26

## Usage
1. Clone the repository.
2. Install dependencies: `pip install pybullet`.
3. Run the simulation: `python simulation.py`.

## Future Improvements
- Refine control parameters for finger movements.
- Enhance motion interpolation for smoother transitions.
