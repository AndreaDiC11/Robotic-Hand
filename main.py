import pybullet as p
import pybullet_data
import time
import math

# Initialize PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set physics engine parameters
p.setPhysicsEngineParameter(fixedTimeStep=1. / 240., numSolverIterations=150)
p.setPhysicsEngineParameter(enableConeFriction=1)

# Set gravity (disabled in this script)
# p.setGravity(0, 0, -9.81)

# Load the plane and the dual-hand robot
planeId = p.loadURDF("plane.urdf", [0, 0, -0.3])
robotStartPos = [0, 0, 0.1]
dualHandId = p.loadURDF("dual_2hand.urdf", robotStartPos, useFixedBase=True)

# Load the cube
cube_position = [1.0, -0.4, 0.5]
cube_scale = 0.8  # Cube size
cubeId = p.loadURDF("cube.urdf", cube_position, globalScaling=cube_scale, useFixedBase=False)

# Set friction for the cube
p.changeDynamics(cubeId, -1, lateralFriction=1.0)

# Get the current position of the cube
cube_position, cube_orientation = p.getBasePositionAndOrientation(cubeId)
print(f"Cube position: {cube_position}")

# Get the number of joints and create a list of all movable joints
num_joints = p.getNumJoints(dualHandId)
all_movable_joints = []
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(dualHandId, joint_index)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    # If the joint is movable (not fixed), add it to the list
    if joint_type != p.JOINT_FIXED:
        all_movable_joints.append(joint_index)

# Joints of the right arm (UR5)
right_arm_joints = [65, 66, 67, 68, 69, 70]

# Joints of the right hand
right_hand_joints = [78, 80, 81, 82, 87, 88, 89, 91, 92, 93, 99, 100, 101, 104, 105, 106]

# Joints of the right thumb
right_thumb_joints = [78, 80, 81]

# Joints of the other fingers
right_other_fingers_joints = [82, 87, 88, 89, 91, 92, 93, 99, 100, 101, 104, 105, 106]

# Initial position of the right hand
start_hand_position = [0.7, -0.2, 0.7]

# Target position of the right hand
right_hand_position = [
    cube_position[0] - 0.1,
    cube_position[1] - 0.047,
    cube_position[2] + 0.01
]

# Target orientation of the right hand
target_orientation_right = p.getQuaternionFromEuler([0, math.pi / 2, 0])

# Index of the end effector for the right hand
right_effector_index = 76  # Index of the right palm link

# Function to move the hand
def move_hand(robot_id, effector_index, position, orientation, arm_joint_indices):
    joint_positions = p.calculateInverseKinematics(
        robot_id,
        effector_index,
        position,
        orientation,
        maxNumIterations=200,
        residualThreshold=0.01
    )

    # Map joint indices to positions
    joint_positions_dict = dict(zip(all_movable_joints, joint_positions))

    # Extract positions for the arm joints
    arm_joint_positions = [joint_positions_dict.get(joint_index, 0.0) for joint_index in arm_joint_indices]

    # Control the arm joints
    for i, joint_index in enumerate(arm_joint_indices):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=arm_joint_positions[i],
            force=300,
            positionGain=2.0,
            velocityGain=1.0,
            maxVelocity=1.0
        )

# Function to interpolate positions
def interpolate_positions(start_positions, end_positions, fraction):
    return [
        start + (end - start) * fraction
        for start, end in zip(start_positions, end_positions)
    ]

# Initial positions of the fingers
thumb_start_positions = [p.getJointState(dualHandId, idx)[0] for idx in right_thumb_joints]
other_fingers_start_positions = [p.getJointState(dualHandId, idx)[0] for idx in right_other_fingers_joints]

# Target positions of the fingers
thumb_end_positions = [1.0 for _ in right_thumb_joints]
other_fingers_end_positions = [10.0 for _ in right_other_fingers_joints]

# Function to close the fingers
def close_fingers(robot_id, thumb_joints, other_fingers_joints, thumb_positions, other_fingers_positions):
    # Control the other fingers
    for idx, pos in zip(other_fingers_joints, other_fingers_positions):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=pos,
            force=500,
            positionGain=2.0,
            velocityGain=1.0,
            maxVelocity=1.0
        )
    # Control the thumb
    for idx, pos in zip(thumb_joints, thumb_positions):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=pos,
            force=10,          # Reduced force
            positionGain=0.3,   # Reduced gain
            velocityGain=1.0,
            maxVelocity=0.3     # Reduced max velocity
        )

# Simulation
for step in range(1000):
    p.stepSimulation()
    time.sleep(1. / 240.)

    # Move the hand
    fraction = min(step / 300, 1.0)
    current_hand_position = interpolate_positions(start_hand_position, right_hand_position, fraction)
    move_hand(dualHandId, right_effector_index, right_hand_position, target_orientation_right, right_arm_joints)

    # Gradual finger closure
    if step >= 300:
        finger_fraction = min((step - 300) / 200, 1.0)
        current_thumb_positions = interpolate_positions(thumb_start_positions, thumb_end_positions, finger_fraction)
        current_other_fingers_positions = interpolate_positions(other_fingers_start_positions, other_fingers_end_positions, finger_fraction)
        close_fingers(dualHandId, right_thumb_joints, right_other_fingers_joints, current_thumb_positions, current_other_fingers_positions)

print("Simulation completed. Press ESC to exit.")

# Keep the simulation window open
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)
