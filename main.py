import pybullet as p
import pybullet_data
import time
import math

# Inizializzazione PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Imposta parametri del motore fisico
p.setPhysicsEngineParameter(fixedTimeStep=1. / 240., numSolverIterations=150)
p.setPhysicsEngineParameter(enableConeFriction=1)

# Imposta la gravità
#p.setGravity(0, 0, -9.81)

# Caricamento del piano e del robot a due mani
planeId = p.loadURDF("plane.urdf", [0, 0, -0.3])
robotStartPos = [0, 0, 0.1]
dualHandId = p.loadURDF("dual_2hand.urdf", robotStartPos, useFixedBase=True)

# Caricamento del cubo
cube_position = [1.0, -0.4, 0.5]
cube_scale = 0.9  # Dimensione del cubo
cubeId = p.loadURDF("cube.urdf", cube_position, globalScaling=cube_scale, useFixedBase=False)

# Imposta l'attrito per il cubo
p.changeDynamics(cubeId, -1, lateralFriction=1.0)

# Ottieni la posizione attuale del cubo
cube_position, cube_orientation = p.getBasePositionAndOrientation(cubeId)
print(f"Posizione del cubo: {cube_position}")

# Controlla il numero di giunti e crea una lista di tutti i giunti mobili
num_joints = p.getNumJoints(dualHandId)
all_movable_joints = []
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(dualHandId, joint_index)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    # Se il giunto è mobile (non fisso), aggiungilo alla lista
    if joint_type != p.JOINT_FIXED:
        all_movable_joints.append(joint_index)

# Giunti del braccio destro (UR5)
right_arm_joints = [65, 66, 67, 68, 69, 70]

# Giunti della mano destra
right_hand_joints = [78, 80, 81, 82, 87, 88, 89, 91, 92, 93, 99, 100, 101, 104, 105, 106]

# Giunti del pollice destro
right_thumb_joints = [78, 80, 81]

# Giunti delle altre dita
right_other_fingers_joints = [82, 87, 88, 89, 91, 92, 93, 99, 100, 101, 104, 105, 106]


# Posizione iniziale della mano destra
start_hand_position = [0.7, -0.2, 0.7]

# Posizione target della mano destra
right_hand_position = [
    cube_position[0] - 0.1,
    cube_position[1] - 0.053,
    cube_position[2] + 0.01
]

# Orientazione target della mano destra
target_orientation_right = p.getQuaternionFromEuler([0, math.pi / 2, 0])

# Indice dell'effettore finale per la mano destra
right_effector_index = 76  # Indice del link del palmo della mano destra

# Funzione per muovere la mano
def move_hand(robot_id, effector_index, position, orientation, arm_joint_indices):
    joint_positions = p.calculateInverseKinematics(
        robot_id,
        effector_index,
        position,
        orientation,
        maxNumIterations=200,
        residualThreshold=0.01
    )

    # Mappatura degli indici dei giunti e delle posizioni
    joint_positions_dict = dict(zip(all_movable_joints, joint_positions))

    # Estrai le posizioni per i giunti del braccio
    arm_joint_positions = [joint_positions_dict.get(joint_index, 0.0) for joint_index in arm_joint_indices]

    # Controllo dei giunti del braccio
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

# Funzione per interpolare le posizioni
def interpolate_positions(start_positions, end_positions, fraction):
    return [
        start + (end - start) * fraction
        for start, end in zip(start_positions, end_positions)
    ]

# Posizioni iniziali delle dita
thumb_start_positions = [p.getJointState(dualHandId, idx)[0] for idx in right_thumb_joints]
other_fingers_start_positions = [p.getJointState(dualHandId, idx)[0] for idx in right_other_fingers_joints]

# Posizioni target finali delle dita
thumb_end_positions = [1.0 for _ in right_thumb_joints]
other_fingers_end_positions = [1.0 for _ in right_other_fingers_joints]

# Funzione per chiudere le dita
def close_fingers(robot_id, thumb_joints, other_fingers_joints, thumb_positions, other_fingers_positions):
    # Controllo delle altre dita
    for idx, pos in zip(other_fingers_joints, other_fingers_positions):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=pos,
            force=300,
            positionGain=2.0,
            velocityGain=1.0,
            maxVelocity=1.0
        )
    # Controllo del pollice
    for idx, pos in zip(thumb_joints, thumb_positions):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=pos,
            force=150,          # Forza ridotta
            positionGain=0.5,   # Guadagno ridotto
            velocityGain=1.0,
            maxVelocity=0.3     # Velocità massima ridotta
        )

def create_grasp_constraint(robot_id, effector_index, object_id):
    # Crea un vincolo fisso tra la mano e il cubo
    constraint_id = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=effector_index,
        childBodyUniqueId=object_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
        )
    return constraint_id


# Simulazione
for step in range(600):
    p.stepSimulation()
    time.sleep(1. / 240.)

    # Movimento della mano
    fraction = min(step / 300, 1.0)
    current_hand_position = interpolate_positions(start_hand_position, right_hand_position, fraction)
    move_hand(dualHandId, right_effector_index, right_hand_position, target_orientation_right, right_arm_joints)

    # Chiusura graduale delle dita
    if step >= 300:
        finger_fraction = min((step - 300) / 200, 1.0)
        current_thumb_positions = interpolate_positions(thumb_start_positions, thumb_end_positions, finger_fraction)
        current_other_fingers_positions = interpolate_positions(other_fingers_start_positions, other_fingers_end_positions, finger_fraction)
        close_fingers(dualHandId, right_thumb_joints, right_other_fingers_joints, current_thumb_positions, current_other_fingers_positions)
    #if step >= 500:
        #create_grasp_constraint(dualHandId, right_effector_index, cubeId)

print("Simulazione completata.")


# Disconnessione
p.disconnect()
