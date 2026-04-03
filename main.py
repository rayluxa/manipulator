import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0])  # add robot
cube = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0, 0.025])  # add cube

def wait(seconds):
    """Wait for a specified number of seconds"""
    for _ in range(int(240 * seconds)):
        p.stepSimulation()
        time.sleep(1./240.)

def move_to(target_position, steps): 
    """move the robot to the target position in a specified number of steps (steps its a speed of movement)"""
    joint_angles = p.calculateInverseKinematics(robot, 6, target_position)  # calculate joint angles to reach the target position

    for i in range(len(joint_angles)):
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition = joint_angles[i])  # main line of movement

    for x in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)

def set_joint(joint, angle, steps=300):
    """Move a specific joint to a specific angle"""
    p.setJointMotorControl2(robot, joint, p.POSITION_CONTROL, targetPosition=angle, force=500)
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)



# ----------- pick & place for cube №1 -----------
wait(1)
move_to([0.5, 0, 0.3],240)      # Move above the cube (pre-grasp position)
move_to([0.5, 0, 0.08],240)     # Move down to grasp height

constraint = p.createConstraint(robot, 6, cube, -1, p.JOINT_FIXED,[0,0,0], [0,0,0.05], [0,0,0])      # Attach cube to end-effector

move_to([0.5, 0, 0.5],240)      # Lift cube to transport height
move_to([0.5, 0.4, 0.5],240)    # Move to target location
move_to([0.5, 0.4, 0.08],240)   # Lower cube for placement

p.removeConstraint(constraint)   # Release cube

move_to([0.5, 0.4, 0.5],240)    # Retract arm upward
move_to([0, 0, 1],240)          # Move to safe/home position



# ----------- preparation for the second pick & place
set_joint(0,0)

for z in range(7):
    p.setJointMotorControl2(robot, z, p.POSITION_CONTROL, targetPosition=0, force=500) 

p.resetBasePositionAndOrientation(cube, [0.5, 0, 0.025], [0, 0, 0, 1]) # reset the cube
wait(1)

# ----------- pick & place for cube №2 -----------
move_to([0.5, 0, 0.3],60)      # Move above the cube (pre-grasp position)
move_to([0.5, 0, 0.08],60)     # Move down to grasp height

constraint = p.createConstraint(robot, 6, cube, -1, p.JOINT_FIXED,[0,0,0], [0,0,0.05], [0,0,0])  # Attach cube to end-effector

move_to([0.5, 0, 0.5],60)      # Lift cube to transport height
set_joint(0, 3.14)              # Rotate joint 0 by 180 degrees (3.14 rad)
move_to([-0.5, 0, 0.08],60)    # Move down to placement height

p.removeConstraint(constraint)  # Release cube

move_to([-0.5, 0, 0.5],60)        # Move above the cube
move_to([0, 0, 2],60)        # Retract arm to safe position

while True:
    p.stepSimulation()
    time.sleep(1./240.) 