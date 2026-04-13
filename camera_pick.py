import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0])  # add robot
cube = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0, 0.025])  # add cube



def move_to(target_position, steps): 
    """move the robot to the target position in a specified number of steps (steps its a speed of movement)"""
    joint_angles = p.calculateInverseKinematics(robot, 6, target_position)  # calculate joint angles to reach the target position

    for i in range(len(joint_angles)):
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition = joint_angles[i])  # main line of movement

    for x in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)

move_to([0.5, 0, 0.3], 240)

def capture_frame():
    state = p.getLinkState(robot, 6) # coordinates of 6-th joing and other its staff
    camera_position = state[4] 
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[camera_position[0],camera_position[1],camera_position[2]-0.12],
        cameraTargetPosition=[camera_position[0],camera_position[1],camera_position[2]-0.5],
        cameraUpVector=[0, 1, 0]
    )
    proj_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=640/480,
        nearVal=0.01, farVal=10.0
    )
    _, _, rgb, depth, _ = p.getCameraImage(
        640, 480,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_TINY_RENDERER
    )
    rgb_arr = np.array(rgb, dtype=np.uint8).reshape(480, 640, 4)[:, :, :3]
    depth_arr = np.array(depth, dtype=np.float32).reshape(480, 640)
    return rgb_arr, depth_arr


rgb, depth = capture_frame()
print("depth min:", depth.min())
print("depth max:", depth.max())
print("depth at cube center:", depth[240, 320])

gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)  # в оттенки серого
_, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)  # бинаризация - белое/чёрное
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # найти контуры


while True:
    p.stepSimulation()
    time.sleep(1./240.)
    
    rgb, depth = capture_frame()
    
    # детектирование глубины
    depth_threshold = depth.max() 
    cube_mask = (depth < depth_threshold).astype(np.uint8) * 255
    contours, _ = cv2.findContours(cube_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    frame = rgb.copy()   # создаём копию ОДИН раз снаружи цикла по контурам
    
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = w * h
        
        if area > 500 and abs(w - h) < 20:
            print("куб найден на позиции:", x, y, " его длина и ширина: ", w, h)
            
            center_x = x + w / 2
            center_y = y + h / 2
            
            offset_x = center_x - 320
            offset_y = center_y - 240
            
            print(f"Центр куба: ({center_x:.1f}, {center_y:.1f}) | Отклонение: ({offset_x:.1f}, {offset_y:.1f})")
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)  # центр куба
    
    # всегда показываем картинку, даже если куб не найден
    cv2.imshow("camera", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    cv2.imshow("depth_mask", cube_mask)

    if cv2.waitKey(1) == ord('q'):
        break