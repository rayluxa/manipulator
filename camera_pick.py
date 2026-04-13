import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np

# --- Simulation initialization ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0])
cube = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0, 0.025])


def move_to(target_position, steps):
    """
    Move the robot end-effector to the given world-space position using inverse kinematics.

    Args:
        target_position: [x, y, z] target position in world coordinates (meters)
        steps: number of simulation steps to run — higher value = slower, smoother movement
    """
    # Compute joint angles for all joints to reach the target position
    joint_angles = p.calculateInverseKinematics(robot, 6, target_position)

    # Send position control command to each joint
    for i in range(len(joint_angles)):
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition=joint_angles[i])

    # Step the simulation forward to let the arm physically move
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1. / 240.)


# Move arm above the cube as starting position
move_to([0.5, 0, 0.3], 240)


def capture_frame():
    """
    Capture an RGB image and depth map from a virtual camera attached to the end-effector (link 6).

    The camera is mounted slightly below the end-effector and points straight down along the Z axis.
    Uses a 60 degree FOV and 640x480 resolution.

    Returns:
        rgb_arr: (480, 640, 3) uint8 numpy array — RGB image
        depth_arr: (480, 640) float32 numpy array — depth values linearized by PyBullet
        camera_position: (x, y, z) world-space position of the end-effector link frame
    """
    # Get world-space position of link 6 (end-effector)
    state = p.getLinkState(robot, 6)
    camera_position = state[4]  # index 4 = worldLinkFramePosition

    # Camera eye is slightly below the joint, target is further down — looks straight down
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[camera_position[0], camera_position[1], camera_position[2] - 0.12],
        cameraTargetPosition=[camera_position[0], camera_position[1], camera_position[2] - 0.5],
        cameraUpVector=[0, 1, 0]
    )

    # Projection matrix: 60 degree FOV, 640x480 aspect ratio
    proj_matrix = p.computeProjectionMatrixFOV(
        fov=60,
        aspect=640 / 480,
        nearVal=0.01,
        farVal=10.0
    )

    # Render frame — returns (width, height, rgb, depth, segmentation)
    _, _, rgb, depth, _ = p.getCameraImage(
        640, 480,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_TINY_RENDERER
    )

    # Convert to numpy arrays, drop alpha channel (RGBA -> RGB)
    rgb_arr = np.array(rgb, dtype=np.uint8).reshape(480, 640, 4)[:, :, :3]
    depth_arr = np.array(depth, dtype=np.float32).reshape(480, 640)

    return rgb_arr, depth_arr, camera_position


# --- Initial test capture to verify camera setup ---
rgb, depth, _ = capture_frame()
print("depth min:", depth.min())
print("depth max:", depth.max())
print("depth at frame center:", depth[240, 320])

# Brightness-based detection test (not used in main loop)
gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
_, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


# --- Main simulation loop ---
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)

    rgb, depth, camera_position = capture_frame()

    # Depth mask: pixels closer than the background maximum = foreground object (cube)
    depth_threshold = depth.max()
    cube_mask = (depth < depth_threshold).astype(np.uint8) * 255

    # Find contours in the depth mask
    contours, _ = cv2.findContours(cube_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    frame = rgb.copy()  # copy for drawing — do not modify the source array

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = w * h

        # Filter: large enough and roughly square shape = likely the cube
        if area > 500 and abs(w - h) < 20:
            # Bounding box center in pixels
            center_x = x + w / 2
            center_y = y + h / 2

            # Pixel offset from frame center (320, 240)
            offset_x = center_x - 320
            offset_y = center_y - 240

            # Convert pixel offset to real-world meters using FOV and camera height
            # Formula: width_in_meters = 2 * tan(FOV/2) * height
            camera_h = camera_position[2] - 0.12
            width_in_meters = 2 * np.tan(np.radians(30)) * camera_h
            meters_per_pixel = width_in_meters / 640

            real_x = offset_x * meters_per_pixel
            real_y = offset_y * meters_per_pixel

            print(f"Cube found at: ({x}, {y}) | size: {w}x{h}")
            print(f"Center: ({center_x:.1f}, {center_y:.1f}) | Offset: ({offset_x:.1f}, {offset_y:.1f}) px")
            print(f"Real-world offset: ({real_x:.4f}, {real_y:.4f}) m")

            # Draw bounding box and centroid
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)

    # Display RGB frame and depth mask
    cv2.imshow("camera", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    cv2.imshow("depth_mask", cube_mask)

    # Press Q to exit
    if cv2.waitKey(1) == ord('q'):
        break