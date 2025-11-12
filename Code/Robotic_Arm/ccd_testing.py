import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ----- Arm parameters -----
segment_lengths = [11, 9.25]  # Upper arm, forearm
num_joints = 4  # 3 shoulder + 1 elbow

# Initialize joint angles: [yaw, pitch, abduction, elbow]
joint_angles = np.array([0.0, 0.0, 0.0, 0.0])

# Target position
target = np.array([2.0, 1.5, 1.0])

# CCD parameters
max_iterations = 50
tolerance = 0.05

# ----- Rotation matrix function using Rodrigue's Rotation Formula -----
def rotation_matrix(axis, angle):
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return R

# ----- Forward kinematics -----
# Computes 3D Positions of shoulder, elbow, and hand
def forward_kinematics(joint_angles):
    shoulder_pos = np.array([0.0, 0.0, 0.0])
    
    # Shoulder rotations
    R_yaw = rotation_matrix(np.array([0,0,1]), joint_angles[0])        # vertical axis
    R_pitch = rotation_matrix(np.array([1,0,0]), joint_angles[1])      # forward/back lift
    R_abduction = rotation_matrix(np.array([0,1,0]), joint_angles[2])  # side lift
    R_shoulder = R_yaw @ R_pitch @ R_abduction
    
    # Elbow flexion (around local y-axis of upper arm)
    R_elbow = rotation_matrix(np.array([0,1,0]), joint_angles[3])
    
    # Upper arm
    elbow_pos = shoulder_pos + R_shoulder @ np.array([segment_lengths[0],0,0])
    
    # Forearm
    hand_pos = elbow_pos + (R_shoulder @ R_elbow) @ np.array([segment_lengths[1],0,0])
    
    return [shoulder_pos, elbow_pos, hand_pos]

# ----- CCD step -----
def ccd_step(joint_angles, target):

    # Get positions and end_effector (hand position)
    positions = forward_kinematics(joint_angles)
    end_effector = positions[-1]
    
    # In reverse to start at elbow (CCD starts at end effector)
    for i in reversed(range(num_joints)):
        joint_pos = positions[0] if i < 3 else positions[1]  # Elbow, then shoulder 3

        # Vectors to current and target hand position
        vec_to_end = end_effector - joint_pos
        vec_to_target = target - joint_pos

        # Checking if vector is 0 to avoid divide by 0
        if np.linalg.norm(vec_to_end) < 1e-6 or np.linalg.norm(vec_to_target) < 1e-6:
            continue

        vec_to_end /= np.linalg.norm(vec_to_end)
        vec_to_target /= np.linalg.norm(vec_to_target)
        
        axis = np.cross(vec_to_end, vec_to_target)
        if np.linalg.norm(axis) < 1e-6:
            continue
        axis /= np.linalg.norm(axis)
        angle = np.arccos(np.clip(np.dot(vec_to_end, vec_to_target), -1.0, 1.0))
        angle *= 0.5  # small step for stability
        
        if i == 0:  # Yaw (z-axis)
            joint_angles[i] += angle * np.sign(np.dot(axis, np.array([0,0,1])))
        elif i == 1:  # Pitch (x-axis)
            joint_angles[i] += angle * np.sign(np.dot(axis, np.array([1,0,0])))
        elif i == 2:  # Abduction (y-axis)
            joint_angles[i] += angle * np.sign(np.dot(axis, np.array([0,1,0])))
        else:  # Elbow flexion
            joint_angles[i] += angle * np.sign(axis[1])
        
        # Updates positions and end effector after each joint movement
        positions = forward_kinematics(joint_angles)
        end_effector = positions[-1]

        # Check if we've gotten close enough
        if np.linalg.norm(end_effector - target) < tolerance:
            break
    return joint_angles, positions

# ----- Persistent visualization -----
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-25,25)
ax.set_ylim(-25,25)
ax.set_zlim(0,25)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# ----- Main loop -----
while True:
    user_input = input("\nEnter new target position as x y z (or 'q' to quit): ")
    if user_input.lower() == 'q':
        print("Exiting...")
        break
    try:
        target = np.array(list(map(float, user_input.split())))
        if len(target) != 3:
            print("Please enter exactly three numbers separated by spaces (x y z).")
            continue

        # Run CCD and visualize updates in same window
        for iteration in range(max_iterations):
            joint_angles, positions = ccd_step(joint_angles, target)
            xs = [p[0] for p in positions]
            ys = [p[1] for p in positions]
            zs = [p[2] for p in positions]

            ax.cla()  # clear old arm drawing but keep window open
            ax.plot(xs, ys, zs, '-o', linewidth=3, markersize=8)
            ax.scatter(target[0], target[1], target[2], c='r', s=100, marker='x')
            ax.set_xlim(-25,25)
            ax.set_ylim(-25,25)
            ax.set_zlim(0,25)
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_title(f'Target: {np.round(target, 2)} | Iter {iteration+1}')
            plt.pause(0.05)

            if np.linalg.norm(positions[-1] - target) < tolerance:
                print(f"Target reached in {iteration+1} iterations!")
                break

    except ValueError:
        print("Invalid input. Example: 6.0 7.0 10.0")

plt.ioff()
plt.show()
