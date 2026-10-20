import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.pyplot as plt

# Lengths of the two robot arm segments (meters)
L1 = 1.0
L2 = 0.8

def forward_kinematics(theta1, theta2):
    """Calculate the end-effector position of a 2-link planar arm."""
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

# Example: joint angles in radians
theta1 = np.radians(30)
theta2 = np.radians(45)

x, y = forward_kinematics(theta1, theta2)
print(f"End-effector position: x={x:.3f}, y={y:.3f}")
  
# INVERSE KINEMATICS

def inverse_kinematics(x, y):
    """
    Calculate joint angles for a desired
    end-effector position.
    """

    # Calculate distance from base to target
    distance = np.sqrt(x**2 + y**2)

    # Check whether target is reachable
    if distance > L1 + L2 or distance < abs(L1 - L2):
        raise ValueError("Target is outside the robot's reachable workspace")

    # Calculate second joint angle
    cos_theta2 = (
        (x**2 + y**2 - L1**2 - L2**2)
        / (2 * L1 * L2)
    )

    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)

    # Calculate first joint angle
    theta1 = (
        np.arctan2(y, x)
        - np.arctan2(
            L2 * np.sin(theta2),
            L1 + L2 * np.cos(theta2)
        )
    )

    return theta1, theta2


# TEST INVERSE KINEMATICS

target_x = 1.2
target_y = 0.8

joint1, joint2 = inverse_kinematics(target_x, target_y)

print("\nInverse Kinematics Results:")
print(f"Joint 1: {np.degrees(joint1):.2f} degrees")
print(f"Joint 2: {np.degrees(joint2):.2f} degrees")

# Verify using forward kinematics
verified_x, verified_y = forward_kinematics(joint1, joint2)

print(f"Verified position: x={verified_x:.3f}, y={verified_y:.3f}")

assert np.allclose(
    [verified_x, verified_y],
    [target_x, target_y],
    atol=1e-6
)

print("Inverse kinematics verification passed!")

# TRAJECTORY PLANNING

def plan_trajectory(start, end, duration=5.0, steps=100):
    """
    Generate a smooth trajectory between two
    joint configurations using cubic interpolation.
    """

    time = np.linspace(0, duration, steps)

    # Normalized time
    t = time / duration

    # Cubic interpolation
    s = 3 * t**2 - 2 * t**3

    start = np.array(start)
    end = np.array(end)

    trajectory = (
        start[None, :]
        + s[:, None] * (end - start)[None, :]
    )

    return time, trajectory


# TEST TRAJECTORY PLANNING

start_angles = [0.0, 0.0]
end_angles = [joint1, joint2]

time, trajectory = plan_trajectory(
    start_angles,
    end_angles
)

print("\nTrajectory Planning Results:")
print(f"Generated {len(trajectory)} trajectory points")
print(f"Start angles: {trajectory[0]}")
print(f"End angles: {trajectory[-1]}")

assert np.allclose(trajectory[0], start_angles)
assert np.allclose(trajectory[-1], end_angles)

print("Trajectory verification passed!")

# ROBOTIC ARM ANIMATION

fig, ax = plt.subplots(figsize=(7, 7))

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect("equal")
ax.set_xlabel("X Position (meters)")
ax.set_ylabel("Y Position (meters)")
ax.set_title("2-Link Robotic Arm Simulation")
ax.grid(True)

# Robot arm and target
arm_line, = ax.plot([], [], "o-", linewidth=5, markersize=10)
ax.plot(target_x, target_y, "rx", markersize=12, label="Target")
ax.legend()


def update(frame):
    theta1, theta2 = trajectory[frame]

    # Elbow position
    elbow_x = L1 * np.cos(theta1)
    elbow_y = L1 * np.sin(theta1)

    # End-effector position
    end_x, end_y = forward_kinematics(theta1, theta2)

    # Update robotic arm
    arm_line.set_data(
        [0, elbow_x, end_x],
        [0, elbow_y, end_y]
    )

    return arm_line,


animation = FuncAnimation(
    fig,
    update,
    frames=len(trajectory),
    interval=50,
    blit=True
)

# Save animation as GIF
animation.save(
    "robot_arm_animation.gif",
    writer=PillowWriter(fps=20)
)

plt.close(fig)

print("Animation saved successfully!")

# PID FEEDBACK CONTROL

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.previous_actual = None

    def update(self, target, actual, dt):
        if dt <= 0:
            raise ValueError("dt must be positive")

        # Current position error
        error = target - actual

        # Integral term
        self.integral += error * dt

        # Derivative based on measured position
        if self.previous_actual is None:
            derivative = 0.0
        else:
            derivative = -(actual - self.previous_actual) / dt

        # PID control output
        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        self.previous_actual = actual

        return output

# TEST PID CONTROLLER

pid = PIDController(kp=5.0, ki=0.1, kd=0.5)

target_angle = np.radians(45)
actual_angle = np.radians(0)

dt = 0.01

actual_angles = []
target_angles = []

for i in range(500):
    control = pid.update(
        target_angle,
        actual_angle,
        dt
    )

    # Simplified joint motion model
    actual_angle += control * dt
    actual_angles.append(np.degrees(actual_angle))
    target_angles.append(np.degrees(target_angle))

print("\nPID Control Results:")
print(f"Target angle: {np.degrees(target_angle):.2f} degrees")
print(f"Actual angle: {np.degrees(actual_angle):.2f} degrees")
print(f"Final error: {np.degrees(target_angle - actual_angle):.4f} degrees")

# PID PERFORMANCE VISUALIZATION

time_pid = np.arange(len(actual_angles)) * dt

plt.figure(figsize=(10, 5))

plt.plot(
    time_pid,
    actual_angles,
    label="Actual Joint Angle",
    linewidth=2
)

plt.plot(
    time_pid,
    target_angles,
    "--",
    label="Target Joint Angle",
    linewidth=2
)

plt.xlabel("Time (seconds)")
plt.ylabel("Joint Angle (degrees)")
plt.title("PID Controller — Joint Position Tracking")
plt.legend()
plt.grid(True)

plt.savefig("pid_performance.png")
plt.close()

print("PID performance graph saved!")

# TWO-JOINT PID TRAJECTORY TRACKING

pid_joint1 = PIDController(kp=5.0, ki=0.1, kd=0.5)
pid_joint2 = PIDController(kp=5.0, ki=0.1, kd=0.5)

actual_joint_angles = np.array([0.0, 0.0])

actual_trajectory = []

dt_tracking = time[1] - time[0]

for desired_angles in trajectory:

    # Calculate control signals
    control1 = pid_joint1.update(
        desired_angles[0],
        actual_joint_angles[0],
        dt_tracking
    )

    control2 = pid_joint2.update(
        desired_angles[1],
        actual_joint_angles[1],
        dt_tracking
    )

    # Simplified joint motion model
    actual_joint_angles[0] += control1 * dt_tracking
    actual_joint_angles[1] += control2 * dt_tracking

    actual_trajectory.append(actual_joint_angles.copy())

actual_trajectory = np.array(actual_trajectory)

print("\nTwo-Joint PID Tracking Results:")

for j in range(2):
    error = np.degrees(
        trajectory[-1, j] - actual_trajectory[-1, j]
    )

    print(f"Joint {j+1} final error: {error:.4f} degrees")

# TWO-JOINT TRAJECTORY TRACKING VISUALIZATION

fig, axes = plt.subplots(2, 1, figsize=(10, 8))

for j in range(2):

    axes[j].plot(
        time,
        np.degrees(trajectory[:, j]),
        "--",
        label="Desired Angle",
        linewidth=2
    )

    axes[j].plot(
        time,
        np.degrees(actual_trajectory[:, j]),
        label="Actual Angle",
        linewidth=2
    )

    axes[j].set_title(f"Joint {j+1} Trajectory Tracking")
    axes[j].set_xlabel("Time (seconds)")
    axes[j].set_ylabel("Joint Angle (degrees)")
    axes[j].legend()
    axes[j].grid(True)

plt.tight_layout()
plt.savefig("two_joint_tracking.png")
plt.close()

print("Two-joint tracking graph saved!")

# TRAJECTORY TRACKING PERFORMANCE METRICS

tracking_error = trajectory - actual_trajectory

mae = np.mean(np.abs(tracking_error), axis=0)
rmse = np.sqrt(np.mean(tracking_error**2, axis=0))
max_error = np.max(np.abs(tracking_error), axis=0)

print("\nTrajectory Tracking Performance:")

for j in range(2):
    print(f"\nJoint {j+1}:")
    print(f"MAE: {np.degrees(mae[j]):.4f} degrees")
    print(f"RMSE: {np.degrees(rmse[j]):.4f} degrees")
    print(f"Maximum Error: {np.degrees(max_error[j]):.4f} degrees")

# PLANNED VS ACTUAL ROBOT ARM ANIMATION

fig, ax = plt.subplots(figsize=(7, 7))

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect("equal")
ax.set_xlabel("X Position (meters)")
ax.set_ylabel("Y Position (meters)")
ax.set_title("Planned vs PID-Controlled Robot Arm")
ax.grid(True)

planned_line, = ax.plot(
    [], [], "b--o", linewidth=3, label="Planned"
)

actual_line, = ax.plot(
    [], [], "r-o", linewidth=3, label="PID Controlled"
)

ax.plot(target_x, target_y, "gx", markersize=12, label="Target")
ax.legend()


def update_comparison(frame):

    planned_theta1, planned_theta2 = trajectory[frame]
    actual_theta1, actual_theta2 = actual_trajectory[frame]

    # Planned arm positions
    px = L1 * np.cos(planned_theta1)
    py = L1 * np.sin(planned_theta1)
    pend_x, pend_y = forward_kinematics(
        planned_theta1, planned_theta2
    )

    # Actual arm positions
    ax_elbow = L1 * np.cos(actual_theta1)
    ay_elbow = L1 * np.sin(actual_theta1)
    aend_x, aend_y = forward_kinematics(
        actual_theta1, actual_theta2
    )

    planned_line.set_data(
        [0, px, pend_x],
        [0, py, pend_y]
    )

    actual_line.set_data(
        [0, ax_elbow, aend_x],
        [0, ay_elbow, aend_y]
    )

    return planned_line, actual_line


comparison_animation = FuncAnimation(
    fig,
    update_comparison,
    frames=len(trajectory),
    interval=50,
    blit=True
)

comparison_animation.save(
    "planned_vs_actual.gif",
    writer=PillowWriter(fps=20)
)

plt.close(fig)

print("Planned vs actual animation saved!")