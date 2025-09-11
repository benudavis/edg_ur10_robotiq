import numpy as np

# parameters
DIGIT_AVG_FRAMES = 3
POINTS_PER_CAPTURE = 1
GRIPPER_OFFSET = [0, 0, 0.25, 0, 0, 0]
LOW_GRASP_FORCE = 30
GRASP_FORCE = 80
TWIST_SPEED = 0.2
TIGHTENING_TORQUE = .6

POSITION_A = np.array([0.5997, -0.123, 0.25])
POSITION_B = np.array([0.5997, -0.123, 0.25])

# ranges for data collection.
# handled as offsets from the standard pose and force
X_OFFSETS = np.linspace(-0.0015, 0.0015, 3)
Y_OFFSETS = np.linspace(-0.0015, 0.0015, 3)
Z_OFFSETS = np.linspace(-0.0015, 0.0015, 3)
ANGLE_OFFSETS = np.linspace(-0.12, 0.12, 3)
FORCE_OFFSETS = np.linspace(-40, 40, 3)