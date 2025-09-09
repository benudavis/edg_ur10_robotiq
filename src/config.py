import numpy as np

# parameters
DIGIT_AVG_FRAMES = 1
POINTS_PER_CAPTURE = 5
GRIPPER_OFFSET = [0, 0, 0.25, 0, 0, 0]
GRASP_FORCE = 40
TWIST_SPEED = 0.2
TIGHTENING_TORQUE = .5

POSITION_A = np.array([0.5959, -0.123, 0.25])
POSITION_B = np.array([0.5959, -0.123, 0.25])

# ranges for data collection.
# handled as offsets from the standard pose and force
X_OFFSETS = np.linspace(-0.001, 0.001, 3)
Y_OFFSETS = np.linspace(-0.001, 0.001, 3)
Z_OFFSETS = np.linspace(-0.0005, 0.0005, 3)
ANGLE_OFFSETS = np.linspace(-0.05, 0.05, 3)
FORCE_OFFSETS = np.linspace(-20, 20, 5)