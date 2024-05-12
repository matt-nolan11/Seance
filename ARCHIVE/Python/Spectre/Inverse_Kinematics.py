'''
Inverse Kinematics - Spectre 30lb
Matthew Nolan

Generalized, world-frame inverse kinematics for 4-wheel omni-wheel robot drive base
G is the center of mass, C is the geometric center of the rectangular base
Geometric parameters are specific to the 30lb version of Spectre
'''
# %%
import numpy as np
from scipy.linalg import block_diag as blkdiag  # creates block-symmetric matrices

# %%
# Geometric parameters, based on robot wheel and G positions
sigma = np.pi/180 * np.array([30.9633, 30.9633, 31.3878, 31.3878]) # acute angles of wheel position lines w.r.t. robot x-axis, in rad
beta = np.pi/180 * np.array([27, 27, 35, 35])  # acute angles of wheels w.r.t robot y-axis, in rad
r_g = np.array([[0.0916], [1.8421], [0]])  # 3D vector from C to G, in inches (assume 0 Z component)
d = [6.4678, 6.4678, 6.3779, 6.3779]  # distance from C to the center of each wheel, in inches
radius = 1.5  # wheel radii (in inches, assumed to be the same for each wheel)

# Relevant vectors constructed from above constants
r = []  # list of vectors from G to the center of each wheel
for i in range(4):
    sign = [-1 if i == 1 or i == 2 else 1, -1 if i == 2 or i == 3 else 1]
    r_i = np.array([
        [sign[0] * d[i] * np.cos(sigma[i])],
        [sign[1] * d[i] * np.sin(sigma[i])],
        [0]
    ]) - r_g
    r.append(r_i)

v = []  # list of vectors representing the direction each wheel is "pointed"
for i in range(4):
    sign = [-1 if i == 0 or i == 1 else 1, -1 if i == 1 or i == 2 else 1]
    v_i = np.array([
        [sign[0] * np.sin(beta[i])],
        [sign[1] * np.cos(beta[i])],
        [0]
    ])
    v.append(v_i)

# %%
# Constants
p = np.array([[0, 0, 1]])  # used to select the rotational component of the input vector
Q = np.array([
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, 0]
])  # used for matrix representation of cross product
Q_t = blkdiag(Q, Q, Q, Q)  # block diagonal of Q
V_hat = np.c_[v[0], v[1], v[2], v[3]].T  # column-wise stack of v vectors, transposed
V_t = blkdiag(v[0], v[1], v[2], v[3]).T  # block diagonal of the v vectors, transposed
R = np.r_[r[0], r[1], r[2], r[3]]  # row-wise stack of r vectors


# IK Function
def ik(x_hat, theta):
    """
    Calculates the inverse kinematics for a particular 4-wheel omni-drive robot.
    Returns wheel angular velocities necessary to achieve desired linear and angular robot velocities.

    Parameters:
    x_hat: (3x1) ndarray of floats representing the desired world-frame x, y, and angular velocities in inches/sec and rad/s
    theta: float representing the current angle (in radians) of the robot w.r.t the world frame

    Outputs:
    omega: (4x1) ndarray representing the necessary wheel angular velocities to achieve the specified motion
    """
    W = np.array([
        [np.cos(theta), np.sin(theta), 0],
        [-np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])
    omega = 1 / radius * (V_hat @ W @ x_hat + V_t @ (p @ W @ x_hat * Q_t @ R))
    return omega