import numpy as np
import scipy
import scipy.linalg

# Σ0 = diagm([1e-4; 1e-4; 1e-6])
# R = diagm([1e-2; 1e-2]) # Sensor covariances (m^2, m^2) -- these should come from GPS reported accuracy
# Q = diagm([1e-4; 1e-4; 1e-2; 1e-2; 2.5e-1]) # Process covariances (m^2/s, m^2/s, rad^2/s, rad^2/s, (m/s)^2/s)
#                               # ^ the process covariances are timestep size dependent


# Given a mean and covariance in N-dimensional space, generate 2N+1 weighted points
# with the given weighted mean and weighted covariance
def generate_sigma_points(x_hat, Sigma):
    Nx = len(x_hat)
    A = scipy.linalg.sqrtm(Sigma)
    sigma = np.zeros((Nx, 2 * Nx + 1))
    W = np.zeros((2 * Nx + 1))
    W[0] = 1 / 3

    sigma[:, 0] = x_hat

    # TODO: terms in A could be complex due to non-SPD Sigma, could handle that by symmetrizing R and using Choleskty
    for j in range(Nx):
        sigma[:, 1 + j] = x_hat + np.sqrt(Nx / (1 - W[0])) * A[:, j]

    for j in range(Nx):
        sigma[:, 1 + Nx + j] = x_hat - np.sqrt(Nx / (1 - W[0])) * A[:, j]

    W[1:] = (1 - W[0]) / (2 * Nx)

    return sigma, W


# maps vector in state space to vector in measurement space
# g, "GPS" measurement of positions
def measurement(x):
    y = x[0:2]
    return y


# Given a state estimate and covariance, apply nonlinear dynamics over dt to sigma points
# and calculate a new state estimate and covariance
def ukf_predict(dynamics, x_hat_curr, Sigma_curr, Q, u_curr, dt, params):
    Nx = len(x_hat_curr)
    sigma, W = generate_sigma_points(x_hat_curr, Sigma_curr)

    for k in range(2 * Nx + 1):
        sigma[:, k] = dynamics(sigma[:, k], u_curr, params, dt)

    x_hat_next = np.zeros((Nx,))
    Sigma_next = np.zeros((Nx, Nx))

    for k in range(2 * Nx + 1):
        x_hat_next += W[k] * sigma[:, k]

    for k in range(2 * Nx + 1):
        Sigma_next += W[k] * (
            (sigma[:, k] - x_hat_next)[:, np.newaxis] @ np.transpose((sigma[:, k] - x_hat_next)[:, np.newaxis])
        )

    Sigma_next += Q * dt

    return x_hat_next, Sigma_next


# Given a state estimate, covariance of the state estimate, measurement and covariance of the measurement,
# apply the measurement function to sigma points,
# calculate the mean and covariance in measurement space, then use this to calculate the Kalman gain,
# then use the gain and measurement to calculate the updated state estimate and covariance.
# returns updated x_hat, Sigma, and a debug dictionary for publish topics
def ukf_update(x_hat, Sigma, y, R):
    Nx = len(x_hat)
    Ny = len(y)
    singular_flag = False
    # 1e-9 is a hardcoded threshhold, based on the fact that values around 1e-5 work
    add_term = 1e-9
    while (abs(np.linalg.det(Sigma)) <= 1e-9):
        Sigma += np.eye(Sigma.shape[0]) * add_term
        add_term *= 2
        singular_flag = True

    sigma_points, W = generate_sigma_points(x_hat, Sigma)

    z = np.zeros((Ny, 2 * Nx + 1))

    for k in range(2 * Nx + 1):
        z[:, k] = measurement(sigma_points[:, k])

    z_hat = np.zeros((Ny))
    S = np.zeros((Ny, Ny))
    Cxz = np.zeros((Nx, Ny))

    for k in range(2 * Nx + 1):
        z_hat += W[k] * z[:, k]

    for k in range(2 * Nx + 1):
        S += W[k] * (z[:, k] - z_hat)[:, np.newaxis] @ np.transpose((z[:, k] - z_hat)[:, np.newaxis])
        Cxz += W[k] * (sigma_points[:, k] - x_hat)[:, np.newaxis] @ np.transpose((z[:, k] - z_hat)[:, np.newaxis])

    S += R
    K = Cxz @ np.linalg.inv(S)

    x_hat_next = x_hat + K @ (y - z_hat)
    Sigma_next = Sigma - K @ S @ np.transpose(K)
    debug_info = {"S": S, "singular_flag": singular_flag}

    return x_hat_next, Sigma_next, debug_info
