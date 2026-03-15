import numpy as np

# Σ0 = diagm([1e-4; 1e-4; 1e-6])
# R = diagm([1e-2; 1e-2]) # Sensor covariances (m^2, m^2) -- these should come from GPS reported accuracy
# Q = diagm([1e-4; 1e-4; 1e-2; 1e-2; 2.5e-1]) # Process covariances (m^2/s, m^2/s, rad^2/s, rad^2/s, (m/s)^2/s)
#                               # ^ the process covariances are timestep size dependent


# Given a mean and covariance in N-dimensional space, generate 2N+1 weighted points
# with the given weighted mean and weighted covariance
def generate_sigma_points(x_hat, Sigma, Sigma_init):
    Nx = len(x_hat)
    # Symmetrize Sigma to ensure it is symmetric before Cholesky (S: Symmetric)
    Sigma = (Sigma + Sigma.T) / 2
    # Use Cholesky decomposition to get the square root of Sigma, needs SPD matrix; faster than scipy.linalg.sqrtm
    singular_flag = False
    try:
        A = np.linalg.cholesky(Sigma)
    except np.linalg.LinAlgError:
        singular_flag = True
        # Add a small hardcoded value 1e-9 to the diagonal to ensure PD (Positive Definite)
        jitter = 1e-9 * np.eye(Nx)
        Sigma += jitter
        try:
            A = np.linalg.cholesky(Sigma)
        except np.linalg.LinAlgError:
            # Sigma is not positive definite, even after adding jitter
            # Re-initialize Sigma to init_Sigma and compute A again
            Sigma = Sigma_init
            try:
                A = np.linalg.cholesky(Sigma)
            except np.linalg.LinAlgError:
                raise ValueError("Failed to compute Cholesky decomposition even after re-initialization.")

    sigma = np.zeros((Nx, 2 * Nx + 1))
    W = np.zeros((2 * Nx + 1))
    W[0] = 1 / 3

    sigma[:, 0] = x_hat

    for j in range(Nx):
        sigma[:, 1 + j] = x_hat + np.sqrt(Nx / (1 - W[0])) * A[:, j]

    for j in range(Nx):
        sigma[:, 1 + Nx + j] = x_hat - np.sqrt(Nx / (1 - W[0])) * A[:, j]

    W[1:] = (1 - W[0]) / (2 * Nx)

    return sigma, Sigma, W, singular_flag


# maps vector in state space to vector in measurement space
# g, "GPS" measurement of positions
def measurement(x):
    y = x[0:2]
    return y


# Given a state estimate and covariance, apply nonlinear dynamics over dt to sigma points
# and calculate a new state estimate and covariance, along with a singular flag
def ukf_predict(dynamics, x_hat_curr, Sigma_curr, Sigma_init, Q, u_curr, dt, params):
    Nx = len(x_hat_curr)
    sigma, _, W, singular_flag = generate_sigma_points(x_hat_curr, Sigma_curr, Sigma_init)

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

    return x_hat_next, Sigma_next, singular_flag


# Given a state estimate, covariance of the state estimate, measurement and covariance of the measurement,
# apply the measurement function to sigma points,
# calculate the mean and covariance in measurement space, then use this to calculate the Kalman gain,
# then use the gain and measurement to calculate the updated state estimate and covariance.
# returns updated x_hat, Sigma, and a singular flag
def ukf_update(x_hat, Sigma, Sigma_init, y, R):
    Nx = len(x_hat)
    Ny = len(y)
    singular_flag = False
    # Decided against this check, per discussion: https://discord.com/channels/1114989213230825492/1482487961382686781/1482500228526506055
    # A determinant-based PD check is not realiable and depends on the size of the state space and the scale of the values in Sigma;
    # This approach inflates the diagonal values of Sigma much more than necessary, leading to worse estimation performance
    # 1e-9 is a hardcoded threshold, based on the fact that values around 1e-5 work
    # add_term = 1e-9
    # while (abs(np.linalg.det(Sigma)) <= 1e-9):
    #     Sigma += np.eye(Sigma.shape[0]) * add_term
    #     add_term *= 2
    #     if add_term > 1e-6:
    #         raise ValueError("Sigma is not positive definite, even after adding significant jitter.")
    #     singular_flag = True

    sigma_points, Sigma, W, singular_flag = generate_sigma_points(x_hat, Sigma, Sigma_init) # sets Sigma to generate_sigma_points's jittered/re-initialized Sigma

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

    return x_hat_next, Sigma_next, singular_flag
