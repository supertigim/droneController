import numpy as np

# Covariance for EKF simulation
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    0.08,  # variance of velocity on x-axis
    0.08,  # variance of velocity on y-axis
    0.06,  # variance of acceleration on x-axis
    0.06  # variance of acceleration on y-axis
]) ** 2  # predict state covariance
R = np.diag([0.01, 0.01]) ** 2  # Observation x,y position covariance

def motion_model(x, u, DT):
    F = np.array([[1.0, 0, 0, 0, 0.5 * DT * DT, 0],
                  [0, 1.0, 0, 0, 0, 0.5 * DT * DT],
                  [0, 0, 0.0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0]
                  ], dtype=np.float32)

    B = np.array([[DT, 0],
                  [DT, 0],
                  [1.0, 0.0],
                  [1.0, 0.0],
                  [1.0 / DT, 0.0],
                  [1.0 / DT, 0.0]
                  ], dtype=np.float32)

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0]
    ], dtype=np.float32)
    z = H @ x

    return z


def jacob_f(x, u, DT):
    """
    Jacobian of Motion Model

    motion model
    """
    vx = u[0, 0]
    vy = u[1, 0]
    ax = x[4, 0]
    ay = x[5, 0]

    jF = np.array([
        [1.0, 0.0, DT * vx, 0.0, 0.5 * DT * DT * ax, 0.0],
        [0.0, 1.0, 0.0, DT * vy, 0.0, 0.5 * DT * DT * ay],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float32)
    return jF


def jacob_h():
    jH = np.array([[1, 0, 0, 0, 0, 0],
                   [0, 1, 0, 0, 0, 0]], dtype=np.float32)
    return jH


def ekf_estimation(xEst, PEst, z, u, DT):
    #  Predict
    xPred = motion_model(xEst, u, DT)
    jF = jacob_f(xEst, u, DT)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst