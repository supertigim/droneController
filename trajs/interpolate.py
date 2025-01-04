import numpy as np
from matplotlib import pyplot as plt 

def interpolate_2d(p1, p2, distance):
    """
    Linearly interpolate between two 2D points based on distance.
    
    Parameters:
    p1 (tuple): First point (x1, y1)
    p2 (tuple): Second point (x2, y2)
    distance (float): Desired distance from p1 towards p2
    
    Returns:
    tuple: Interpolated point (x, y)
    """
    x1, y1 = p1
    x2, y2 = p2
    
    # Calculate total distance between points
    total_distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Calculate interpolation factor
    t = distance / total_distance
    
    # Ensure t is between 0 and 1
    t = max(0, min(1, t))
    
    # Interpolate x and y coordinates
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)
    
    return (x, y)


if __name__ == "__main__":
    L1 = np.array([[1540.399, 1320.082], [1601.358, -177.795], [-970.348, -154.368]]) / 1000.0
    L2 = np.array([[-970.348, -154.368], [3187.236, -205.191], [2198.592, -1947.821]]) / 1000.0

    
    # path1 = np.array([interpolate_2d(p1, p2, 0.15) for p1, p2 in L1])
    # path2 = np.array([interpolate_2d(p1, p2, 0.15) for p1, p2 in L1])

    for i, p in enumerate(L1):
        if i == 0:
            continue
        points = interpolate_2d(L1[i-1], L1[i], 0.15)
        print(points)

    # plt.plot(path1[:, 0], path1[:, 1])
    # plt.show()



