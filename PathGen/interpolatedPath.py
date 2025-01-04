import numpy as np

def generate_interpolated_points(p1, p2, num_points):
    p1 = np.array(p1)
    p2 = np.array(p2)
    
    # Generate evenly spaced values from 0 to 1
    t = np.linspace(0, 1, num_points)
    
    # Interpolate between p1 and p2
    interpolated_points = p1[np.newaxis, :] * (1 - t)[:, np.newaxis] + p2[np.newaxis, :] * t[:, np.newaxis]
    
    return interpolated_points.tolist()

L1 = np.array([[1540.399, 1320.082], [1601.358, -177.795], [-970.348, -154.368]]) / 1000.0
L2 = np.array([[-970.348, -154.368], [3187.236, -205.191], [2198.592, -1947.821]]) / 1000.0

def main(path, V):
    interpolated_path = []
    
    for i, p in enumerate(path):
        if i == 0:
            continue
        N = int(np.linalg.norm(path[i-1] - path[i]) / V)
        points = generate_interpolated_points(path[i-1], path[i], N)
        interpolated_path.extend(points)
    return interpolated_path

if __name__=="__main__":
    dt = 0.03  # s
    v = 2 # m/s
    path = main(L2, v * dt) 

    np.savetxt('L2.csv', path, delimiter=',')

    print(len(path))

    # from pprint import pprint
    # pprint(path)