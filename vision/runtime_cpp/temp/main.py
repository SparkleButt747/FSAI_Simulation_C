import csv
import time
import subprocess
import numpy as np
import scipy.optimize as opt
from sample_data_for_centroid import generate_samples

import matplotlib.pyplot as plt


CONE_SIZE = [0.228, 0.228, 0.325] # Length Width Height
# K = R / H
Radius = CONE_SIZE[0] / 2
K = Radius / CONE_SIZE[2]

# Linear Approach
# Points: Numpy array [x, y, z]
def centroid_linear(points):
    N = points.shape[0]

    # Construct Matrices
    A = np.zeros((N - 1, 2))
    B = np.zeros((N - 1, 1))

    x1, y1, z1 = points[0]
    r1_prime_sq = (K * (CONE_SIZE[2] - z1))**2
    c1 = x1**2 + y1**2 - r1_prime_sq

    for i in range(1, N):
        xi, yi, zi = points[i]
        ri_prime_sq = (K * (CONE_SIZE[2] - zi))**2
        ci = xi**2 + yi**2 - ri_prime_sq

        A[i-1, 0] = 2 * (x1 - xi)
        A[i-1, 1] = 2 * (y1 - yi)
        B[i-1, 0] = c1 - ci
    
    # Least Squares Solution
    try:
        sol, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
        return sol.flatten()
    except np.linalg.LinAlgError:
        return None

# Non-Linear Approach
def residuals(center, points):
    [cx, cy] = center

    xi = points[:, 0]
    yi = points[:, 1]
    zi = points[:, 2]

    distances = np.sqrt((xi - cx)**2 + (yi - cy)**2)
    theo_r = K * (CONE_SIZE[2] - zi)

    return distances - theo_r

def centroid_nonlinear(points, initial):
    try:
        result = opt.least_squares(
            residuals,
            initial,
            args=(points,),
            method='lm'
        )
        if result.success:
            return result.x
        else:
            return None
    except Exception as e:
        print("Optimization Error:", e)
        return None




FEATURE_MATHCING_ERROR = 0.1 # 5 cm
if __name__ == "__main__":
    # Read Two Sides
    FILENAME = "test_cones.csv"
    Left, Right = [], []
    Left_x, Left_y, Right_x, Right_y = [], [], [], []
    with open(FILENAME, mode='r') as f:
        csv_reader = csv.reader(f)

        for row in csv_reader:
            if row[0] == "blue":
                Left_x.append(float(row[2]))
                Left_y.append(float(row[3]))
                Left.append([Left_x[-1], Left_y[-1]])
            if row[0] == "yellow":
                Right_x.append(float(row[2]))
                Right_y.append(float(row[3]))
                Right.append([Right_x[-1], Right_y[-1]])

    # Generate Mid Line
    Mid = []
    Mid_x, Mid_y = [], []
    for i in range(0, len(Left_x)):
        Mid_x.append((Left_x[i] + Right_x[i]) / 2)
        Mid_y.append((Left_y[i] + Right_y[i]) / 2)
        Mid.append([Mid_x[-1], Mid_y[-1]])

    # Generate Sample Points
    Sample_x, Sample_y, Sample_z = [], [], []
    Samples_l, Samples_r = [], []
    
    for i in range(0, len(Mid_x)):

        left = [Left_x[i], Left_y[i]]
        sample = []
        for k in generate_samples(left, sample_num=5, random_size=FEATURE_MATHCING_ERROR):
            Sample_x.append(k[0])
            Sample_y.append(k[1])
            Sample_z.append(k[2])
            sample.append([Sample_x[-1], Sample_y[-1], Sample_z[-1]])
        Samples_l.append(np.array(sample))

        right = [Right_x[i], Right_y[i]]
        sample = []
        for k in generate_samples(right, sample_num=5, random_size=FEATURE_MATHCING_ERROR):
            Sample_x.append(k[0])
            Sample_y.append(k[1])
            Sample_z.append(k[2])
            sample.append([Sample_x[-1], Sample_y[-1], Sample_z[-1]])
        Samples_r.append(np.array(sample))

    # Solution -> Linear Approach
    # Better -> Non-Linear Approach
    Solution_l, Solution_r = [], []
    Solution_x, Solution_y = [], []

    Better_l, Better_r = [], []
    Better_x, Better_y = [], []

    for i in range(len(Samples_l)):
        centroid = centroid_linear(Samples_l[i])
        if centroid is not None:
            Solution_x.append(centroid[0])
            Solution_y.append(centroid[1])
        Solution_l.append(centroid)
        better = centroid_nonlinear(Samples_l[i], centroid)
        Better_l.append(better)

    for i in range(len(Samples_r)):
        centroid = centroid_linear(Samples_r[i])
        if centroid is not None:
            Solution_x.append(centroid[0])
            Solution_y.append(centroid[1])
        Solution_r.append(centroid)
        better = centroid_nonlinear(Samples_r[i], centroid)
        Better_r.append(better)

    for i in range(len(Better_l)):
        Better_x.append(Better_l[i][0])
        Better_y.append(Better_l[i][1])
    for i in range(len(Better_r)):
        Better_x.append(Better_r[i][0])
        Better_y.append(Better_r[i][1])
    

    # Calculate Errors
    Errors_sol = []
    Errors_bet = []
    for i in range(len(Solution_l)):
        Errors_sol.append(np.sqrt((Solution_l[i][0] - Left_x[i])**2 + (Solution_l[i][1] - Left_y[i])**2))
    for i in range(len(Solution_r)):
        Errors_sol.append(np.sqrt((Solution_r[i][0] - Right_x[i])**2 + (Solution_r[i][1] - Right_y[i])**2))

    for i in range(len(Better_l)):
        Errors_bet.append(np.sqrt((Better_l[i][0] - Left_x[i])**2 + (Better_l[i][1] - Left_y[i])**2))
    for i in range(len(Better_r)):
        Errors_bet.append(np.sqrt((Better_r[i][0] - Right_x[i])**2 + (Better_r[i][1] - Right_y[i])**2))

    print("Cone", Left_x[0], Left_y[0])
    for i in Samples_l[0]:
        print("Sample Point:", i[0], i[1], i[2])
    print("Linear Solution:", Solution_x[0], Solution_y[0])
    print("Non-Linear Better:", Better_x[0], Better_y[0])
    # Start C++ test
    # process = subprocess.Popen(
    #     "./build/centroid_test",
    #     stdin=subprocess.PIPE,
    #     stdout=subprocess.PIPE,
    #     stderr=subprocess.PIPE,
    #     text=True,
    #     encoding='utf-8',
    #     bufsize=1
    # )
    # time.sleep(1)
    # process.stdin.write(f"{len(Left_x)}\n")
    # for i in range(len(Left_x)):
    #     for j in range(5):
    #         process.stdin.write(f"{Samples_l[i][j][0]} {Samples_l[i][j][1]} {Samples_l[i][j][2]}\n")
    #         process.stdin.flush()

    # process.stdin.flush()
    # CPP_x, CPP_y = [], []
    # for i in range(len(Left_x)):
    #     line = process.stdout.readline()
    #     cx, cy = map(float, line.strip().split())
    #     CPP_x.append(cx)
    #     CPP_y.append(cy)

    # process.stdout.close()
    # process.stderr.close()
    # process.wait()

    # Error_cpp = []
    # for i in range(len(Left_x)):
    #     Error_cpp.append(np.sqrt((CPP_x[i] - Left_x[i])**2 + (CPP_y[i] - Left_y[i])**2))

    # process = subprocess.Popen(
    #     "./build/centroid_test",
    #     stdin=subprocess.PIPE,
    #     stdout=subprocess.PIPE,
    #     stderr=subprocess.PIPE,
    #     text=True,
    #     encoding='utf-8',
    #     bufsize=1
    # )
    # time.sleep(1)
    # process.stdin.write(f"{len(Right_x)}\n")
    # for i in range(len(Right_x)):
    #     for j in range(5):
    #         process.stdin.write(f"{Samples_r[i][j][0]} {Samples_r[i][j][1]} {Samples_r[i][j][2]}\n")
    #         process.stdin.flush()

    # process.stdin.close()
    # for i in range(len(Right_x)):
    #     line = process.stdout.readline()
    #     cx, cy = map(float, line.strip().split())
    #     CPP_x.append(cx)
    #     CPP_y.append(cy)

    # process.stdout.close()
    # process.stderr.close()
    # process.wait()

    # Error_cpp = []
    # for i in range(len(Right_x)):
    #     Error_cpp.append(np.sqrt((CPP_x[len(Left_x)+i] - Right_x[i])**2 + (CPP_y[len(Left_x)+i] - Right_y[i])**2))

    # print("Cones_Left:", len(Left_x))
    # print("Cones_Right:", len(Right_x))
    # print("Sample Points:", len(Sample_x))
    # print("Samples/Cone: {p:.2f}".format(p=len(Sample_x) / (len(Left_x) + len(Right_x))))
    # print("Total Centroids:", len(Solution_x))
    # print("============================")
    # print("Python:")
    # print("Avg Absolute Error: {e:.4f} m".format(e=np.mean(Errors_sol)))
    # print("Max Absolute Error: {e:.4f} m".format(e=np.max(Errors_sol)))
    # print("Min Absolute Error: {e:.4f} m".format(e=np.min(Errors_sol)))
    # print("Avg Absolute Error (Better): {e:.4f} m".format(e=np.mean(Errors_bet)))
    # print("Max Absolute Error (Better): {e:.4f} m".format(e=np.max(Errors_bet)))
    # print("Min Absolute Error (Better): {e:.4f} m".format(e=np.min(Errors_bet)))
    # print("============================")
    # print("C++")
    # print("Avg Absolute Error: {e:.4f} m".format(e=np.mean(Error_cpp)))
    # print("Max Absolute Error: {e:.4f} m".format(e=np.max(Error_cpp)))
    # print("Min Absolute Error: {e:.4f} m".format(e=np.min(Error_cpp)))

    # plt.scatter(Left_x, Left_y, color='black', s=1)
    # plt.scatter(Right_x, Right_y, color='black', s=1)
    # plt.scatter(Better_x, Better_y, color='red', s=1)
    # plt.scatter(CPP_x, CPP_y, color='blue', s=1)
    # plt.axis('equal')
    # plt.show()
