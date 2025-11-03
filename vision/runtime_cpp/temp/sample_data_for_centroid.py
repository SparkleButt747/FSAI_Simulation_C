import csv
import random
import numpy as np
import matplotlib.pyplot as plt

# Points are in format [x, y, z] where z is height
CONE_SIZE = [0.228, 0.228, 0.325]

def generate_samples(center_pos, sample_num=5, random_size=0.05):
    '''
    Generate sample points around the cone surface

    :param list center_pos: 2D coordinates of the cone center
    :param int sample_num: number of sample points to generate
    :param float random_size: maximum random offset to add to each sample point
    :return samples: List of sample points, each is [x, y, z]
    '''
    samples = []

    [cx, cy] = center_pos
    radius = CONE_SIZE[0] / 2
    H = CONE_SIZE[2]
    k = radius / H

    for _ in range(sample_num):
        z_perfect = random.uniform(0.05, 0.95) * H
        r_perfect = k * (H - z_perfect)
        angle = random.uniform(0, 2 * np.pi)

        x_perfect = cx + r_perfect * np.cos(angle)
        y_perfect = cy + r_perfect * np.sin(angle)

        x = x_perfect + random.uniform(-random_size, random_size)
        y = y_perfect + random.uniform(-random_size, random_size)
        z = z_perfect + random.uniform(-random_size, random_size)

        z = np.clip(z, 0, H)
        samples.append([x, y, z])

    return samples


# For test usage
if __name__ == "__main__":
    FILENAME = "test_cones.csv"

    # Read two sides of cones
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

    # Generate sample points
    Sample_x, Sample_y, Sample_z = [], [], []
    for i in range(0, len(Left_x)):
        left = [Left_x[i], Left_y[i]]
        for k in generate_samples(left, sample_num=random.randint(3,7)):
            Sample_x.append(k[0])
            Sample_y.append(k[1])
            Sample_z.append(k[2])

        right = [Right_x[i], Right_y[i]]
        for k in generate_samples(right, sample_num=random.randint(3,7)):
            Sample_x.append(k[0])
            Sample_y.append(k[1])
            Sample_z.append(k[2])

    # Plotting
    plt.scatter(Sample_x, Sample_y, color='purple', s=1)
    plt.scatter(Left_x, Left_y, color='black', s=1)
    plt.scatter(Right_x, Right_y, color='black', s=1)
    plt.axis('equal')
    plt.show()
