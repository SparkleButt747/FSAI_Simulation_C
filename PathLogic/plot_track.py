import matplotlib.pyplot as plt
import pandas as pd

def main():
    # Load CSV data
    left_df = pd.read_csv('../build/left_cones.csv')
    right_df = pd.read_csv('../build/right_cones.csv')
    checkpoints_df = pd.read_csv('../build/checkpoints.csv')
    
    plt.figure(figsize=(8, 8))
    
    # Plot left cones (blue), right cones (green), and checkpoints (red)
    plt.scatter(left_df['x'], left_df['z'], color='blue', label='Left Cones')
    plt.scatter(right_df['x'], right_df['z'], color='green', label='Right Cones')
    plt.scatter(checkpoints_df['x'], checkpoints_df['z'], color='red', label='Checkpoints')
    
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.title('Track Cone and Checkpoint Positions')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    main()
