# Centroid Recovery

From the pipeline, there are several sample points generated from image recognition (yolov8) and feature matching. The task here is to calculate the origin of the bottom circle of the cone, with these sample points.

For the implementation so far, it is assumed that the cone is perpendicular to the ground and the latter is completely flat.

Below the $z$ axis is considered to be the one perpendicular to the ground.

## Approch 1 - Linear Method
It is known that the cone's radius and height $R=0.114m$, $H=0.325m$  
We also have multiple sample points $(x_i,y_i,z_i)$ from previous pipeline (3~5 points), but they are not perfectly on the surface.

The initial idea is to use similar triangles.  
For a point on the cone surface $(x_i,y_i,z_i)$, the radius of that point satisfies
$$\frac{r_i}{R}=\frac{H-z_i}{H}$$

Hence ($X_c,Y_c$ is the 2D coordinate of that origin)
$$(x-X_c)^2+(y-Y_c)^2=r_i^2$$

Here we need to find $X_c$ and $Y_c$, so
$$2x_iX_c+2y_iY_c+(r_i^2-X_c^2-Y_c^2-x_i^2-y_i^2)=0$$

The equation is non-linear, but if we substract two equations from seperate points it yields
$$2(x_1-x_2)X_c+2(y_1-y_2)Y_c=(x_1^2+y_1^2-r_1^2)-((x_2^2+y_2^2-r_2^2))$$

And it's linear, which is easy to solve $x=[X_c,Y_c]^T$  
So for $N$ sample points, by substracting we got $N-1$ linear equations $x\cdot A=b$, where $A$ is a matrix of $(N-1)\times 2$ and $b$ is a vector of $(N-1)\times 1$  
When $N=3$ we can just solve it and when $N>3$ it's an overdtermined system, we can solve this with a simple least squares method.  
The solution now is $x=(A^TA)^{-1}A^Tb$

- Benefit: Fast, one solution, no need to make an initial guess
- Drawback: When the points are close each other the noise may be amplified

Here is a python implementation:
```python
import numpy as np

CONE_SIZE = [0.228, 0.228, 0.325]
# K = R / H
Radius = CONE_SIZE[0] / 2
Height = CONE_SIZE[2]
K = Radius / Height

# Points: Numpy Array [x, y, z]
def centroid_linear(points):
    if len(points) < 3:
        return None
    
    N = points.shape[0]
    # Construct Matrices
    A = np.zeros((N - 1, 2))
    b = np.zeros((N - 1, 1))

    x1, y1, z1 = points[0]
    r1_prime_sq = (K * (Height - z1))**2
    c1 = x1**2 + y1**2 - r1_prime_sq

    for i in range(1, N):
        xi, yi, zi = points[i]
        ri_prime_sq = (K * (Height - zi))**2
        ci = xi**2 + yi**2 - ri_prime_sq

        A[i-1, 0] = 2 * (x1 - xi)
        A[i-1, 1] = 2 * (y1 - yi)
        b[i-1, 0] = c1 - ci
    
    # Least Squares Solution
    try:
        sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        return sol.flatten()
    except np.linalg.LinAlgError:
        return None
```

It turns out that this approach performs awfully when there are few samples (n<5) and cannot handle existing errors.

## Approach 2 - Non-linear Approach
This approach is to directly solve errors.  
Initially, we have to define a residual function to calculate the degree of fit between the model $(X_c,Y_c)$ and the real point $(x_i,y_i,z_i)$

For each point $i$, its theoretical radius $r_i=(R/H)\times (H-z_i)$, and its actual distance to $(X_c,Y_c)$ is given by $d_i=\sqrt{(x_i-X_c)^2+(y_i-Y_c)^2}$  
So we are just to find a $(X_c,Y_c)$ that makes the differences between $r_i$ and $d_i$ the minimum for all points:
$$Min\sum^{n}_{i=1} (d_i-r_i)^2$$

There is also an algorithim to be chosen to iterate the solution, here Levenberg-Marquardt method is used

- Benefit: Minimum geometrical error, more robust with noises
- Drawback: A little bit slower, need an initial guess to begin with

However, we can just use the result from approach 1 as the initial guess point, and here's a python implement
```python
import scipy.optimize as opt
# Non-Linear Approach
def residuals(center, points):
    [cx, cy] = center

    xi = points[:, 0]
    yi = points[:, 1]
    zi = points[:, 2]

    distances = np.sqrt((xi - cx)**2 + (yi - cy)**2)
    theo_r = K * (Height - zi)

    return distances - theo_r

# Just pass the result from linear appraoch to initial
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
```

The result now is much more accurate

## RANSAC - Not adopted
Usually when doing least squares method we also do a RANSAC (RANdom SAmple Consensus), which is to randomly choose some points from the sample, and test the approximation using the smaller sample  
It is helpful to eliminate samples with extreme errors

However, it is not working if the sample size is too small (3~5) so we just ignore this method.
