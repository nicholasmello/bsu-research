import matplotlib.pyplot as plt
import numpy as np
from itertools import chain

# Fixing random state for reproducibility
np.random.seed(19680801)

def randrange(n, vmin, vmax):
    """
    Helper function to make an array of random numbers having shape (n, )
    with each number distributed Uniform(vmin, vmax).
    """
    return (vmax - vmin)*np.random.rand(n) + vmin

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

n = 100

# For each set of style and range settings, plot n random points in the box
# defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].
xs = randrange(n*n, n//2-2, n//2+2)
ys = []
zs = []
for i in range(0,n):
    ys = list(chain(ys, range(0, n)))
    zs = list(chain(zs, range(i, n)))
    zs = list(chain(zs, range(0, i)))

# ax.scatter(xs, ys, zs, marker='^')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

ax.set_xlim([0, n])
ax.set_ylim([0, n])
ax.set_zlim([0, n])

def surface_detection(xs, ys, zs):
    if max(xs)-min(xs) < max(ys)-min(ys):
        # do fit
        tmp_A = []
        tmp_b = []
        for i in range(len(zs)):
            tmp_A.append([zs[i], ys[i], 1])
            tmp_b.append(xs[i])
        b = np.matrix(tmp_b).T
        A = np.matrix(tmp_A)

        # Manual solution
        fit = (A.T * A).I * A.T * b
        errors = b - A * fit
        residual = np.linalg.norm(errors)

        print("solution: %f z + %f y + %f = x" % (fit[0], fit[1], fit[2]))

        # plot plane
        Z,Y = np.meshgrid(np.arange(min(zs), max(zs)),
                        np.arange(min(ys), max(ys)))
        X = np.zeros(Z.shape)
        for r in range(Z.shape[0]):
            for c in range(Z.shape[1]):
                X[r,c] = fit[0] * Z[r,c] + fit[1] * Y[r,c] + fit[2]
    else:
        # do fit
        tmp_A = []
        tmp_b = []
        for i in range(len(zs)):
            tmp_A.append([zs[i], xs[i], 1])
            tmp_b.append(ys[i])
        b = np.matrix(tmp_b).T
        A = np.matrix(tmp_A)

        # Manual solution
        fit = (A.T * A).I * A.T * b
        errors = b - A * fit
        residual = np.linalg.norm(errors)
        print(residual)

        print("solution: %f z + %f x + %f = y" % (fit[0], fit[1], fit[2]))

        # plot plane
        Z,X = np.meshgrid(np.arange(min(zs), max(zs)),
                        np.arange(min(xs), max(xs)))
        Y = np.zeros(Z.shape)
        for r in range(Z.shape[0]):
            for c in range(Z.shape[1]):
                Y[r,c] = fit[0] * Z[r,c] + fit[1] * X[r,c] + fit[2]
    return X, Y, Z

X, Y, Z = surface_detection(xs, ys, zs)
print(X)
print(Y)
print(Z)
ax.plot_wireframe(X,Y,Z, color='k')

plt.show()

# Max - Min to choose which one to do between x and y