import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
#import matplotlib.colors as colors
from matplotlib.colors import Normalize

def normal_pdf(x, mean, var):
    return np.exp(-(x - mean)**2 / (2*var))

def openPath(fname):
    path = []
    if fname:
        for line in open(fname, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            path.append([float(x) for x in l.split(' ')])
    return path

path = openPath('/home/shshlei/mine/motion/omplapp/release/bin/path.txt')
path  = np.array(path).T

# compute some interesting data
x0, x1 = 0, 1
y0, y1 = 0, 1
x = np.linspace(x0, x1, 500)
y = np.linspace(y0, y1, 500)
X1 = normal_pdf(x, 0.2, 0.0003)
Y1 = normal_pdf(x, 0.2, 0.0003)
X2 = normal_pdf(x, 0.7, 0.0003)
Y2 = normal_pdf(x, 0.4, 0.0003)
Z = (np.outer(Y1, X1) + np.outer(Y2, X2)) * 2

# First we'll plot these blobs using ``imshow`` without transparency.
vmax = np.abs(Z).max()
imshow_kwargs = {
    'vmax': vmax,
    'vmin': -vmax,
    'cmap': 'RdYlBu',
    'origin':'lower',
    'extent': (x0, x1, y0, y1),
}

# Set up a colormap:
palette = plt.cm.gray#.with_extremes(over='r', under='g', bad='b')
# Alternatively, we could use
# palette.set_bad(alpha = 0.0)

# set up the Axes objects
fig, ax = plt.subplots(figsize=(6, 5.4))

# plot using 'continuous' colormap
#im = ax.imshow(Z, interpolation='bilinear',
#                cmap=palette,
#                norm=colors.Normalize(vmin=-0.2, vmax=0.5),
#                aspect='auto',
#                origin='lower',
#                extent=[x0, x1, y0, y1])

# greys = np.full((*Z.shape, 3), 200.0, dtype=np.uint8)
# ax.imshow(greys)
# ax.imshow(Z, interpolation='bilinear',  cmap='gray', origin='lower', extent=[x0, x1, y0, y1])
#ax.set_axis_off()

circ = patches.Circle((0.5, 0.5), 0.2, color = 'gray')
ell1 = patches.Ellipse((0.3, 0.85), 0.4, 0.2, color = 'gray')
ell2 = patches.Ellipse((0.75, 0.25), 0.4, 0.2, 45, color = 'gray')
rec1 = patches.Rectangle((0.1, 0.1), 0.2, 0.2, color = 'gray')
rec2 = patches.Rectangle((0.1, 0.5), 0.2, 0.2, color = 'gray')
rec3 = patches.Rectangle((0.65, 0.55), 0.3, 0.4, color = 'gray')

ax.add_patch(circ)
ax.add_patch(ell1)
ax.add_patch(ell2)
ax.add_patch(rec1)
ax.add_patch(rec2)
ax.add_patch(rec3)

plt.plot(path[0], path[1])

ax.set_aspect('equal')

plt.show()
