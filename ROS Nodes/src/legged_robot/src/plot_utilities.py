#!/usr/bin/env python3
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt  

def plotLine(ax : plt.Axes, v1: np.ndarray, color):
    ax.plot3D(v1[:,0], v1[:,1], v1[:,2], c=color)
    
def plotLine2(ax: plt.Axes, v1: np.ndarray, v2: np.ndarray, color):
    ax.plot3D([v1[0], v2[0]], [v1[1], v2[1]], [v1[2], v2[2]], c=color)

def plotPoint(ax: plt.Axes, p: np.ndarray, color):
    ax.scatter(p[0], p[1], p[2], c=color, marker='.')