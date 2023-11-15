#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Shi Shenglei

import argparse
import os

import numpy as np
import matplotlib.pyplot as plt
#plt.switch_backend("cairo")
from matplotlib.patches import Circle, Polygon
from matplotlib.collections import PatchCollection

import cmasher as cmr
from math import sqrt

def func(x, y, r, delta):
    xmin, xmax = max(0.0, x - r), min(1.0, x + r)
    ymin, ymax = max(0.0, y - r), min(1.0, y + r)
    xs, xe = int((xmin - x)/delta), int((xmax - x)/delta)
    ys, ye = int((ymin - y)/delta), int((ymax - y)/delta)
    X = np.linspace(xmin, xmax, xe - xs + 1)
    Y = np.linspace(ymin, ymax, ye - ys + 1)
    Z = np.zeros([ye - ys + 1, xe - xs + 1])
    sigma = r/3.0
    wh2 = 2.0 * r * r / (delta * delta)
    for i in range(xs, xe):
        for j in range(ys, ye):
            ij2 = i*i + j*j
            if ij2 > wh2:
                continue
            ijr2 = ij2 * delta * delta
            Z[j - ys, i - xs] = np.exp(-ijr2/(2.0*sigma*sigma))
    return X, Y, Z

def func_inverty(x, y, r, delta):
    xmin, xmax = max(0.0, x - r), min(1.0, x + r)
    ymin, ymax = max(0.0, y - r), min(1.0, y + r)
    xs, xe = int((xmin - x)/delta), int((xmax - x)/delta)
    ys, ye = int((ymin - y)/delta), int((ymax - y)/delta)
    X = np.linspace(xmin, xmax, xe - xs + 1)
    Y = -np.linspace(ymin, ymax, ye - ys + 1)
    Z = np.zeros([ye - ys + 1, xe - xs + 1])
    sigma = r/3.0
    wh2 = 2.0 * r * r / (delta * delta)
    for i in range(xs, xe):
        for j in range(ys, ye):
            ij2 = i*i + j*j
            if ij2 > wh2:
                continue
            ijr2 = ij2 * delta * delta
            Z[ye -ys - (j - ys), i - xs] = np.exp(-ijr2/(2.0*sigma*sigma))
    return X, Y, Z

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Show random scenario motion planning result.')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})

    invert_yaxis = False

    fig, ax = plt.subplots(figsize=(1.8, 1.8))

    if False:
        """heat map"""
        delta = 0.0005
        if invert_yaxis:
            x, y, z = func_inverty(0.59, 0.45, 0.03, delta)
        else:
            x, y, z = func(0.59, 0.45, 0.03, delta)
        x, y = np.meshgrid(x, y)
        cmap = 'RdBu'
        pcm = plt.pcolormesh(x, y, z, cmap=cmap, vmin=0.0, vmax=2.0)
        if invert_yaxis:
            pc = Circle((0.59, -0.45), 0.03, transform=ax.transData)
        else:
            pc = Circle((0.59, 0.45), 0.03, transform=ax.transData)
        pcm.set_clip_path(pc)
        ax.set_title(cmap, fontsize=14)
        plt.colorbar()

