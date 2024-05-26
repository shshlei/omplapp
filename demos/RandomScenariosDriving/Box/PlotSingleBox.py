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

import os
import argparse

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.patches import Circle, Polygon, Rectangle, Ellipse, FancyArrow
from matplotlib.collections import PatchCollection

from shapely.geometry import LineString as SLineString
from descartes import PolygonPatch as SPolygonPatch

from math import cos, sin, sqrt

# vehicle params
l = 0.028
l1 = 0.0096
l2 = 0.00929
W = 0.01942
L = l + l1 + l2
crx = 0.25 * L - l2
cfx = 0.75 * L - l2
cradius = sqrt(0.25*L*0.25*L + 0.5*W*0.5*W)

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw Trajectory.')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    colors = ['#1072b4', '#ef7f00', '#1f9c3a', '#d6191b', '#9763a6', '#965947', '#db79ae', '#c0c205', '#26b9ce', '#2a3377', '#7f3b71', '#9fa0a0']

    fig, ax = plt.subplots()

    # capsule scenario
    x = 0.540926
    y = 0.1448617
    yaw = 3.00245
    r = 0.0505465
    h = 0.0591358
    c = cos(yaw)
    s = sin(yaw)
    p1 = (x - s * h, y + c * h)
    p2 = (x + s * h, y - c * h)
    capsule = SLineString([p1, p2]).buffer(r)
    ax.add_patch(SPolygonPatch(capsule, fc='gray', ec='gray'))

    # rect vehicle
    x = 0.470161
    y = 0.249875
    theta = 0.710477
    cx = x + crx * cos(theta)
    cy = y + crx * sin(theta)
    rect = Rectangle((x - l2, y - 0.5 * W), L, W, angle = 180.0 * theta / np.pi, rotation_point=(x, y), facecolor='w', edgecolor='#1f9c3a', lw=1.0, zorder=10, fill=False)
    ax.add_patch(rect)
    ax.scatter(x, y, color='#B30059', s = 20)

    # rear circle
    circle = Circle((cx, cy), cradius, color='#1f9c3a', lw=1.0, fill=False)
    ax.add_patch(circle)
    box = Rectangle((cx - cradius - 0.0001, cy - cradius - 0.0001), 2 * cradius + 0.0002, 2 * cradius + 0.0002, angle=0.0, facecolor='w', edgecolor='#ef7f00', lw=1.0, zorder=10, fill=False)
    ax.add_patch(box)

    # rear box
    point = [0.423295, 0.234718, 0.0624091, 0.0654534]
    box = Rectangle((point[0], point[1]), point[2], point[3], angle=0.0, facecolor='w', edgecolor='#1072b4', lw=1.0, zorder=10, fill=False)
    ax.add_patch(box)

    box = Rectangle((point[0] - cradius, point[1] - cradius), point[2] + 2 * cradius, point[3] + 2 * cradius, angle=0.0, facecolor='w', edgecolor='r', lw=1.0, zorder=10, fill=False)
    ax.add_patch(box)

    ax.yaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
    ax.xaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)

    ax.set_xlim(0.40, 0.61)
    ax.set_ylim(0.18, 0.32)
    #ax.autoscale(enable=None, axis="both", tight=True)
    ax.margins(0.05, 0.1)
    ax.set_axis_on()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.savefig('parking.svg')
    plt.show()
