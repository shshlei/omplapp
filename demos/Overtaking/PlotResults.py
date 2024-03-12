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

from math import cos, sin, sqrt

# vehicle params
l = 2.8
l1 = 0.96
l2 = 0.929
W = 1.942
L = l + l1 + l2

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw Trajectory.')
    parser.add_argument('--cases', type=int, default = 0, \
        help='The parking cases')
    parser.add_argument('-t', '--time', default=None, \
        help='Filename of time trajectory')
    parser.add_argument('-s', '--states', default=None, \
        help='Filename of states trajectory')
    parser.add_argument('-c', '--controls', default=None, \
        help='Filename of controls trajectory')
    parser.add_argument('-p', '--points', default=None, \
        help='Filename of active points')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    colors = ['#1072b4', '#ef7f00', '#1f9c3a', '#d6191b', '#9763a6', '#965947', '#db79ae', '#c0c205', '#26b9ce', '#2a3377', '#7f3b71', '#9fa0a0']

    fig, ax = plt.subplots()#(figsize=(3.5, 3.5))

    time = []
    states = []
    points = []
    if args.time:
        line = open(args.time, 'r').readline()
        line = line.strip()
        time=[float(x) for x in line.split(' ')]
    if args.states:
        for line in open(args.states, 'r').readlines():
            line = line.strip()
            if not line:
                continue
            state = [float(x) for x in line.split(' ')]
            states.append(state)
        states = np.array(states).T
    if args.points:
        points = []
        for line in open(args.points, 'r').readlines():
            line = line.strip()
            if not line:
                continue
            point = [float(x) for x in line.split(' ')]
            points.append(point)
        points = np.array(points).T

    if args.time and args.states:
        # obstacles
        patches_rectangle = []
        if args.cases == 0:
            rect = Rectangle((10.0 - l2, 0.0 - 0.5 * W), L, W, angle=0.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
            patches_rectangle.append(rect)
        else:
            for t in time:
                rect = Rectangle((10.0 * t + 10.0 - l2, 0.0 - 0.5 * W), L, W, angle=0.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
                patches_rectangle.append(rect)
                if args.cases == 2:
                    rect = Rectangle((10.0 * t + 20.0 - l2, 0.0 - 0.5 * W), L, W, angle=0.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
                    patches_rectangle.append(rect)
        pcrect = PatchCollection(patches_rectangle, match_original=True)
        ax.add_collection(pcrect)
        # vehicle
        patches_rectangle = []
        patches_arrow = []
        for i, x, y, theta in zip(range(len(states[0])), states[0], states[1], states[2]):
            rect = Rectangle((x - l2, y - 0.5 * W), L, W, angle = 180.0 * theta / np.pi, rotation_point=(x, y), facecolor='w', edgecolor='k', lw=0.5, fill=False)
            patches_rectangle.append(rect)
            arrow = FancyArrow(x, y, cos(theta), sin(theta), width = 0.05, color = 'red')
            patches_arrow.append(arrow)
        pcrect = PatchCollection(patches_rectangle, match_original=True)
        ax.add_collection(pcrect)
        parrow = PatchCollection(patches_arrow, match_original=True)
        ax.add_collection(parrow)
    if args.points:
        ax.scatter(points[1], points[2], facecolor='r', edgecolor='k', s=30.0)

    ax.axhline(0.0, xmin=-10.0, xmax=100.0, ls='--', color='k', lw=1.0, alpha=0.2)
    ax.axhline(-5.25, xmin=-10.0, xmax=100.0, ls='--', color='#d6191b', lw=1.0)
    ax.axhline(-1.75, xmin=-10.0, xmax=100.0, ls='--', color='#d6191b', lw=1.0)
    ax.axhline(1.75, xmin=-10.0, xmax=100.0, ls='--', color='#d6191b', lw=1.0)
    ax.axhline(5.25, xmin=-10.0, xmax=100.0, ls='--', color='#d6191b', lw=1.0)

    ax.yaxis.set_major_locator(ticker.FixedLocator([-5.25, -1.75, 1.75, 5.25]))
    ax.yaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
    ax.xaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
    if args.cases == 0:
        ax.set_xlim(-1.0, 25.0)
    if args.cases == 1:
        ax.set_xlim(-1.0, 55.0)
    if args.cases == 2:
        ax.set_xlim(-1.0, 65.0)
    ax.set_ylim(-6.75, 6.75)

    ax.margins(0.05, 0.1)
    ax.set_axis_on()
    #ax.set_xticks([])
    #ax.set_yticks([])
    ax.set_aspect('equal')
    #setup(ax)
    plt.tight_layout()
    #plt.savefig('overtaking_i_0.svg')
    plt.show()
