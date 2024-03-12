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
    parser.add_argument('--forward', action="store_true", help='If forward parking')
    parser.add_argument('--scenarios', type=int, default = 0, \
        help='The parking scenarios')
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

    patches_rectangle = [] # scenario
    if args.scenarios ==  0:
        rect = Rectangle((0.0, 0.0), 6.0, 2.5, facecolor='w', edgecolor='b', lw=3.0) # parking lot
        #print(rect.get_corners())
        patches_rectangle.append(rect)
        rect = Rectangle((-5.6, 0.6), L, W, angle=-10.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
        #print(rect.get_corners())
        patches_rectangle.append(rect)
        rect = Rectangle((5.8, -0.5), L, W, angle=13.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
        #print(rect.get_corners())
        patches_rectangle.append(rect)
    elif args.scenarios == 1:
        rect = Rectangle((0.0, 0.0), 6.0, 2.5, facecolor='w', edgecolor='b', lw=3.0) # parking lot
        patches_rectangle.append(rect)
        rect = Rectangle((-7.0, 2.8), L, W, angle=-55.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
        patches_rectangle.append(rect)
        rect = Rectangle((5.8, -0.5), L, W, angle=13.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
        patches_rectangle.append(rect)
    elif args.scenarios == 2: # vertical
        rect = Rectangle((0.0, 0.0), 2.5, 6.0, facecolor='w', edgecolor='b', lw=3.0) # parking lot
        patches_rectangle.append(rect)
        rect = Rectangle((-2.2, 0.4), W, L, facecolor='r', edgecolor='r', lw=2.0) # obstacle one
        patches_rectangle.append(rect)
        rect = Rectangle((2.3, 0.3), W, L, angle=-5.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
        patches_rectangle.append(rect)
    elif args.scenarios == 3:
        rect = Rectangle((0.0, 0.0), 6.0, 2.5, facecolor='w', edgecolor='b', lw=3.0) # parking lot
        patches_rectangle.append(rect)
        rect = Rectangle((-6.3, 0.3), L, W, angle=20.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle one
        patches_rectangle.append(rect)
        rect = Rectangle((6.8, -1.0), L, W, angle=40.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
        patches_rectangle.append(rect)
        rect = Rectangle((2.0, 3.6), L, W, angle=60.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle three
        patches_rectangle.append(rect)
        rect = Rectangle((-1.5, -2.2), L, W, angle=-3.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle four
        patches_rectangle.append(rect)
    pcrect = PatchCollection(patches_rectangle, match_original=True)
    ax.add_collection(pcrect)

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

    if args.states:
        patches_rectangle = []
        patches_arrow = []
        for i, x, y, theta in zip(range(len(states[0])), states[0], states[1], states[2]):
            if args.forward:
                rect = Rectangle((x - l2, y - 0.5 * W), L, W, angle = 180.0 * theta / np.pi, rotation_point=(x, y), facecolor='w', edgecolor='k', lw=0.5, fill=False)
            else:
                rect = Rectangle((x - l - l1, y - 0.5 * W), L, W, angle = 180.0 * theta / np.pi, rotation_point=(x, y), facecolor='w', edgecolor='k', lw=0.5, fill=False)
            patches_rectangle.append(rect)
            arrow = FancyArrow(x, y, cos(theta), sin(theta), width = 0.03, color = 'magenta')
            patches_arrow.append(arrow)
        pcrect = PatchCollection(patches_rectangle, match_original=True)
        ax.add_collection(pcrect)
        parrow = PatchCollection(patches_arrow, match_original=True)
        ax.add_collection(parrow)
    if args.points:
        ax.scatter(points[1], points[2], facecolor='r', edgecolor='k', s=30.0)

    ax.xaxis.set_major_locator(ticker.MultipleLocator(2.0))
    ax.yaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
    ax.xaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)

    #ax.set_xlim(-10.0, 12.0)
    #ax.set_ylim(-5.0, 10.0)
    #ax.autoscale(enable=None, axis="both", tight=True)
    ax.margins(0.05, 0.1)
    ax.set_axis_on()
    #ax.set_xticks([])
    #ax.set_yticks([])
    ax.set_aspect('equal')
    #setup(ax)
    plt.tight_layout()
    #plt.savefig('parking.svg')
    plt.show()
