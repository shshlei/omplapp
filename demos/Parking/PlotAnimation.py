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
import matplotlib.animation as animation

from math import cos, sin, sqrt

# vehicle params
l = 2.8
l1 = 0.96
l2 = 0.929
W = 1.942
L = l + l1 + l2

class ActivePointsAnimator(object):
    def __init__(self, forward, scenarios, states, points):
        self.fig, self.ax = plt.subplots()
        #self.fig.set_size_inches([4.8, 4.8])
        self.forward = forward
        self.scenarios = scenarios
        self.states = states
        self.points = points
        self.max_frames = len(self.states[0])
        if not self.points is None:
            self.point_index = int(self.points[0][0])
            self.end_index = 0

    def init_fig(self):
        # scenarios
        self.ax.cla()
        if not self.scenarios is None:
            self.ax.add_collection(self.scenarios)
        # initial vehicle
        x = self.states[0][0]
        y = self.states[1][0]
        t = self.states[2][0]
        c = cos(t)
        s = sin(t)
        if self.forward:
            xy = (x - c * l2 + 0.5 * s * W, y - s * l2 - 0.5 * c * W)
        else:
            xy = (x - c * (l +l1) + 0.5 * s * W, y - s * (l +l1) - 0.5 * c * W)
        self.h_vehicle = Rectangle(xy, L, W, angle = 180.0 * t / np.pi, edgecolor='#ef7f00', facecolor='green', lw=1.5)
        self.h_arrow = FancyArrow(x, y, c, s, width = 0.1, color = 'k')
        #self.p_vehicle = PatchCollection(self.h_vehicle, match_original=False)
        self.ax.add_patch(self.h_vehicle)
        self.ax.add_patch(self.h_arrow)
        if not self.points is None:
            self.h_points = self.ax.scatter([], [], facecolor='r', edgecolor='k', s=30.0)

        self.ax.xaxis.set_major_locator(ticker.MultipleLocator(2.0))
        self.ax.yaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
        self.ax.xaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
        self.ax.set_xlim(-8.0, 11.0)
        self.ax.set_ylim(-4.0, 9.0)
        self.ax.margins(0.05, 0.1)
        self.ax.set_axis_on()
        #self.ax.set_xticks([])
        #self.ax.set_yticks([])
        self.ax.set_aspect('equal')
        if not self.points is None:
            self.plot_artists = [self.h_vehicle, self.h_arrow, self.h_points]
        else:
            self.plot_artists = [self.h_vehicle, self.h_arrow]
        return self.plot_artists

    def animate(self, i):
        x = self.states[0][i]
        y = self.states[1][i]
        t = self.states[2][i]
        c = cos(t)
        s = sin(t)
        if self.forward:
            self.h_vehicle.set_x(x - c * l2 + 0.5 * s * W)
            self.h_vehicle.set_y(y - s * l2 - 0.5 * c * W)
        else:
            self.h_vehicle.set_x(x - c * (l +l1) + 0.5 * s * W)
            self.h_vehicle.set_y(y - s * (l +l1) - 0.5 * c * W)
        self.h_vehicle.set_angle(180.0 * t / np.pi)
        self.h_arrow.set_data(x=x, y=y, dx=c, dy=s)
        if not self.points is None:
            if i == 0:
                self.point_index = int(self.points[0][0])
                self.end_index = 0
            if self.point_index == i:
                found = False
                start = self.end_index
                for j in range(start, len(self.points[0])):
                    if int(self.points[0][j]) > self.point_index:
                        found = True
                        self.end_index = j
                        self.point_index = self.points[0][self.end_index]
                        break
                if not found:
                    self.end_index = len(self.points[0])
                self.h_points.set_offsets(np.stack((self.points[1][range(start, self.end_index)], self.points[2][range(start, self.end_index)])).T)
                self.plot_artists = [self.h_vehicle, self.h_arrow, self.h_points]
            else:
                self.plot_artists = [self.h_vehicle, self.h_arrow]

        return self.plot_artists

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw Trajectory.')
    parser.add_argument('--forward', action="store_true", help='If forward parking')
    parser.add_argument('--scenarios', type=int, default = 0, \
        help='The parking scenarios')
    parser.add_argument('-s', '--states', default=None, \
        help='Filename of states trajectory')
    parser.add_argument('-p', '--points', default=None, \
        help='Filename of active points')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    colors = ['#1072b4', '#ef7f00', '#1f9c3a', '#d6191b', '#9763a6', '#965947', '#db79ae', '#c0c205', '#26b9ce', '#2a3377', '#7f3b71', '#9fa0a0']

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

    states = []
    points = None
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
        animation_length = 20.0
        anim1 = ActivePointsAnimator(args.forward, pcrect, states, points)
        delta_t = (animation_length * 1000.0 / anim1.max_frames)
        animation1 = animation.FuncAnimation(anim1.fig, anim1.animate, init_func=anim1.init_fig, frames=anim1.max_frames, interval=delta_t, blit=True, repeat_delay=1000.0)
        plt.tight_layout()
        plt.show()
        #animation1.save('S4C3_ap_biase_100.gif', writer='ffmpeg', fps=int(1000.0/delta_t), dpi=300)
