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

import time as stime

from math import cos, sin, sqrt

# vehicle params
l = 2.8
l1 = 0.96
l2 = 0.929
W = 1.942
L = l + l1 + l2

class Animator(object):
    def __init__(self, cases, time, states, points):
        self.fig, self.ax = plt.subplots()
        #self.fig.set_size_inches([4.8, 2.4])
        self.cases = cases
        self.time = time
        self.states = states
        self.max_frames = len(self.time)
        self.points = points
        if not self.points is None:
            self.point_index = int(self.points[0][0])
            self.end_index = 0

    def init_fig(self):
        self.ax.cla()
        # obstacles
        self.h_obstacle1 = Rectangle((10.0 - l2, 0.0 - 0.5 * W), L, W, angle=0.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
        self.ax.add_patch(self.h_obstacle1)
        if args.cases != 0:
            self.h_arrow1 = FancyArrow(10.0, 0.0 , 1.0, 0.0, width = 0.15, color = 'k')
            self.ax.add_patch(self.h_arrow1)
        if args.cases == 2:
            self.h_obstacle2 = Rectangle((20.0 - l2, 0.0 - 0.5 * W), L, W, angle=0.0, facecolor='r', edgecolor='r', lw=2.0) # obstacle two
            self.ax.add_patch(self.h_obstacle2)
            self.h_arrow2 = FancyArrow(20.0, 0.0, 1.0, 0.0, width = 0.15, color = 'k')
            self.ax.add_patch(self.h_arrow2)
        # initial vehicle
        x = self.states[0][0]
        y = self.states[1][0]
        t = self.states[2][0]
        c = cos(t)
        s = sin(t)
        xy = (x - c * l2 + 0.5 * s * W, y - s * l2 - 0.5 * c * W)
        self.h_vehicle = Rectangle(xy, L, W, angle = 180.0 * t / np.pi, edgecolor='#ef7f00', facecolor='green', lw=2.5)
        self.h_arrow = FancyArrow(x, y, cos(t), sin(t), width = 0.12, color = 'k')
        self.ax.add_patch(self.h_vehicle)
        self.ax.add_patch(self.h_arrow)
        if not self.points is None:
            self.h_points = self.ax.scatter([], [], facecolor='r', edgecolor='k', s=30.0)

        self.ax.axhline(0.0, xmin=-10.0, xmax=100.0, ls='--', color='k', lw=1.0, alpha=0.2)
        self.ax.axhline(-5.25, xmin=-10.0, xmax=100.0, ls='--', color='#d6191b', lw=1.0)
        self.ax.axhline(-1.75, xmin=-10.0, xmax=100.0, ls='--', color='#d6191b', lw=1.0)
        self.ax.axhline(1.75, xmin=-10.0, xmax=100.0, ls='--', color='#d6191b', lw=1.0)
        self.ax.axhline(5.25, xmin=-10.0, xmax=100.0, ls='--', color='#d6191b', lw=1.0)

        #self.ax.xaxis.set_major_locator(ticker.MultipleLocator(2.0))
        #self.ax.yaxis.set_major_locator(ticker.MultipleLocator(3.5))
        self.ax.yaxis.set_major_locator(ticker.FixedLocator([-5.25, -1.75, 1.75, 5.25]))
        self.ax.yaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
        self.ax.xaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
        if args.cases == 0:
            self.ax.set_xlim(-1.0, 25.0)
        if args.cases == 1:
            self.ax.set_xlim(-1.0, 55.0)
        if args.cases == 2:
            self.ax.set_xlim(-1.0, 75.0)
        self.ax.set_ylim(-6.75, 6.75)
        #self.ax.margins(0.05, 0.1)
        self.ax.set_axis_on()
        #ax.set_xticks([])
        #ax.set_yticks([])
        self.ax.set_aspect('equal')
        if args.cases == 0:
            self.plot_artists = [self.h_vehicle, self.h_arrow]
        elif args.cases == 1:
            self.plot_artists = [self.h_vehicle, self.h_arrow, self.h_obstacle1, self.h_arrow1]
        elif args.cases == 2:
            self.plot_artists = [self.h_vehicle, self.h_arrow, self.h_obstacle1, self.h_arrow1, self.h_obstacle2, self.h_arrow2]
        if not self.points is None:
            self.plot_artists.append(self.h_points)
        return self.plot_artists

    def animate(self, i):
        x = self.states[0][i]
        y = self.states[1][i]
        t = self.states[2][i]
        c = cos(t)
        s = sin(t)
        self.h_vehicle.set_x(x - c * l2 + 0.5 * s * W)
        self.h_vehicle.set_y(y - s * l2 - 0.5 * c * W)
        self.h_vehicle.set_angle(180.0 * t / np.pi)
        self.h_arrow.set_data(x=x, y=y, dx=c, dy=s)
        if self.cases != 0:
            t = self.time[i]
            self.h_obstacle1.set_x(10.0 * t + 10.0 - l2)
            self.h_arrow1.set_data(x=10.0 * t + 10.0, y=0.0, dx=1.0, dy=0.0)
            if self.cases == 2:
                self.h_obstacle2.set_x(10.0 * t + 20.0 - l2)
                self.h_arrow2.set_data(x=10.0 * t + 20.0, y=0.0, dx=1.0, dy=0.0)
        if args.cases == 0:
            self.plot_artists = [self.h_vehicle, self.h_arrow]
        elif args.cases == 1:
            self.plot_artists = [self.h_vehicle, self.h_arrow, self.h_obstacle1, self.h_arrow1]
        elif args.cases == 2:
            self.plot_artists = [self.h_vehicle, self.h_arrow, self.h_obstacle1, self.h_arrow1, self.h_obstacle2, self.h_arrow2]
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
                self.plot_artists.append(self.h_points)
        if i < self.max_frames - 1:
            stime.sleep(self.time[i+1] - self.time[i])
        return self.plot_artists

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw Trajectory.')
    parser.add_argument('--cases', type=int, default = 0, \
        help='The parking cases')
    parser.add_argument('-t', '--time', default=None, \
        help='Filename of time trajectory')
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

    time = []
    states = []
    points = None
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
        animation_length = 15.0
        anim1 = Animator(args.cases, time, states, points)
        delta_t = (animation_length * 1000.0 / anim1.max_frames)
        animation1 = animation.FuncAnimation(anim1.fig, anim1.animate, init_func=anim1.init_fig, frames=anim1.max_frames, interval=delta_t, blit=True)
        #animation1 = animation.FuncAnimation(anim1.fig, anim1.animate, init_func=anim1.init_fig, frames=anim1.max_frames, interval=0.0, blit=True, repeat_delay=1000.0)
        plt.tight_layout()
        plt.show()
        #animation1.save('overtaking_i_0.gif', writer='ffmpeg', fps=int(1000.0/delta_t), dpi=300)
