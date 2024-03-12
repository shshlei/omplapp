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
l = 0.028
l1 = 0.0096
l2 = 0.00929
W = 0.01942
L = l + l1 + l2

class ActivePointsAnimator(object):
    def __init__(self, scenarios, states, points):
        self.fig, self.ax = plt.subplots()
        #self.fig.set_size_inches([4.8, 4.8])
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
        xy = (x - c * l2 + 0.5 * s * W, y - s * l2 - 0.5 * c * W)
        self.h_vehicle = Rectangle(xy, L, W, angle = 180.0 * t / np.pi, edgecolor='#ef7f00', facecolor='green', lw=1.5, fill=False)
        self.h_arrow = FancyArrow(x, y, 0.02 * c, 0.02 * s, width = 0.001, color = 'k')
        #self.p_vehicle = PatchCollection(self.h_vehicle, match_original=False)
        self.ax.add_patch(self.h_vehicle)
        self.ax.add_patch(self.h_arrow)
        if not self.points is None:
            self.h_points = self.ax.scatter([], [], facecolor='r', edgecolor='k', s=10.0)

        #self.ax.xaxis.set_major_locator(ticker.MultipleLocator(2.0))
        self.ax.yaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
        self.ax.xaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
        self.ax.set_xlim(0.0, 1.0)
        self.ax.set_ylim(0.0, 1.0)
        #self.ax.margins(0.1, 0.1)
        self.ax.set_axis_on()
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_aspect('equal')
        self.plot_artists = [self.h_vehicle, self.h_arrow]
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
        self.h_arrow.set_data(x=x, y=y, dx=0.02 * c, dy=0.02 * s)
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
    parser.add_argument('--scenario', default=None, \
        help='Filename of random scenario')
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

    pc = None
    if args.scenario:
        nobstacle = 0
        patches_circle = []
        patches_polygon = []
        patches_ellipse = []
        patches_capsule = []
        patches_rectangle = []
        for line in open(args.scenario, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            if l[0] == '#':
                continue
            if l == 'shapes':
                continue
            if nobstacle == 0:
                temp = l.split(' ')
                if temp[0] == 'numbers':
                    nobstacle = int(temp[1])
                    continue
            params = [float(x) for x in l.split(' ')]
            params[0] = int(params[0])
            if params[0] == 0: # circle
                x = params[1]
                y = params[2]
                r = params[3]
                circle = Circle((x, y), r, color = 'gray')
                patches_circle.append(circle)
            elif params[0] == 1: # polygon
                params[1] = int(params[1])
                count = params[1]
                vecs = params[2:]
                x = vecs[::2]
                y = np.array(vecs[1::2])
                polygon = Polygon(np.array([x,y]).transpose(), color = 'gray')
                patches_polygon.append(polygon)
            elif params[0] == 2: # ellipse
                x = params[1]
                y = params[2]
                yaw = params[3]
                a = params[4]
                b = params[5]
                ellipse = Ellipse((x, y), 2.0 * a, 2.0 * b, angle = 180.0 * yaw / np.pi, color = 'gray')
                patches_ellipse.append(ellipse)
            elif params[0] == 3: # capsule
                x = params[1]
                y = params[2]
                yaw = params[3]
                r = params[4]
                h = params[5]
                s = sin(yaw)
                c = cos(yaw)
                p1 = (x - s * h, y + c * h)
                p2 = (x + s * h, y - c * h)
                capsule = SLineString([p1, p2]).buffer(r)
                patches_capsule.append(SPolygonPatch(capsule, fc='gray', ec='gray'))
            elif params[0] == 4: # rectangle
                x = params[1]
                y = params[2]
                yaw = params[3]
                a = params[4]
                b = params[5]
                s = sin(yaw)
                c = cos(yaw)
                R = np.array(((c, -s), (s, c)))
                xy = np.array(((-a, a, a, -a),(-b, -b, b, b)))
                xy = R.dot(xy)
                x1, y1 = xy
                x1 = x1 + x
                y1 = y1 + y
                rect = Polygon(np.array([x1,y1]).transpose(), color = 'gray')
                patches_rectangle.append(rect)
        pc = PatchCollection(patches_circle + patches_polygon + patches_ellipse + patches_capsule + patches_rectangle, match_original=False)
        pc.set_color('gray')

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
        animation_length = 50.0
        anim1 = ActivePointsAnimator(pc, states, points)
        delta_t = (animation_length * 1000.0 / anim1.max_frames)
        animation1 = animation.FuncAnimation(anim1.fig, anim1.animate, init_func=anim1.init_fig, frames=anim1.max_frames, interval=delta_t, blit=True, repeat_delay=1000.0)
        plt.tight_layout()
        plt.show()
        #animation1.save('puresampling.gif', writer='ffmpeg', fps=int(1000.0/delta_t), dpi=300)
