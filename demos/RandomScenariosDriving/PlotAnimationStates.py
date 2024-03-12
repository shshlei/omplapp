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

from shapely.geometry import LineString as SLineString
from descartes import PolygonPatch as SPolygonPatch

from math import cos, sin, sqrt

# vehicle params
l = 0.028
l1 = 0.0096
l2 = 0.00929
W = 0.01942
L = l + l1 + l2

class StatesAnimator(object):
    def __init__(self, scenarios, num):
        self.fig, self.ax = plt.subplots()
        #self.fig.set_size_inches([4.8, 4.8])
        self.scenarios = scenarios
        self.max_frames = num

    def init_fig(self):
        # scenarios
        self.ax.cla()
        if not self.scenarios is None:
            self.ax.add_collection(self.scenarios)
        #rect = Rectangle((0, 0), 1, 1, angle=0.0, facecolor='w', edgecolor='gray', lw=0.5, zorder=10, fill=False)
        #self.ax.add_patch(rect)

        #self.title=self.ax.text(0.5, 1.05, 'Iteration', transform=self.ax.transData, ha="center", va="center", fontsize=10.5, fontfamily='SimSun', math_fontfamily='stix', fontweight=200)
        self.title=self.ax.text(0.5, 0.95, '', transform=self.ax.transData, ha='center', va='center', fontfamily='serif', fontsize=10.5, color='r')

        #self.ax.xaxis.set_major_locator(ticker.MultipleLocator(2.0))
        self.ax.yaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
        self.ax.xaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=10.5)
        self.ax.set_xlim(0.0, 1.0)
        self.ax.set_ylim(0.0, 1.0)
        #self.ax.margins(0.1, 0.1)
        self.ax.set_axis_on()
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_axis_off()
        self.ax.set_aspect('equal')
        self.plot_artists = [self.title]
        return self.plot_artists

    def animate(self, i):
        states_file = str('../../build/bin/states_'+str(i)+'.txt')
        states = []
        for line in open(states_file, 'r').readlines():
            line = line.strip()
            if not line:
                continue
            state = [float(x) for x in line.split(' ')]
            states.append(state)
        states = np.array(states).T

        title = str('Iteration '+str(i))
        if i==0:
            title='概率采样'
        elif i==self.max_frames-1:
            title='最优轨迹'
        self.title.set_text(title)

        patches_rectangle = []
        patches_arrow = []
        for i, x, y, theta in zip(range(len(states[0])), states[0], states[1], states[2]):
            rect = Rectangle((x - l2, y - 0.5 * W), L, W, angle = 180.0 * theta / np.pi, rotation_point=(x, y), facecolor='w', edgecolor='g', lw=0.5, zorder=10, fill=False)
            patches_rectangle.append(rect)
            arrow = FancyArrow(x, y, 0.01* cos(theta), 0.01* sin(theta), width = 0.0005, color = 'r')
            patches_arrow.append(arrow)
        self.pcrect = PatchCollection(patches_rectangle, match_original=True)
        self.parrow = PatchCollection(patches_arrow, match_original=True)
        self.ax.add_collection(self.pcrect)
        self.ax.add_collection(self.parrow)
        self.plot_artists = [self.pcrect, self.parrow, self.title]
        return self.plot_artists

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw Trajectory.')
    parser.add_argument('--scenario', default=None, \
        help='Filename of random scenario')
    parser.add_argument('-n', '--num', default=None, \
        help='States Filename number.')
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

    if args.num:
        animation_length = 10.0
        anim1 = StatesAnimator(pc, int(args.num))
        delta_t = (animation_length * 1000.0 / anim1.max_frames)
        animation1 = animation.FuncAnimation(anim1.fig, anim1.animate, init_func=anim1.init_fig, frames=anim1.max_frames, interval=delta_t, blit=True, repeat_delay=1000.0)
        plt.show()
        #animation1.save('puresampling.gif', writer='ffmpeg', fps=int(1000.0/delta_t), dpi=300)
