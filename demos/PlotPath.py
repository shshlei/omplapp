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
from matplotlib.patches import Circle, Polygon
from matplotlib.collections import PatchCollection

from math import cos, sin, sqrt

def setup(axs):
    xlim = axs.get_xlim()
    ylim = axs.get_ylim()

    #axs.spines["top"].set_visible(False)
    #axs.spines["right"].set_visible(False)

    lw = 0.5
    axs.spines.left.set_linewidth(lw)
    axs.spines.right.set_linewidth(lw)
    axs.spines.top.set_linewidth(lw)
    axs.spines.bottom.set_linewidth(lw)

    axs.spines["left"].set_position(("data", xlim[0]))
    axs.spines['left'].set_bounds(ylim[0], ylim[1])
    axs.spines['right'].set_bounds(ylim[0], ylim[1])

    axs.spines["bottom"].set_position(("data", ylim[0]))
    axs.spines['bottom'].set_bounds(xlim[0], xlim[1])
    axs.spines['top'].set_bounds(xlim[0], xlim[1])

    axs.xaxis.set_ticks_position('bottom')
    axs.xaxis.set_tick_params(which='major', direction = 'in', width=1.00, length=3)
    #axs.xaxis.set_major_formatter(ticker.StrMethodFormatter("{x} s"))

    axs.yaxis.set_ticks_position('left')
    axs.yaxis.set_tick_params(which='major', direction = 'in', width=1.00, length=3)

    axs.yaxis.grid(True, linestyle=(0, (5, 5)), lw = 1, which='major', color='darkgrey', alpha=1.0)

    axs.set_xlim(xlim)
    axs.set_ylim(ylim)

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw planned path.')
    parser.add_argument('-s', '--scenario', default=None, \
        help='Filename of random scenario')
    parser.add_argument('-path', '--plannerpath', default=None, \
        help='(Optional) Filename of the planner path.')
    parser.add_argument('-paths', nargs='*')
    args = parser.parse_args()

    plt.style.use(['seaborn-deep', 'seaborn-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']

    tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
    ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];

    fig, ax = plt.subplots(figsize=(1.8, 1.8))
    if args.scenario:
        nobstacle = 0
        patches_circle = []
        patches_polygon = []
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
                polygon = Polygon(np.array([x,y]).transpose(), True, color = 'gray')
                patches_polygon.append(polygon)
        pcc = PatchCollection(patches_circle, match_original=False)
        pcp = PatchCollection(patches_polygon, match_original=False)
        pcc.set_color('gray')
        pcp.set_color('gray')
        ax.add_collection(pcc)
        ax.add_collection(pcp)
    if args.plannerpath:
        xy = []
        for line in open(args.plannerpath, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy.append([float(x) for x in l.split(' ')])
        dxy = np.array(xy).transpose()
        x = dxy[0]
        y = dxy[1]
        if len(dxy) == 3:
            theta = dxy[2]
        if False:
            xx = list(x)
            yy = list(y)
            tt = list(theta)
            xxx = xx[1:-1]
            yyy = yy[1:-1]
            ttt = tt[1:-1]
            x = xx[0:1] + xxx[1:-1:15] + xx[-1:]
            y = yy[0:1] + yyy[1:-1:15] + yy[-1:]
            theta = tt[0:1] + ttt[1:-1:15] + tt[-1:]
        ax.plot(x, y, color='#B30059', linewidth=1.0)
        if True:
            patches_polygon = []
            if len(dxy) == 3:
                for xc, yc, t in zip(x, y, theta):
                    c  = cos(t)
                    s  = sin(t)
                    ttx = [c * x - s * y + xc for x, y in zip(tx, ty)]
                    tty = [s * x + c * y + yc for x, y in zip(tx, ty)]
                    polygon = Polygon(np.array([ttx, tty]).transpose(), True, color = 'gray')
                    patches_polygon.append(polygon)
            else:
                for xc, yc in zip(x, y):
                    ttx = [x + xc for x in tx]
                    tty = [y + yc for y in ty]
                    polygon = Polygon(np.array([ttx, tty]).transpose(), True, color = 'gray')
                    patches_polygon.append(polygon)
            pcp = PatchCollection(patches_polygon, match_original=False)
            pcp.set_color('green')
            ax.add_collection(pcp)
        ax.scatter(x, y, color='#B30059', s = 2.0)
    if args.paths:
        for path in args.paths:
            print(path)
            xy = []
            for line in open(path, 'r').readlines():
                l = line.strip()
                if not l:
                    continue
                xy.append([float(x) for x in l.split(' ')])
            dxy = np.array(xy).transpose()
            x = dxy[0]
            y = dxy[1]
            xx = list(x)
            yy = list(y)
            xxx = xx[1:-1]
            yyy = yy[1:-1]
            x = xx[0:1] + xxx[1:-1:25] + xx[-1:]
            y = yy[0:1] + yyy[1:-1:25] + yy[-1:]
            ax.scatter(x, y, s = 8)
            ax.plot(x, y, linewidth=2.0)

    if False:
        """start and goal polygon"""
        xc = 0.05
        yc = 0.05
        ttx = [x + xc for x in tx]
        tty = [y + yc for y in ty]
        ax.fill(ttx, tty, 'green')

        xc = 0.95
        yc = 0.95
        ttx = [x + xc for x in tx]
        tty = [y + yc for y in ty]
        ax.fill(ttx, tty, 'red')

    #ax.set_xlim(0.0, 1.0)
    #ax.set_ylim(0.0, 1.0)
    ax.set_axis_on()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    setup(ax)
    plt.tight_layout()
    plt.savefig('random_scenarios.eps')
    plt.show()
