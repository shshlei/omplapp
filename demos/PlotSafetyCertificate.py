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
import matplotlib
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
    parser = argparse.ArgumentParser(description='Show random scenario motion planning result.')
    parser.add_argument('-s', '--scenario', default=None, \
        help='Filename of random scenario')
    parser.add_argument('-collision', '--collision_status', default=None, \
        help='(Optional) Filename of the collision.')
    parser.add_argument('-sc_pure_sampling', '--sc_pure_sampling', default=None, \
        help='(optional) filename of the sc pure sampling.')
    parser.add_argument('-sc_time', '--sc_time', default=None, \
        help='(optional) filename of the sc time.')
    parser.add_argument('-sc_count', '--sc_count', default=None, \
        help='(optional) filename of the planner path.')
    parser.add_argument('-sc_counts', nargs='*')
    parser.add_argument('-sc_counts1', nargs='*')
    parser.add_argument('-sc_counts2', nargs='*')
    parser.add_argument('-sc', '--safety_certificate', default=None, \
        help='(Optional) Filename of the safety certificate spheres.')
    parser.add_argument('-cc', '--collision_certificate', default=None, \
        help='(Optional) Filename of the collision certificate spheres.')
    args = parser.parse_args()

    plt.style.use(['seaborn-deep', 'seaborn-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']

    tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
    ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];

    fig, ax = plt.subplots(figsize=(1.8, 1.8))
    if args.collision_status:
        z = []
        for line in open(args.collision_status, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            z.append([float(x) for x in l.split(' ')])
        x = np.linspace(0.0, 1.0, len(z[0]))
        y = np.linspace(0.0, 1.0, len(z[0]))
        x, y = np.meshgrid(x, y)
        ax.contourf(x, y, z, levels=1, colors=['#2ca02c', '#FFFFFF', '#d62728'])
    if args.collision_certificate:
        data = []
        for line in open(args.collision_certificate, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            data.append([float(x) for x in l.split(' ')])
        data = np.array(data)
        xys = data[:, :2]
        sphere1s = data[:, 2:5]
        sphere2s = data[:, 5:8]
        point1s= data[:, 8:10]
        point2s = data[:, 10:12]

        color2 = '#d62728'#'#B30059'#'#DD5D5F'
        patches_circle = []
        for xy, sphere1, sphere2, point1, point2 in zip(xys, sphere1s, sphere2s, point1s, point2s):
            xc = xy[0] + sphere2[0] - sphere1[0]
            yc = xy[1] + sphere2[1] - sphere1[1]
            r = sqrt(sphere1[2] * sphere1[2] + sphere2[2] * sphere2[2])
            circle = Circle((xc, yc), r, facecolor = color2, alpha=1.0, linestyle='--', linewidth=0.1, edgecolor='blue')
            patches_circle.append(circle)

            xc = xy[0] + point2[0] - sphere1[0]
            yc = xy[1] + point2[1] - sphere1[1]
            r = sphere1[2]
            circle = Circle((xc, yc), r, color = color2)
            patches_circle.append(circle)

            xc = xy[0] + sphere2[0] - point1[0]
            yc = xy[1] + sphere2[1] - point1[1]
            r = sphere2[2]
            circle = Circle((xc, yc), r, color = color2)
            patches_circle.append(circle)
        pcc = PatchCollection(patches_circle, match_original=True)
        ax.add_collection(pcc)

        for xy, sphere1, sphere2, point1, point2 in zip(xys, sphere1s, sphere2s, point1s, point2s):
            xc = xy[0] + sphere2[0] - sphere1[0]
            yc = xy[1] + sphere2[1] - sphere1[1]
            ax.scatter(xc, yc, s=0.1, color='#8A0C4B')

            xc = xy[0] + point2[0] - sphere1[0]
            yc = xy[1] + point2[1] - sphere1[1]
            ax.scatter(xc, yc, s=0.1, color='#8A0C4B')

            xc = xy[0] + sphere2[0] - point1[0]
            yc = xy[1] + sphere2[1] - point1[1]
            ax.scatter(xc, yc, s=0.1, color='#8A0C4B')
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
        if args.collision_status:
            pcc.set_color('#85878b')
            pcp.set_color('#85878b')
            pcc.set_alpha(0.3)
            pcp.set_alpha(0.3)
            pcc.set_linestyle('dashed')
            pcp.set_linestyle('dashed')
            pcc.set_edgecolor('#73cdc9')
            pcp.set_edgecolor('#73cdc9')
            pcc.set_linewidth(0.4)
            pcp.set_linewidth(0.4)
            pcc.set_joinstyle('round')
            pcp.set_joinstyle('round')
        else:
            pcc.set_color('gray')
            pcp.set_color('gray')
        ax.add_collection(pcc)
        ax.add_collection(pcp)
    if args.safety_certificate:
        xyrs = []
        for line in open(args.safety_certificate, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xyrs.append([float(x) for x in l.split(' ')])
        xyrs = np.array(xyrs)
        xys = xyrs[:, :2]
        xys, ind = np.unique(xys, axis=0, return_index=True)
        xyrs = xyrs[ind]

        color1 = '#B30059'#'#DD5D5F'
        color2 = '#2ca02c'#'#1f9c3a'
        patches_circle = []
        for xyr in xyrs:
            xc = xyr[0]
            yc = xyr[1]
            r  = xyr[2]
            circle = Circle((xc, yc), r, facecolor = color2, alpha=0.7, linestyle='--', linewidth=0.1, edgecolor='blue')
            patches_circle.append(circle)
            ax.scatter(xc, yc, s=0.2, color=color1)
        pcc = PatchCollection(patches_circle, match_original=True)
        ax.add_collection(pcc)
    if args.sc_count:
        xy = []
        for line in open(args.sc_count, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy.append([float(x) for x in l.split(' ')])
        dxy = np.array(xy).transpose()
        x = dxy[0] # total
        y = dxy[1] # sc
        ax.plot(y, color='green', linewidth=2.5)
    if args.sc_counts:
        xys = np.array([])
        for sc_count in args.sc_counts:
            xy = []
            for line in open(sc_count, 'r').readlines():
                l = line.strip()
                if not l:
                    continue
                xy.append([float(x) for x in l.split(' ')])
            if (len(xys) == 0):
                xys = np.array(xy)
            else:
                l = min(len(xys), len(xy))
                xys = xys[:l] + np.array(xy)[:l]
        xys = xys / len(args.sc_counts)
        dxy = np.array(xys).transpose()
        x = dxy[0] # total
        y = dxy[1] # sc
        ax.plot(y, linewidth=2.5)
    if args.sc_counts1:
        xys = np.array([])
        for sc_count in args.sc_counts1:
            xy = []
            for line in open(sc_count, 'r').readlines():
                l = line.strip()
                if not l:
                    continue
                xy.append([float(x) for x in l.split(' ')])
            if (len(xys) == 0):
                xys = np.array(xy)
            else:
                l = min(len(xys), len(xy))
                xys = xys[:l] + np.array(xy)[:l]
        xys = xys / len(args.sc_counts1)
        xys = xys[0:50]
        dxy = np.array(xys).transpose()
        x = dxy[0] # total
        y = dxy[1] # sc
        ax.plot(y, linewidth=2.5, label='Real2')
    if args.sc_counts2:
        xys = np.array([])
        for sc_count in args.sc_counts2:
            xy = []
            for line in open(sc_count, 'r').readlines():
                l = line.strip()
                if not l:
                    continue
                xy.append([float(x) for x in l.split(' ')])
            if (len(xys) == 0):
                xys = np.array(xy)
            else:
                l = min(len(xys), len(xy))
                xys = xys[:l] + np.array(xy)[:l]
        xys = xys / len(args.sc_counts2)
        xys = xys[0:50]
        dxy = np.array(xys).transpose()
        x = dxy[0] # total
        y = dxy[1] # sc
        ax.plot(y, linewidth=2.5, label='SE2')
    if args.sc_pure_sampling:
        xy = []
        for line in open(args.sc_pure_sampling, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy.append([float(x) for x in l.split(' ')])
        dxy = np.array(xy).transpose()
        x = dxy[0] # total
        y = dxy[1] # sc
        ax.plot(x, x, linewidth=2.5)
        ax.plot(x, y, linewidth=2.5)
    if args.sc_time:
        xy = []
        for line in open(args.sc_time, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy.append([float(x) for x in l.split(' ')])
        dxy = np.array(xy).transpose()
        x = dxy[0] # total
        y = 100000 * dxy[1] # sc
        z = 100000 * dxy[2] # notoptimal
        ax.plot(x, y, linewidth=2.5)
        ax.plot(x, z, linewidth=2.5)

    if False:
        xc = 0.0235317
        yc = 0.403731
        c  = cos(0.435234)
        s  = sin(0.435234)
        ttx = [c * x - s * y + xc for x, y in zip(tx, ty)]
        tty = [s * x + c * y + yc for x, y in zip(tx, ty)]
        ax.fill(ttx, tty, 'green', edgecolor='#084F17', linewidth=0.5, linestyle='--', alpha=1.0)
        ax.scatter(xc, yc, s=2.5, c='#B30059')

    if False:
        xyrs = [[0.212781, 0.58101899999999995, 0.025804199999999999]]
        for xyr in xyrs:
            xc = xyr[0]
            yc = xyr[1]
            r  = xyr[2]
            circle = Circle((xc, yc), r, color = 'yellow')
            ax.add_patch(circle)

    #props = matplotlib.font_manager.FontProperties()
    #props.set_size('small')
    #ax.tick_params(labelsize=5)
    #ax.legend(framealpha = 0, loc = (0.4, 0.65), fontsize=6)
    #ax.set_xlim(0.0, 1.0)
    #ax.set_ylim(0.0, 1.0)
    ax.set_axis_on()
    #ax.set_xticks([])
    #ax.set_yticks([])
    ax.set_aspect('equal')
    setup(ax)
    plt.tight_layout()
    #plt.savefig('random_scenarios.eps')
    plt.savefig('random_scenarios.pdf')
    plt.show()
