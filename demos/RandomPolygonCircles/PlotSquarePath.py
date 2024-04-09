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
from matplotlib.patches import Circle, Ellipse, Polygon, Rectangle
from matplotlib.collections import PatchCollection

from shapely.geometry import LineString as SLineString
from descartes import PolygonPatch as SPolygonPatch

from math import cos, sin, sqrt

h_a = 0.025
h_b = 0.025

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw planned path.')
    parser.add_argument('-s', '--scenario', default=None, \
        help='Filename of random scenario')
    parser.add_argument('-path', '--plannerpath', default=None, \
        help='(Optional) Filename of the planner path.')
    parser.add_argument('-path2', '--plannerpath2', default=None, \
        help='(Optional) Filename of the planner path.')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']

    fig, ax = plt.subplots(figsize=(2.5, 2.5))
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
        pcc = PatchCollection(patches_circle, match_original=False)
        pcp = PatchCollection(patches_polygon, match_original=False)
        pce = PatchCollection(patches_ellipse, match_original=False)
        pccap = PatchCollection(patches_capsule, match_original=False)
        pcrect = PatchCollection(patches_rectangle, match_original=False)
        pcc.set_color('gray')
        pcp.set_color('gray')
        pce.set_color('gray')
        pccap.set_color('gray')
        pcrect.set_color('gray')
        ax.add_collection(pcc)
        ax.add_collection(pcp)
        ax.add_collection(pce)
        ax.add_collection(pccap)
        ax.add_collection(pcrect)

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
        ax.plot(x, y, color='#B30059', linewidth=0.7)

        if True:
            patches_rectangle = []
            if len(dxy) == 3:
                for xc, yc, t in zip(x, y, theta):
                    rect = Rectangle((xc - h_a, yc - h_b), 2 * h_a, 2 * h_b, angle=180.0* t/np.pi, rotation_point=(xc, yc), facecolor='w', edgecolor='g', lw=0.5, zorder=10, fill=False)
                    patches_rectangle.append(rect)
            else:
                for xc, yc in zip(x, y):
                    rect = Rectangle((xc - h_a, yc - h_b), 2 * h_a, 2 * h_b, facecolor='w', edgecolor='g', lw=0.5, zorder=10, fill=False)
                    patches_rectangle.append(rect)
            pcrect = PatchCollection(patches_rectangle, match_original=True)
            ax.add_collection(pcrect)

    if args.plannerpath2:
        xy = []
        for line in open(args.plannerpath2, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy.append([float(x) for x in l.split(' ')])
        dxy = np.array(xy).transpose()
        x = dxy[0]
        y = dxy[1]
        if len(dxy) == 3:
            theta = dxy[2]
        ax.plot(x, y, color='#B30059', linewidth=0.7)

        if True:
            patches_rectangle = []
            if len(dxy) == 3:
                for xc, yc, t in zip(x, y, theta):
                    rect = Rectangle((xc - h_a, yc - h_b), 2 * h_a, 2 * h_b, angle=180.0* t/np.pi, rotation_point=(xc, yc), facecolor='w', edgecolor='b', lw=0.5, zorder=10, fill=False)
                    patches_rectangle.append(rect)
            else:
                for xc, yc in zip(x, y):
                    rect = Rectangle((xc - h_a, yc - h_b), 2 * h_a, 2 * h_b, facecolor='w', edgecolor='b', lw=0.5, zorder=10, fill=False)
                    patches_rectangle.append(rect)
            pcrect = PatchCollection(patches_rectangle, match_original=True)
            ax.add_collection(pcrect)

    if True:
        """start and goal polygon"""
        rect = Rectangle((0.05 - h_a, 0.05 - h_b), 2 * h_a, 2 * h_b, color='green')
        ax.add_patch(rect)
        ax.scatter(0.05, 0.05, color='#B30059', s=10)

        rect = Rectangle((0.95 - h_a, 0.95 - h_b), 2 * h_a, 2 * h_b, color='red')
        ax.add_patch(rect)
        ax.scatter(0.95, 0.95, color='#B30059', s=10)

    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)

    ax.set_axis_on()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    plt.tight_layout()
    #plt.savefig('random_scenarios.svg')
    plt.show()
