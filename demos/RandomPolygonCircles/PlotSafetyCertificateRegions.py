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
from matplotlib.patches import Circle, Polygon, Rectangle, Ellipse
from matplotlib.collections import PatchCollection

from shapely.geometry import LineString as SLineString
from descartes import PolygonPatch as SPolygonPatch


from math import cos, sin, sqrt

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Show random scenario motion planning result.')
    parser.add_argument('-s', '--scenario', default=None, \
        help='Filename of random scenario')
    parser.add_argument('-collision', '--collision_status', default=None, \
        help='(Optional) Filename of the collision.')
    parser.add_argument('-sc', '--safety_certificate', default=None, \
        help='(Optional) Filename of the safety certificate spheres.')
    parser.add_argument('-cc', '--collision_certificate', default=None, \
        help='(Optional) Filename of the collision certificate spheres.')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']

    fig, ax = plt.subplots(figsize=(2.5, 2.5))
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
        #ax.contourf(x, y, z, levels=1, colors=['#2ca02c', '#FFFFFF', '#d62728'])
        ax.contourf(x, y, z, levels=1, colors=['#FFFFFF', '#d62728'])
    if args.safety_certificate:
        xyrs = []
        for line in open(args.safety_certificate, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xyrs.append([float(x) for x in l.split(' ')])
        xyrs = np.array(xyrs)
        xys = xyrs
        #xys = xyrs[:, :2]
        xys, ind = np.unique(xys, axis=0, return_index=True)
        xyrs = xyrs[ind]

        color1 = '#B30059'#'#DD5D5F'
        color2 = '#2ca02c'#'#1f9c3a'
        patches_circle = []
        for xyr in xyrs:
            xc = xyr[0]
            yc = xyr[1]
            r  = xyr[2]
            circle = Circle((xc, yc), r, facecolor=color2, alpha=0.7, linestyle='--', linewidth=0.5, edgecolor='blue')
            patches_circle.append(circle)
            ax.scatter(xc, yc, s=2.0, color=color1, zorder=10)
        pcc = PatchCollection(patches_circle, match_original=True)
        ax.add_collection(pcc)
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
                rect = Polygon(np.array([x1,y1]).transpose(), True, color = 'gray')
                patches_rectangle.append(rect)
        pcc = PatchCollection(patches_circle, match_original=False)
        pcp = PatchCollection(patches_polygon, match_original=False)
        pce = PatchCollection(patches_ellipse, match_original=False)
        pccap = PatchCollection(patches_capsule, match_original=False)
        pcrect = PatchCollection(patches_rectangle, match_original=False)
        if args.collision_status:
            pcc.set_color('#85878b')
            pcp.set_color('#85878b')
            pce.set_color('#85878b')
            pccap.set_color('#85878b')
            pcrect.set_color('#85878b')
            pcc.set_alpha(0.3)
            pcp.set_alpha(0.3)
            pce.set_alpha(0.3)
            pccap.set_alpha(0.3)
            pcrect.set_alpha(0.3)
            pcc.set_linestyle('dashed')
            pcp.set_linestyle('dashed')
            pce.set_linestyle('dashed')
            pccap.set_linestyle('dashed')
            pcrect.set_linestyle('dashed')
            pcc.set_edgecolor('#73cdc9')
            pcp.set_edgecolor('#73cdc9')
            pce.set_edgecolor('#73cdc9')
            pccap.set_edgecolor('#73cdc9')
            pcrect.set_edgecolor('#73cdc9')
            pcc.set_linewidth(0.4)
            pcp.set_linewidth(0.4)
            pce.set_linewidth(0.4)
            pccap.set_linewidth(0.4)
            pcrect.set_linewidth(0.4)
            pcc.set_joinstyle('round')
            pcp.set_joinstyle('round')
            pce.set_joinstyle('round')
            pccap.set_joinstyle('round')
            pcrect.set_joinstyle('round')
        else:
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

    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_axis_on()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    plt.tight_layout()
    #plt.savefig('random_scenarios.eps')
    #plt.savefig('safety_circles.svg')
    plt.show()
