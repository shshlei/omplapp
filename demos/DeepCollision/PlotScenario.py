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
from matplotlib.patches import Circle, Ellipse, Polygon
from matplotlib.collections import PatchCollection

from shapely.geometry import LineString as SLineString
from descartes import PolygonPatch as SPolygonPatch

from math import cos, sin, sqrt

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw scenario.')
    parser.add_argument('-s', '--scenario', default=None, \
                        help='Filename of random scenario')
    parser.add_argument('--circle', action='store_true', default=False,
                        help='Circle scenario')
    parser.add_argument('--ellipse', action='store_true', default=False,
                        help='Ellipse scenario')
    parser.add_argument('--capsule', action='store_true', default=False,
                        help='Capsule scenario')
    parser.add_argument('--rectangle', action='store_true', default=False,
                        help='Rectangle scenario')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']

    fig, ax = plt.subplots()#(figsize=(1.8, 1.8))
    if args.scenario:
        patches = []
        for line in open(args.scenario, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            params = [float(x) for x in l.split(' ')]
            if args.circle: # circle
                x = params[0]
                y = params[1]
                r = params[2]
                circle = Circle((x, y), r, color = 'gray')
                patches.append(circle)
            elif args.ellipse: # ellipse
                x = params[0]
                y = params[1]
                yaw = params[2]
                a = params[3]
                b = params[4]
                ellipse = Ellipse((x, y), 2.0 * a, 2.0 * b, angle = 180.0 * yaw / np.pi, color = 'gray')
                patches.append(ellipse)
            elif args.capsule: # capsule
                x = params[0]
                y = params[1]
                yaw = params[2]
                r = params[3]
                h = params[4]
                s = sin(yaw)
                c = cos(yaw)
                p1 = (x - s * h, y + c * h)
                p2 = (x + s * h, y - c * h)
                capsule = SLineString([p1, p2]).buffer(r)
                patches.append(SPolygonPatch(capsule, fc='gray', ec='gray'))
            elif args.rectangle: # rectangle
                x = params[0]
                y = params[1]
                yaw = params[2]
                a = params[3]
                b = params[4]
                s = sin(yaw)
                c = cos(yaw)
                R = np.array(((c, -s), (s, c)))
                xy = np.array(((-a, a, a, -a),(-b, -b, b, b)))
                xy = R.dot(xy)
                x1, y1 = xy
                x1 = x1 + x
                y1 = y1 + y
                rect = Polygon(np.array([x1,y1]).transpose(), True, color = 'gray')
                patches.append(rect)
        pcc = PatchCollection(patches, match_original=False)
        pcc.set_color('gray')
        ax.add_collection(pcc)

    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)

    ax.set_axis_on()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    #setup(ax)
    plt.tight_layout()
    plt.show()
