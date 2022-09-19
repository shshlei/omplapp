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

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Show plannar discretization result.')
    parser.add_argument('-s', '--samples', default=None, \
        help='Filename of the random samples data.')
    parser.add_argument('-hm', '--heatmap', default=None, \
        help='Filename of the heatmap data.')
    args = parser.parse_args()

    cmap = 'plasma_r'
    fig, ax = plt.subplots()
    if args.heatmap:
        z = []
        for line in open(args.heatmap, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            z.append([float(x) for x in l.split(' ')])
        delta = 0.05
        x = np.arange(delta/2.0, 1.0 + delta/2.0, delta)
        y = np.arange(delta/2.0, 1.0 + delta/2.0, delta)
        x, y = np.meshgrid(x, y)
        pcm = plt.pcolormesh(x, y, z, cmap=cmap, vmin=0.0, vmax=1.0)
        plt.colorbar()
    if args.samples:
        xy = []
        for line in open(args.samples, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy.append([float(x) for x in l.split(' ')])
        dxy = np.array(xy).transpose()
        x = dxy[0]
        y = dxy[1]
        ax.scatter(x, y, c=np.ones([len(x),1]), cmap=cmap, vmin=0.0, vmax=1.0)
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_axis_on()
    # ax.set_xticks([])
    # ax.set_yticks([])
    ax.set_aspect('equal')
    plt.show()
