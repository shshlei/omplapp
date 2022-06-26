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
    parser = argparse.ArgumentParser(description='Sampling motion planning statistics.')
    parser.add_argument('-sc_count', '--sc_count', default=None, \
        help='(Optional) Filename of the planner path.')
    parser.add_argument('-sc_counts', nargs='*')
    args = parser.parse_args()

    plt.style.use(['seaborn-deep', 'seaborn-paper'])
    plt.rcParams.update({'axes.grid': False})

    fig, ax = plt.subplots(figsize=(8.0, 8.0))
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
        z = dxy[2] # notoptimal
        w = dxy[3] # unchecked
        ax.plot(y, color='green', linewidth=2.5)
        ax.plot(y + z, color='red', linewidth=2.5)
        ax.plot(y + z + w, color='blue', linewidth=2.5)
    if args.sc_counts:
        txy = np.array([])
        for sc_count in args.sc_counts:
            xy = []
            for line in open(sc_count, 'r').readlines():
                l = line.strip()
                if not l:
                    continue
                xy.append([float(x) for x in l.split(' ')])
            if len(txy) == 0:
                txy = np.array(xy)
            else:
                l = min(len(txy), len(xy))
                txy = txy[:l,:] + np.array(xy)[:l,:]
        txy = txy/len(args.sc_counts)
        dxy = txy.transpose()
        x = dxy[0] # total
        y = dxy[1] # sc
        z = dxy[2] # notoptimal
        w = dxy[3] # unchecked
        ax.plot(y, color='green', linewidth=2.5)
        ax.plot(y + z, color='red', linewidth=2.5)
        ax.plot(y + z + w, color='blue', linewidth=2.5)
    ax.set_aspect('equal')
    #setup(ax)
    #plt.tight_layout()
    #plt.savefig('random_scenarios.pdf')
    plt.show()
