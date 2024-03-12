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

from shapely.geometry import LineString as SLineString
from descartes import PolygonPatch as SPolygonPatch

from math import cos, sin, sqrt

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw Trajectory.')
    parser.add_argument('-s', '--scenario', default=None, help='Scenario.')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    colors = ['#1072b4', '#ef7f00', '#1f9c3a', '#d6191b', '#9763a6', '#965947', '#db79ae', '#c0c205', '#26b9ce', '#2a3377', '#7f3b71', '#9fa0a0']

    time1 = []
    for line in open(str('TimeCost/box_time_'+str(args.scenario)+'.txt'), 'r').readlines():
        line = line.strip()
        if not line:
            continue
        t = [float(x) for x in line.split(' ')]
        time1.append(t)

    time2 = []
    for line in open(str('TimeCost/box_ap_time_'+str(args.scenario)+'.txt'), 'r').readlines():
        line = line.strip()
        if not line:
            continue
        t = [float(x) for x in line.split(' ')]
        time2.append(t)

    print(np.max(time1[1]))

    time = []
    time.append(time2[1])
    time.append(time1[1])

    fig, ax = plt.subplots(figsize=(3.0, 2.0))
    bplot = ax.boxplot(time, notch=0, patch_artist=True)#, sym='k+', vert=1, whis=1.5, bootstrap=1000)

    ax.yaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=8)
    ax.xaxis.set_tick_params(which='major', direction = 'in', width=1.0, length=3, labelsize=8)
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)

    ax.set_ylabel('Time (s)', fontsize=8)

    labels = ['Box-Ap', 'Box']
    ticks = ax.get_xticks()
    texts = ax.set_xticklabels(labels, rotation=0, fontsize=8)
    texts[0].set_fontweight('bold')

    markers = ['o', '^', 's', 'o', '^', 's', 'o', '^', 's', 's']
    bcolors = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974']
    bcolors = ['#08d9d6', '#ffbd59', '#3EC1D3', '#FFFD8C', '#F6416C']
    bcolors = ['#1072b4', '#ef7f00', '#1f9c3a', '#d6191b', '#9763a6', '#965947', '#db79ae', '#c0c205', '#26b9ce', '#2a3377', '#7f3b71', '#9fa0a0']
    ecolors = ['black', '#1072b4', '#EF7F00', '#1F9C3A', '#D6191B']
    lw = 1.0
    for box, median, color, ecolor in zip(bplot['boxes'], bplot['medians'], bcolors, ecolors):
        box.set_facecolor(color)
        box.set_edgecolor(ecolor)
        median.set_color('dimgray')
        box.set_linewidth(lw)
        median.set_linewidth(lw)
    for whiskers in bplot['whiskers']:
        whiskers.set_linewidth(lw)
    for caps in bplot['caps']:
        caps.set_linewidth(lw)
    for fliers in bplot['fliers']:
        fliers.set_markeredgewidth(lw)
        fliers.set_markersize(3.0)

    plt.tight_layout()
    plt.savefig('boxtime.svg')
    #plt.savefig('boxtime.pdf')
    plt.show()
