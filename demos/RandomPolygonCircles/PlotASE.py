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
import matplotlib.ticker as ticker
from matplotlib.patches import Circle, Ellipse, Polygon
from matplotlib.collections import PatchCollection

from shapely.geometry import LineString as SLineString
from descartes import PolygonPatch as SPolygonPatch

from math import cos, sin, sqrt, atan2

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
    parser.add_argument('-pd', '--plannerdata', default=None, \
        help='(Optional) Filename of the planner data.')
    parser.add_argument('-path', '--plannerpath', default=None, \
        help='(Optional) Filename of the planner path.')
    parser.add_argument('-path2', '--plannerpath2', default=None, \
        help='(Optional) Filename of the planner path2.')
    parser.add_argument('-e', '--ellipses', default=None, \
        help='Filename of ellipse')
    parser.add_argument('-fe', '--fellipses', default=None, \
        help='Filename of fellipse')
    parser.add_argument('-ie', '--iellipses', default=None, \
        help='Filename of iellipse')
    parser.add_argument('-samples', nargs='*')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']

    invert_yaxis = False
    if args.plannerdata:
        from ompl import base as ob
        import graph_tool.all as gt
        invert_yaxis = True
        plt.switch_backend("cairo")

    fig, ax = plt.subplots(figsize=(2.5, 2.5))
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']
    colors = ['#1072b4', '#ef7f00', '#1f9c3a', '#d6191b', '#9763a6', '#965947', '#db79ae', '#c0c205', '#26b9ce', '#2a3377', '#7f3b71', '#9fa0a0']

    if False:
        """random partition 1"""
        x = [0.4, 0.6, 0.6, 0.4]
        y = [0.1, 0.1, 0.275, 0.275]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='gray')
        ax.add_patch(polygon)

        y = [0.325, 0.325, 0.675, 0.675]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='gray')
        ax.add_patch(polygon)

        y = [0.725, 0.725, 0.9, 0.9]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='gray')
        ax.add_patch(polygon)

        x = [0.3, 0.4, 0.4, 0.3]
        y = [0.4, 0.4, 0.6, 0.6]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#1072b4')
        ax.add_patch(polygon)

        x = [0.6, 0.7, 0.7, 0.6]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#1072b4')
        ax.add_patch(polygon)

        ax.plot([0.4, 0.6], [0.4, 0.4], color='#1072b4', ls='--')
        ax.plot([0.4, 0.6], [0.6, 0.6], color='#1072b4', ls='--')

        circle = Circle((0.2, 0.5), 0.1, alpha=1.0, color='#ef7f00')
        ax.add_patch(circle)

        circle = Circle((0.8, 0.5), 0.1, alpha=1.0, color='#1f9c3a')
        ax.add_patch(circle)

        x = [0.1, 0.4, 0.4, 0.1]
        y = [0.6, 0.6, 0.8, 0.8]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#d6191b')
        ax.add_patch(polygon)

        y = [0.2, 0.2, 0.4, 0.4]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#db79ae')
        ax.add_patch(polygon)

        y = [0.275, 0.275, 0.325, 0.325]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#db79ae')
        ax.add_patch(polygon)

        x = [0.6, 0.9, 0.9, 0.6]
        y = [0.6, 0.6, 0.8, 0.8]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#d6191b')
        ax.add_patch(polygon)

        y = [0.2, 0.2, 0.4, 0.4]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#db79ae')
        ax.add_patch(polygon)

        x = [0.4, 0.6, 0.6, 0.4]
        y = [0.275, 0.275, 0.325, 0.325]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#db79ae')
        ax.add_patch(polygon)

        y = [0.675, 0.675, 0.725, 0.725]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#d6191b')
        ax.add_patch(polygon)

        ax.plot([0.4, 0.6], [0.8, 0.8], color='#d6191b', ls='--')
        ax.plot([0.4, 0.6], [0.2, 0.2], color='#db79ae', ls='--')

        ax.text(0.6, 0.58, r'$\mathbf{\mathit{X}}_1$', transform=ax.transData, ha="left", va="top",  fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.15, 0.52, r'$\mathbf{\mathit{X}}_2$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.75, 0.52, r'$\mathbf{\mathit{X}}_3$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.70, 0.72, r'$\mathbf{\mathit{X}}_4$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.70, 0.32, r'$\mathbf{\mathit{X}}_5$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)

        ax.text(0.45, 0.22, r'$\mathbf{\mathit{O}}_1$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.45, 0.53, r'$\mathbf{\mathit{O}}_2$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.45, 0.82, r'$\mathbf{\mathit{O}}_3$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)

        """start goal"""
        ax.scatter(0.35, 0.5, color='green', s=12, zorder=10)
        ax.scatter(0.65, 0.5, color='red', s=12, zorder=10)
        ax.text(.33, .47, r'$\mathbf{\mathit{x}}_\mathit{s}$', transform=ax.transData, ha="left", va="top", fontsize=8, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(.63, .47, r'$\mathbf{\mathit{x}}_\mathit{g}$', transform=ax.transData, ha="left", va="top", fontsize=8, fontfamily='serif', math_fontfamily='stix', fontweight=200)
    if False:
        """random partition 2"""
        x = [0.4, 0.6, 0.6, 0.4]
        y = [0.1, 0.1, 0.275, 0.275]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='gray')
        ax.add_patch(polygon)

        y = [0.325, 0.325, 0.675, 0.675]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='gray')
        ax.add_patch(polygon)

        y = [0.725, 0.725, 0.9, 0.9]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='gray')
        ax.add_patch(polygon)

        x = [0.3, 0.4, 0.4, 0.3]
        y = [0.4, 0.4, 0.6, 0.6]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#1072b4')
        ax.add_patch(polygon)

        x = [0.6, 0.7, 0.7, 0.6]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#1072b4')
        ax.add_patch(polygon)

        x = [0.225, 0.4, 0.4, 0.225]
        y = [0.6, 0.6, 0.725, 0.725]
        ax.fill(x, y, alpha=1.0, color='#ef7f00', ec='#ef7f00')

        y = [0.275, 0.275, 0.4, 0.4]
        ax.fill(x, y, alpha=1.0, color='#ef7f00', ec='#ef7f00')

        x = [0.6, 0.775, 0.7775, 0.6]
        y = [0.6, 0.6, 0.725, 0.725]
        ax.fill(x, y, alpha=1.0, color='#ef7f00', ec='#ef7f00')

        y = [0.275, 0.275, 0.4, 0.4]
        ax.fill(x, y, alpha=1.0, color='#ef7f00', ec='#ef7f00')

        x = [0.225, 0.3, 0.3, 0.225]
        y = [0.4, 0.4, 0.6, 0.6]
        ax.fill(x, y, alpha=1.0, color='#ef7f00', ec='#ef7f00')
        x = [0.7, 0.775, 0.775, 0.7]
        ax.fill(x, y, alpha=1.0, color='#ef7f00', ec='#ef7f00')

        x = [0.4, 0.6, 0.6, 0.4]
        y = [0.275, 0.275, 0.325, 0.325]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#ef7f00')
        ax.add_patch(polygon)

        y = [0.675, 0.675, 0.725, 0.725]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#ef7f00')
        ax.add_patch(polygon)

        x = [0.15, 0.4, 0.4, 0.15]
        y = [0.725, 0.725, 0.825, 0.825]
        ax.fill(x, y, alpha=1.0, color='#1f9c3a')

        y = [0.175, 0.175, 0.275, 0.275]
        ax.fill(x, y, alpha=1.0, color='#1f9c3a')

        x = [0.6, 0.85, 0.85, 0.6]
        y = [0.725, 0.725, 0.825, 0.825]
        ax.fill(x, y, alpha=1.0, color='#1f9c3a')

        y = [0.175, 0.175, 0.275, 0.275]
        ax.fill(x, y, alpha=1.0, color='#1f9c3a')

        x = [0.15, 0.225, 0.225, 0.15]
        y = [0.275, 0.275, 0.725, 0.725]
        ax.fill(x, y, alpha=1.0, color='#1f9c3a')

        x = [0.775, 0.85, 0.85, 0.775]
        ax.fill(x, y, alpha=1.0, color='#1f9c3a')

        x = [0.05, 0.4, 0.4, 0.05]
        y = [0.825, 0.825, 0.95, 0.95]
        ax.fill(x, y, alpha=1.0, color='#d6191b')

        y = [0.05, 0.05, 0.175, 0.175]
        ax.fill(x, y, alpha=1.0, color='#d6191b')

        x = [0.6, 0.95, 0.95, 0.6]
        y = [0.825, 0.825, 0.95, 0.95]
        ax.fill(x, y, alpha=1.0, color='#d6191b')

        y = [0.05, 0.05, 0.175, 0.175]
        ax.fill(x, y, alpha=1.0, color='#d6191b')

        x = [0.05, 0.15, 0.15, 0.05]
        y = [0.175, 0.175, 0.825, 0.825]
        ax.fill(x, y, alpha=1.0, color='#d6191b')

        x = [0.85, 0.95, 0.95, 0.85]
        ax.fill(x, y, alpha=1.0, color='#d6191b')

        x = [0.4, 0.6, 0.6, 0.4]
        y = [0.9, 0.9, 0.95, 0.95]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#d6191b')
        ax.add_patch(polygon)

        y = [0.05, 0.05, 0.1, 0.1]
        polygon = Polygon(np.array([x,y]).transpose(), True, alpha=1.0, color='#d6191b')
        ax.add_patch(polygon)

        ax.text(0.6, 0.58, r'$\mathbf{\mathit{X}}_1$', transform=ax.transData, ha="left", va="top", fontsize=8, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.67, 0.68, r'$\mathbf{\mathit{X}}_2$', transform=ax.transData, ha="left", va="top", fontsize=8, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.75, 0.78, r'$\mathbf{\mathit{X}}_3$', transform=ax.transData, ha="left", va="top", fontsize=8, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.85, 0.9, r'$\mathbf{\mathit{X}}_4$', transform=ax.transData, ha="left", va="top", fontsize=8, fontfamily='serif', math_fontfamily='stix', fontweight=200)

        ax.text(0.45, 0.22, r'$\mathbf{\mathit{O}}_1$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.45, 0.53, r'$\mathbf{\mathit{O}}_2$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(0.45, 0.82, r'$\mathbf{\mathit{O}}_3$', transform=ax.transData, ha="left", va="top", fontsize=7.5, fontfamily='serif', math_fontfamily='stix', fontweight=200)

        """start goal"""
        ax.scatter(0.35, 0.5, color='green', s=12, zorder=10)
        ax.scatter(0.65, 0.5, color='red', s=12, zorder=10)
        ax.text(.33, .47, r'$\mathbf{\mathit{x}}_\mathit{s}$', transform=ax.transData, ha="left", va="top", fontsize=8, fontfamily='serif', math_fontfamily='stix', fontweight=200)
        ax.text(.63, .47, r'$\mathbf{\mathit{x}}_\mathit{g}$', transform=ax.transData, ha="left", va="top", fontsize=8, fontfamily='serif', math_fontfamily='stix', fontweight=200)
    if False:
        """adptive ellipse"""
        annulus1 = Annulus((0.5, 0.5), (0.6, 0.3), 0.10, alpha=1.0, color='#d6191b')
        ax.add_patch(annulus1)

        annulus1 = Annulus((0.5, 0.5), (0.5, 0.2), 0.07, alpha=1.0, color='#1f9c3a')
        ax.add_patch(annulus1)

        annulus1 = Annulus((0.5, 0.5), (0.43, 0.13), 0.07, alpha=1.0, color='#ef7f00')
        ax.add_patch(annulus1)

        ellipse1 = Ellipse((0.5, 0.5), 0.72, 0.12, alpha=1.0, color='#1072b4')
        ax.add_patch(ellipse1)

        x = [0.4, 0.6, 0.6, 0.4]
        y = [0.3, 0.3, 0.7, 0.7]
        polygon = Polygon(np.array([x,y]).transpose(), True, color = 'gray', alpha=1.0, ec='#CCB974', ls='--', lw=1.0)
        ax.add_patch(polygon)

        """infeasible path"""
        x = [0.35, 0.4, 0.45, 0.5, 0.55, 0.65]
        y = [0.5, 0.47, 0.52, 0.5, 0.53, 0.5]
        ax.plot(x, y, color='red', linewidth=1.0)
        ax.scatter(x, y, color='#B30059', s = 2.0, zorder=10)

        annulus1 = Annulus((0.5, 0.5), (0.5, 0.2), 0.07, alpha=1.0, facecolor='none', edgecolor='black', lw=0.2)
        ax.add_patch(annulus1)

        annulus1 = Annulus((0.5, 0.5), (0.43, 0.13), 0.07, alpha=1.0, facecolor='none', edgecolor='black', lw=0.2)
        ax.add_patch(annulus1)

        ellipse1 = Ellipse((0.5, 0.5), 0.72, 0.12, alpha=1.0, facecolor='none', edgecolor='black', lw=0.2)
        ax.add_patch(ellipse1)
    if False:
        """uniform ring"""
        annulus1 = Annulus((0.0, 0.0), (0.4, 0.4), 0.1, alpha=1.0, color='#d6191b')
        ax.add_patch(annulus1)

        ellipse1 = Circle((0.0, 0.0), 0.3, alpha=1.0, color='#1f9c3a')
        ax.add_patch(ellipse1)

        annulus1 = Annulus((0.0, -1.0), (0.8, 0.4), 0.2, alpha=1.0, color='#d6191b')
        ax.add_patch(annulus1)

        ellipse1 = Ellipse((0.0, -1.0), 1.2, 0.4, alpha=1.0, color='#1f9c3a')
        ax.add_patch(ellipse1)

        ax.scatter(-0.20, 0.0, color='#ef7f00', s=12)
        ax.scatter(-0.40, -1.0, color='#ef7f00', s=12)
        ax.annotate("",
                    xy=(-0.40, -1.0), xycoords='data',
                    xytext=(-0.20, -0.03), textcoords='data',
                    size=8, va="center", ha="center",
                    arrowprops=dict(arrowstyle="fancy",
                                    connectionstyle="arc3,rad=0.2",
                                    ls = "dashed",
                                    lw = 0.1,
                                    color='#1072b4')
                    )

        ax.scatter(0.34, 0.0, color='#ef7f00', s=12)
        ax.scatter(0.70, -1.0, color='#ef7f00', s=12)
        ax.annotate("",
                    xy=(0.70, -1.0), xycoords='data',
                    xytext=(0.32, -0.03), textcoords='data',
                    size=8, va="center", ha="center",
                    arrowprops=dict(arrowstyle="fancy",
                                    connectionstyle="arc3,rad=-0.2",
                                    ls = "dashed",
                                    lw = 0.1,
                                    color='#1072b4')
                    )
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.5, 0.5)
    if args.samples:
        markers = ['o', '^', 's', 'h', '^', 's', 'o', '^', 's', 's']
        for sample, mk in zip(args.samples, markers):
            print(sample)
            xy = []
            for line in open(sample, 'r').readlines():
                l = line.strip()
                if not l:
                    continue
                xy.append([float(x) for x in l.split(' ')])
            dxy = np.array(xy).transpose()
            x = dxy[0]
            y = dxy[1]
            ax.scatter(x, y, marker=mk, s = 4, ls='--', lw=0.1, c='#e377c2')
    if args.ellipses:
        xys = []
        for line in open(args.ellipses, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xys.append([float(x) for x in l.split(' ')])
        for xy in xys:
            x1 = xy[0]
            y1 = xy[1]
            x2 = xy[2]
            y2 = xy[3]
            a  = xy[4] * 0.5
            c  = 0.5 * sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
            b  = sqrt(a * a - c * c)
            if invert_yaxis:
                y1 = -y1
                y2 = -y2
            t  = atan2(y2 - y1, x2 - x1)
            xc = 0.5 * (x1 + x2)
            yc = 0.5 * (y1 + y2)
            #annulus1 = Annulus((xc, yc), (a + 0.1, b + 0.1), 0.10, angle=180.0 * t/np.pi, alpha=0.5, color='#ef7f00')
            #ax.add_patch(annulus1)
            ellipse1 = Ellipse((xc, yc), 2.0 * a, 2.0 * b, angle=180.0 * t/np.pi, alpha=0.5, color='#1072b4')
            ax.add_patch(ellipse1)
    if args.fellipses:
        xys = []
        for line in open(args.fellipses, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xys.append([float(x) for x in l.split(' ')])
        for xy in xys:
            x1 = xy[0]
            y1 = xy[1]
            x2 = xy[2]
            y2 = xy[3]
            a  = xy[4] * 0.5
            c  = 0.5 * sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
            b  = sqrt(a * a - c * c)
            if invert_yaxis:
                y1 = -y1
                y2 = -y2
            t  = atan2(y2 - y1, x2 - x1)
            xc = 0.5 * (x1 + x2)
            yc = 0.5 * (y1 + y2)
            annulus1 = Annulus((xc, yc), (a + 0.07, b + 0.07), 0.07, angle=180.0 * t/np.pi, alpha=1.0, color='#ef7f00', zorder=-100)
            ax.add_patch(annulus1)
            ellipse1 = Ellipse((xc, yc), 2.0 * a, 2.0 * b, angle=180.0 * t/np.pi, alpha=1.0, color='#1072b4', zorder=-100)
            ax.add_patch(ellipse1)
        for xy in xys:
            x1 = xy[0]
            y1 = xy[1]
            x2 = xy[2]
            y2 = xy[3]
            a  = xy[4] * 0.5
            c  = 0.5 * sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
            b  = sqrt(a * a - c * c)
            if invert_yaxis:
                y1 = -y1
                y2 = -y2
            t  = atan2(y2 - y1, x2 - x1)
            xc = 0.5 * (x1 + x2)
            yc = 0.5 * (y1 + y2)
            annulus1 = Annulus((xc, yc), (a + 0.07, b + 0.07), 0.07, angle=180.0 * t/np.pi, alpha=1.0, facecolor='none', edgecolor='black', lw=0.1, zorder=-100)
            ax.add_patch(annulus1)
            ellipse1 = Ellipse((xc, yc), 2.0 * a, 2.0 * b, angle=180.0 * t/np.pi, alpha=1.0, facecolor='none', edgecolor='black', lw=0.1, zorder=-100)
            ax.add_patch(ellipse1)
    if args.iellipses:
        xys = []
        for line in open(args.iellipses, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xys.append([float(x) for x in l.split(' ')])
        i = 0
        es = []
        an = []
        for xy in xys:
            x1 = xy[0]
            y1 = xy[1]
            x2 = xy[2]
            y2 = xy[3]
            a  = xy[4] * 0.5
            c  = 0.5 * sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
            b  = sqrt(a * a - c * c)
            if invert_yaxis:
                y1 = -y1
                y2 = -y2
            t  = atan2(y2 - y1, x2 - x1)
            xc = 0.5 * (x1 + x2)
            yc = 0.5 * (y1 + y2)
            annulus1 = Annulus((xc, yc), (a + 0.05, b + 0.05), 0.05, angle=180.0 * t/np.pi, alpha=1.0, color='#d6191b', zorder=-100)
            ax.add_patch(annulus1)
            ellipse1 = Ellipse((xc, yc), 2.0 * a, 2.0 * b, angle=180.0 * t/np.pi, alpha=1.0, color='#1f9c3a', zorder=-100)
            ax.add_patch(ellipse1)
        for xy in xys:
            x1 = xy[0]
            y1 = xy[1]
            x2 = xy[2]
            y2 = xy[3]
            a  = xy[4] * 0.5
            c  = 0.5 * sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
            b  = sqrt(a * a - c * c)
            if invert_yaxis:
                y1 = -y1
                y2 = -y2
            t  = atan2(y2 - y1, x2 - x1)
            xc = 0.5 * (x1 + x2)
            yc = 0.5 * (y1 + y2)
            annulus1 = Annulus((xc, yc), (a + 0.05, b + 0.05), 0.05, angle=180.0 * t/np.pi, alpha=1.0, facecolor='none', edgecolor='black', lw=0.1, zorder=-100)
            ax.add_patch(annulus1)
            ellipse1 = Ellipse((xc, yc), 2.0 * a, 2.0 * b, angle=180.0 * t/np.pi, alpha=1.0, facecolor='none', edgecolor='black', lw=0.1, zorder=-100)
            ax.add_patch(ellipse1)
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
                if invert_yaxis:
                    y = -y
                circle = Circle((x, y), r, color = 'gray')
                patches_circle.append(circle)
            elif params[0] == 1: # polygon
                params[1] = int(params[1])
                count = params[1]
                vecs = params[2:]
                x = vecs[::2]
                y = np.array(vecs[1::2])
                if invert_yaxis:
                    y = list(-np.array(vecs[1::2]))
                polygon = Polygon(np.array([x,y]).transpose(), True, color = 'gray')
                patches_polygon.append(polygon)
            elif params[0] == 2: # ellipse
                x = params[1]
                y = params[2]
                yaw = params[3]
                a = params[4]
                b = params[5]
                if invert_yaxis:
                    ellipse = Ellipse((x, -y), 2.0 * a, 2.0 * b, angle = -180.0 * yaw / np.pi, color = 'gray')
                else:
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
                if invert_yaxis:
                    p1 = (x - s * h, - y - c * h)
                    p2 = (x + s * h, - y + c * h)
                else:
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
                if invert_yaxis:
                    xy = np.array(((-a, a, a, -a),(b, b, -b, -b)))
                else:
                    xy = np.array(((-a, a, a, -a),(-b, -b, b, b)))
                xy = R.dot(xy)
                x1, y1 = xy
                x1 = x1 + x
                if invert_yaxis:
                    y1 = y1 - y
                else:
                    y1 = y1 + y
                rect = Polygon(np.array([x1,y1]).transpose(), True, color = 'gray')
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
        if invert_yaxis:
            y = -dxy[1]
        if len(dxy) == 3:
            theta = dxy[2]
        if False:
            xx = list(x)
            yy = list(y)
            xxx = xx[1:-1]
            yyy = yy[1:-1]
            x = xx[0:1] + xxx[1:-1:15] + xx[-1:]
            y = yy[0:1] + yyy[1:-1:15] + yy[-1:]
            if len(dxy) == 3:
                tt = list(theta)
                ttt = tt[1:-1]
                theta = tt[0:1] + ttt[1:-1:15] + tt[-1:]
        ax.plot(x, y, color='green', linewidth=1.5)
        ax.scatter(x[::2], y[::2], color='#B30059', s = 2.0, zorder=10)
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
        if invert_yaxis:
            y = -dxy[1]
        if len(dxy) == 3:
            theta = dxy[2]
        ax.plot(x, y, color='red', linewidth=1.0)
        ax.scatter(x, y, color='#B30059', s = 2.0, zorder=10)
    if args.plannerdata:
        pd = ob.PlannerData(ob.SpaceInformation(ob.RealVectorStateSpace(2)))
        pds= ob.PlannerDataStorage()
        pds.load(args.plannerdata, pd)
        pd.computeEdgeWeights()

        # Extract the graphml representation of the planner data
        graphml = pd.printGraphML()
        f = open("graph.graphml", 'w')
        f.write(graphml)
        f.close()

        # Load the graphml data using graph-tool
        graph = gt.load_graph("graph.graphml", fmt="xml")
        os.remove("graph.graphml")

        edgeweights = graph.edge_properties["weight"]
        colorprops = graph.new_vertex_property("string")
        colorprops2= graph.new_vertex_property("vector<float>")
        shapeprops = graph.new_vertex_property("string")
        vertexsize = graph.new_vertex_property("double")

        start = -1
        goal = -1

        for v in range(graph.num_vertices()):
            # Color and size vertices by type: start, goal, other
            if pd.isStartVertex(v):
                start = v
                colorprops[graph.vertex(v)] = "black"
                vertexsize[graph.vertex(v)] = 5
            elif pd.isGoalVertex(v):
                goal = v
                colorprops[graph.vertex(v)] = "black"
                vertexsize[graph.vertex(v)] = 5
            else:
                colorprops[graph.vertex(v)] = "black"
                vertexsize[graph.vertex(v)] = 2.5
            if pd.getVertex(v).getTag() == 2:
                #colorprops2[graph.vertex(v)] = [141.0/255.0, 211.0/255.0, 199.0/255.0, 1.0]
                colorprops2[graph.vertex(v)] = [255.0/255.0, 237.0/255.0, 111.0/255.0, 1.0]
                #colorprops2[graph.vertex(v)] = [141.0/255.0, 211.0/255.0, 199.0/255.0, 1.0]
                #colorprops2[graph.vertex(v)] = [255.0/255.0, 244.0/255.0, 159.0/255.0, 1.0]
                #colorprops2[graph.vertex(v)] = [213.0/255.0, 164.0/255.0, 179.0/255.0, 1.0]
                #colorprops2[graph.vertex(v)] = [25.0/255.0, 179.0/255.0, 64.0/255.0, 1.0]
                shapeprops[graph.vertex(v)] = "square"
            else:
                colorprops2[graph.vertex(v)] = [183.0/255.0, 63.0/255.0, 68.0/255.0, 1.0]
                shapeprops[graph.vertex(v)] = "circle"

        # default edge color is black with size 0.5:
        edgecolor = graph.new_edge_property("string")
        edgecolor2= graph.new_edge_property("vector<float>")
        edgesize = graph.new_edge_property("double")
        for e in graph.edges():
            edgecolor[e] = "black"
#            edgecolor2[e]= [158.0/255.0, 217.0/255.0, 207.0/255.0, 1.0]
            edgecolor2[e]= [110.0/255.0, 115.0/255.0, 116.0/255.0, 1.0]
            edgesize[e] = 0.001

        pos = graph.new_vertex_property("vector<double>")
        for v in range(graph.num_vertices()):
            vtx = pd.getVertex(v);
            st = vtx.getState()
            pos[graph.vertex(v)] = [st[0], -st[1]]
        gt.graph_draw(graph, pos=pos, vertex_shape=shapeprops, vertex_size=0.02025, vertex_fill_color=colorprops2, vertex_pen_width=0.001,
                      edge_pen_width=0.005, edge_color=edgecolor2, edge_end_marker='none', mplfig=ax)

    if True:
        """start goal"""
        if invert_yaxis:
            ax.scatter(0.05, -0.05, color='green', s=12, zorder=10)
            ax.scatter(0.95, -0.95, color='red', s=12, zorder=10)
        else:
            ax.scatter(0.05, 0.05, color='green', s=12, zorder=10)
            ax.scatter(0.95, 0.95, color='red', s=12, zorder=10)

    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    if invert_yaxis:
        ax.set_ylim(0.0, -1.0)

    ax.set_axis_on()
    ax.set_xticks([])
    ax.set_yticks([])
    #ax.xaxis.set_major_locator(ticker.MultipleLocator(0.1))
    #ax.yaxis.set_major_locator(ticker.MultipleLocator(0.05))
    ax.set_aspect('equal')
    #setup(ax)
    plt.tight_layout()
    plt.savefig('random_scenarios.svg')
    # plt.savefig('random_scenarios.pdf')
    if not invert_yaxis:
        plt.show()
