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
from matplotlib.patches import Circle, Polygon, Rectangle, Ellipse
from matplotlib.collections import PatchCollection

from shapely.geometry import LineString as SLineString
from descartes import PolygonPatch as SPolygonPatch

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
    parser.add_argument('-contact', '--contact_test', default=None, \
        help='(Optional) Filename of the contact.')
    parser.add_argument('-contacts', '--contact_spheres', default=None, \
        help='(Optional) Filename of the contact spheres.')
    parser.add_argument('-cp', '--collision_points', default=None, \
        help='(Optional) Filename of the collision points.')
    parser.add_argument('-sp', '--safe_points', default=None, \
        help='(Optional) Filename of the safe points.')
    args = parser.parse_args()

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']

    tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
    ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];

    tx = [0.00232897858414355, -0.006532563766459726, 0.003404498957143567];
    ty = [0.007361752832367847, 0.00424225972895909, -0.007564010952072658];

    tx = [0.00432897858414355, -0.012532563766459726, 0.006404498957143567];
    ty = [0.014361752832367847, 0.00824225972895909, -0.013564010952072658];

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
        ax.contourf(x, y, z, levels=1, colors=['#FFFFFF', '#d62728', '#2ca02c'])
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
                polygon = Polygon(np.array([x,y]).transpose(), True, color = 'gray')
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
    if args.contact_test:
        for line in open(args.contact_test, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy= [float(x) for x in l.split(' ')]
            xc = xy[0]
            yc = xy[1]
            ttx = [x + xc for x in tx]
            tty = [y + yc for y in ty]
            ax.fill(ttx, tty, '#1f9c3a', edgecolor='black', linewidth=0.8, alpha=0.5)
            ax.scatter(xc, yc, s=2.5, c='#B01E2F')

            xy = xy[2:]
            x  = xy[::2]
            y  = xy[1::2]
            #ax.scatter(x, y)
            ax.scatter(x[0], y[0], c='blue')
            ax.scatter(x[1], y[1], c='red')
    if args.contact_spheres:
        patches_circle = []
        for line in open(args.contact_spheres, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy= [float(x) for x in l.split(' ')]
            if (len(xy) >= 3):
                circle = Circle((xy[0], xy[1]), xy[2], color='red', alpha=0.5)
                patches_circle.append(circle)
            if (len(xy) >= 6):
                circle = Circle((xy[3], xy[4]), xy[5], color='red', alpha=0.5)
                patches_circle.append(circle)
        pcc = PatchCollection(patches_circle, match_original=True)
        ax.add_collection(pcc)
    if args.collision_points:
        xyrs = []
        for line in open(args.collision_points, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xyrs.append([float(x) for x in l.split(' ')])

        color2 = '#d62728'#'#B30059'#'#DD5D5F'
        patches_circle = []
        for xyr in xyrs:
            circle = Circle((xyr[0], xyr[1]), xyr[2], facecolor = color2, alpha=1.0, linestyle='--', linewidth=0.1, edgecolor='blue')
            patches_circle.append(circle)
        pcc = PatchCollection(patches_circle, match_original=True)
        ax.add_collection(pcc)

        for xyr in xyrs:
            ax.scatter(xyr[0], xyr[1], s=0.2, color='#8A0C4B')
    if args.safe_points:
        xyrs = []
        for line in open(args.safe_points, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xyrs.append([float(x) for x in l.split(' ')])

        color1 = '#B30059'#'#DD5D5F'
        color2 = '#2ca02c'#'#1f9c3a'
        patches_circle = []
        for xyr in xyrs:
            circle = Circle((xyr[0], xyr[1]), xyr[2], facecolor = color2, alpha=0.7, linestyle='--', linewidth=0.1, edgecolor='blue')
            patches_circle.append(circle)
            ax.scatter(xyr[0], xyr[1], s=0.2, color=color1)
        pcc = PatchCollection(patches_circle, match_original=True)
        ax.add_collection(pcc)

        if False:
            """sort"""
            def takeRadius(elem):
                return elem[2]
            xyrs.sort(key=takeRadius, reverse=True)
            patches_circle = []
            for i in range(30):
                xyr = xyrs[i]
                circle = Circle((xyr[0], xyr[1]), xyr[2], facecolor = color2, alpha=0.8, linestyle='--', linewidth=0.7, edgecolor='blue')
                patches_circle.append(circle)
                ax.scatter(xyr[0], xyr[1], s=2.5, color=color1)
            pcc = PatchCollection(patches_circle, match_original=True)
            ax.add_collection(pcc)

    if False:
        """wssc"""
        tx = list(np.array(tx) * 8.0)
        ty = list(np.array(ty) * 8.0)

        color1 = 'green'##1f9c3a'
        color2 = '#C73232'
        xyirs =[[0.25, 0.55, 2, 0.145]]

        for xyir in xyirs:
            xc = xyir[0]
            yc = xyir[1]
            i  = xyir[2]
            r  = xyir[3]
            ttx = [x + xc for x in tx]
            tty = [y + yc for y in ty]
            ax.fill(ttx, tty, color1, edgecolor='#084F17', linewidth=0.5, alpha=1.0)
            circle = Circle((ttx[i], tty[i]), r, color = color2, alpha=0.7)
            ax.add_patch(circle)
            ax.scatter(ttx[i], tty[i], s=20.0, c='#8CE69F')
            ax.scatter(xc, yc, s=2.5, c='#B30059')
        if True:
            xc = 0.55
            yc = 0.55
            c  = cos(-np.pi/3.0)
            s  = sin(-np.pi/3.0)
            ttx = [c * x - s * y + xc for x, y in zip(tx, ty)]
            tty = [s * x + c * y + yc for x, y in zip(tx, ty)]
            ax.fill(ttx, tty, color1, edgecolor='#084F17', linewidth=0.5, linestyle='--', alpha=1.0)
            ax.scatter(ttx[2], tty[2], s=10, c='#8CE69F', marker='s')
            ax.scatter(xc, yc, s=2.5, c='#B30059')
        ax.annotate("",
                    xy=(0.25, 0.6), xycoords='data',
                    xytext=(0.18, 0.85), textcoords='data',
                    size=5, va="center", ha="center",
                    arrowprops=dict(arrowstyle="simple",
                                    connectionstyle="arc3,rad=0.0",
                                    color='blue')
                    )
        ax.text(.10, .92, "robot", color='red', transform=ax.transAxes, ha="left", va="top", fontsize=6, fontfamily='Times New Roman', fontweight=200)
        ax.annotate("",
                    xy=(0.62, 0.6), xycoords='data',
                    xytext=(0.78, 0.82), textcoords='data',
                    size=5, va="center", ha="center",
                    arrowprops=dict(arrowstyle="simple",
                                    connectionstyle="arc3,rad=0.0",
                                    color='blue')
                    )
        ax.text(.42, .92, "a new configuration", color='blue', transform=ax.transAxes, ha="left", va="top", fontsize=6, fontfamily='Times New Roman', fontweight=200)
        ax.annotate("",
                    xy=(0.35, 0.30), xycoords='data',
                    xytext=(0.50, 0.1), textcoords='data',
                    size=5, va="center", ha="center",
                    arrowprops=dict(arrowstyle="simple",
                                    connectionstyle="arc3,rad=0.0",
                                    color='blue')
                    )
        ax.text(.15, .10, "certificate point", transform=ax.transAxes, ha="left", va="top", fontsize=6, fontfamily='Times New Roman', fontweight=200)
    if False:
        """cssc"""
        color1 = '#B30059'#'#DD5D5F'
        color2 = '#2ca02c'#'#1f9c3a'
        xyrs = [[0.18, 0.15, 0.10],
               [0.50, 0.38, 0.10],
               [0.22, 0.30, 0.05],
               [0.80, 0.22, 0.08],
               [0.55, 0.15, 0.07],
               [0.60, 0.70, 0.15],
               [0.10, 0.90, 0.25]]
        for xyr in xyrs:
            xc = xyr[0]
            yc = xyr[1]
            r  = xyr[2]
            circle = Circle((xc, yc), r, facecolor = color2, alpha=0.7, linestyle='--', linewidth=0.5, edgecolor='blue')
            ax.add_patch(circle)
            ax.scatter(xc, yc, s=0.5, c=color1)
    if True:
        xc = 0.10
        yc = 0.10
        ttx = [x + xc for x in tx]
        tty = [y + yc for y in ty]
        ax.fill(ttx, tty, 'green')
        ax.scatter(xc, yc, s=0.2, c='#B30059')

    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_axis_on()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    setup(ax)
    plt.tight_layout()
    #plt.savefig('random_scenarios.eps')
    plt.savefig('random_scenarios.pdf')
    plt.show()
