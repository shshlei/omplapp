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
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection

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
    parser.add_argument("--hard", default=False, action='store_true', help='plot the hard envrionment')
    parser.add_argument('-pd', '--plannerdata', default=None, \
        help='(Optional) Filename of the planner data.')
    parser.add_argument('-path', '--plannerpath', default=None, \
        help='(Optional) Filename of the planner path.')
    args = parser.parse_args()

    invert_yaxis = False
    if args.plannerdata:
        import graph_tool.all as gt
        from ompl import base as ob
        invert_yaxis = True
        plt.switch_backend("cairo")

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']

    fig, ax = plt.subplots()#(figsize=(1.8, 1.8))
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']
    colors = ['#1072b4', '#ef7f00', '#1f9c3a', '#d6191b', '#9763a6', '#965947', '#db79ae', '#c0c205', '#26b9ce', '#2a3377', '#7f3b71', '#9fa0a0']

    if args.hard:
        patches_rectangle = []
        if invert_yaxis:
            rect = Rectangle((0.15, -0.15), 0.1, -0.34, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, -0.51), 0.1, -0.34, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.75, -0.15), 0.1, -0.7, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, -0.75), 0.7, -0.1, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, -0.15), 0.7, -0.1, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, -0.51), 0.5, -0.1, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, -0.39), 0.5, -0.1, color='gray')
            patches_rectangle.append(rect)
        else:
            rect = Rectangle((0.15, 0.15), 0.1, 0.34, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, 0.51), 0.1, 0.34, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.75, 0.15), 0.1, 0.7, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, 0.75), 0.7, 0.1, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, 0.15), 0.7, 0.1, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, 0.51), 0.5, 0.1, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.15, 0.39), 0.5, 0.1, color='gray')
            patches_rectangle.append(rect)
        pcrect = PatchCollection(patches_rectangle, match_original=False)
        pcrect.set_color('gray')
        ax.add_collection(pcrect)
    else:
        patches_rectangle = []
        if invert_yaxis:
            rect = Rectangle((0.4, 0.0), 0.2, -0.49, alpha=1.0, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.4, -0.51), 0.2, -0.49, alpha=1.0, color='gray')
            patches_rectangle.append(rect)
        else:
            rect = Rectangle((0.4, 0.0), 0.2, 0.49, alpha=1.0, color='gray')
            patches_rectangle.append(rect)
            rect = Rectangle((0.4, 0.51), 0.2, 0.49, alpha=1.0, color='gray')
            patches_rectangle.append(rect)
        pcrect = PatchCollection(patches_rectangle, match_original=False)
        pcrect.set_color('gray')
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
        if False:
            xx = list(x)
            yy = list(y)
            xxx = xx[1:-1]
            yyy = yy[1:-1]
            x = xx[0:1] + xxx[1:-1:15] + xx[-1:]
            y = yy[0:1] + yyy[1:-1:15] + yy[-1:]
        ax.plot(x, y, color='green', linewidth=1.5)
        ax.scatter(x[::2], y[::2], color='#B30059', s = 2.0, zorder=10)
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
        #gt.graph_draw(graph, pos=pos, vertex_shape=shapeprops, vertex_size=0.02025, vertex_fill_color=colorprops2, vertex_pen_width=0.001,
        #              edge_pen_width=0.005, edge_color=edgecolor2, edge_end_marker='none', mplfig=ax)
        gt.graph_draw(graph, pos=pos, vertex_shape=shapeprops, vertex_size=0.01025, vertex_fill_color=colorprops2, vertex_pen_width=0.001,
                      edge_pen_width=0.005, edge_color=edgecolor2, edge_end_marker='none', mplfig=ax)

    """start goal"""
    if invert_yaxis:
        if args.hard:
            ax.scatter(0.3, -0.3, color='green', s=12, zorder=10)
            ax.scatter(0.05, -0.95, color='red', s=12, zorder=10)
        else:
            ax.scatter(0.05, -0.05, color='green', s=12, zorder=10)
            ax.scatter(0.95, -0.95, color='red', s=12, zorder=10)
    else:
        if args.hard:
            ax.scatter(0.3, 0.3, color='green', s=12, zorder=10)
            ax.scatter(0.05, 0.95, color='red', s=12, zorder=10)
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
    plt.savefig('narrowpassages.pdf')
    if not invert_yaxis:
        plt.show()
