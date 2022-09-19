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

try:
    from ompl import base as ob
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob

try:
    # graph-tool and py-OMPL have some minor issues coexisting with each other.  Both modules
    # define conversions to C++ STL containers (i.e. std::vector), and the module that is imported
    # first will have its conversions used.  Order doesn't seem to matter on Linux,
    # but on Apple, graph_tool will not be imported properly if OMPL comes first.
    import graph_tool.all as gt
    graphtool = True
except ImportError:
    print('Failed to import graph-tool.  PlannerData will not be analyzed or plotted')
    graphtool = False

import argparse
import os

import numpy as np
import matplotlib.pyplot as plt
#plt.switch_backend("cairo")
from matplotlib.patches import Circle, Polygon
from matplotlib.collections import PatchCollection

import cmasher as cmr
from math import sqrt

def func(x, y, r, delta):
    xmin, xmax = max(0.0, x - r), min(1.0, x + r)
    ymin, ymax = max(0.0, y - r), min(1.0, y + r)
    xs, xe = int((xmin - x)/delta), int((xmax - x)/delta)
    ys, ye = int((ymin - y)/delta), int((ymax - y)/delta)
    X = np.linspace(xmin, xmax, xe - xs + 1)
    Y = np.linspace(ymin, ymax, ye - ys + 1)
    Z = np.zeros([ye - ys + 1, xe - xs + 1])
    sigma = r/3.0
    wh2 = 2.0 * r * r / (delta * delta)
    for i in range(xs, xe):
        for j in range(ys, ye):
            ij2 = i*i + j*j
            if ij2 > wh2:
                continue
            ijr2 = ij2 * delta * delta
            Z[j - ys, i - xs] = np.exp(-ijr2/(2.0*sigma*sigma))
    return X, Y, Z

def func_inverty(x, y, r, delta):
    xmin, xmax = max(0.0, x - r), min(1.0, x + r)
    ymin, ymax = max(0.0, y - r), min(1.0, y + r)
    xs, xe = int((xmin - x)/delta), int((xmax - x)/delta)
    ys, ye = int((ymin - y)/delta), int((ymax - y)/delta)
    X = np.linspace(xmin, xmax, xe - xs + 1)
    Y = -np.linspace(ymin, ymax, ye - ys + 1)
    Z = np.zeros([ye - ys + 1, xe - xs + 1])
    sigma = r/3.0
    wh2 = 2.0 * r * r / (delta * delta)
    for i in range(xs, xe):
        for j in range(ys, ye):
            ij2 = i*i + j*j
            if ij2 > wh2:
                continue
            ijr2 = ij2 * delta * delta
            Z[ye -ys - (j - ys), i - xs] = np.exp(-ijr2/(2.0*sigma*sigma))
    return X, Y, Z

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
    parser.add_argument('-pd', '--plannerdata', default=None, \
        help='(Optional) Filename of the planner data.')
    parser.add_argument('-path', '--plannerpath', default=None, \
        help='(Optional) Filename of the planner path.')
    parser.add_argument('-collision', '--collision_status', default=None, \
        help='(Optional) Filename of the collision.')
    parser.add_argument('-contact', '--contact_test', default=None, \
        help='(Optional) Filename of the contact.')
    parser.add_argument('-contacts', '--contact_spheres', default=None, \
        help='(Optional) Filename of the contact spheres.')
    parser.add_argument('-sc', '--safety_certificate', default=None, \
        help='(Optional) Filename of the safety certificate spheres.')
    parser.add_argument('-cc', '--collision_certificate', default=None, \
        help='(Optional) Filename of the collision certificate spheres.')
    args = parser.parse_args()

    invert_yaxis = False
    if args.plannerdata:
        invert_yaxis = True

    plt.style.use(['seaborn-deep', 'seaborn-paper'])
    plt.rcParams.update({'axes.grid': False})

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
        #cmap = 'RdBu_r'
        #pcm = plt.pcolormesh(x, y, z, cmap=cmap, vmin=0.0, vmax=1.0, shading='nearest')
        #['#85878b', '#73cdc9', '#afceff', '#ffafaf']
        colors = ['#FFFFFF', '#d6191b', '#9763a6', '#965947', '#db79ae', '#c0c205', '#26b9ce', '#2a3377', '#7f3b71', '#9fa0a0']
        ax.contourf(x, y, z, levels=1, colors=colors)
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

        color2 = '#d6191b'
        patches_circle = []
        for xy, sphere1, sphere2, point1, point2 in zip(xys, sphere1s, sphere2s, point1s, point2s):
            xc = xy[0] + sphere2[0] - sphere1[0]
            yc = xy[1] + sphere2[1] - sphere1[1]
            r = sqrt(sphere1[2] * sphere1[2] + sphere2[2] * sphere2[2])
            if invert_yaxis:
                circle = Circle((xc, -yc), r, color = color2)
                patches_circle.append(circle)
            else:
                circle = Circle((xc, yc), r, color = color2)
                patches_circle.append(circle)

            xc = xy[0] + point2[0] - sphere1[0]
            yc = xy[1] + point2[1] - sphere1[1]
            r = sphere1[2]
            if invert_yaxis:
                circle = Circle((xc, -yc), r, color = color2)
                patches_circle.append(circle)
            else:
                circle = Circle((xc, yc), r, color = color2)
                patches_circle.append(circle)

            xc = xy[0] + sphere2[0] - point1[0]
            yc = xy[1] + sphere2[1] - point1[1]
            r = sphere2[2]
            if invert_yaxis:
                circle = Circle((xc, -yc), r, color = color2)
                patches_circle.append(circle)
            else:
                circle = Circle((xc, yc), r, color = color2)
                patches_circle.append(circle)
        pcc = PatchCollection(patches_circle, match_original=True)
        ax.add_collection(pcc)
    if args.plannerpath:
        xy = []
        for line in open(args.plannerpath, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy.append([float(x) for x in l.split(' ')])
        dxy = np.array(xy).transpose()
        x = dxy[0]
        if invert_yaxis:
            y = -dxy[1]
        else:
            y = dxy[1]
        ax.plot(x, y, color='green', linewidth=2.5)
        if False:
            for xc, yc in zip(x, y):
                tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
                ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
                tx = [x + xc for x in tx]
                ty = [y + yc for y in ty]
                ax.fill(tx, ty, 'green')
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
                if invert_yaxis:
                    y = -params[2]
                else:
                    y = params[2]
                r = params[3]
                circle = Circle((x, y), r, color = 'gray')
                patches_circle.append(circle)
            elif params[0] == 1: # polygon
                params[1] = int(params[1])
                count = params[1]
                vecs = params[2:]
                x = vecs[::2]
                if invert_yaxis:
                    y = list(-np.array(vecs[1::2]))
                else:
                    y = np.array(vecs[1::2])
                polygon = Polygon(np.array([x,y]).transpose(), True, color = 'gray')
                patches_polygon.append(polygon)
        pcc = PatchCollection(patches_circle, match_original=False)
        pcp = PatchCollection(patches_polygon, match_original=False)
        if args.collision_status:
            pcc.set_color('#85878b')
            pcp.set_color('#85878b')
            pcc.set_alpha(0.7)
            pcp.set_alpha(0.7)
            pcc.set_linestyle('dashed')
            pcp.set_linestyle('dashed')
            pcc.set_edgecolor('#73cdc9')
            pcp.set_edgecolor('#73cdc9')
            pcc.set_linewidth(0.8)
            pcp.set_linewidth(0.8)
            pcc.set_joinstyle('round')
            pcp.set_joinstyle('round')
        else:
            pcc.set_color('gray')
            pcp.set_color('gray')
        ax.add_collection(pcc)
        ax.add_collection(pcp)
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
        # Write some interesting statistics
        avgdeg, stddevdeg = gt.vertex_average(graph, "total")
        avgwt, stddevwt = gt.edge_average(graph, edgeweights)

        print("---- PLANNER DATA STATISTICS ----")
        print(str(graph.num_vertices()) + " vertices and " + str(graph.num_edges()) + " edges")
        print("Average vertex degree (in+out) = " + str(avgdeg) + "  St. Dev = " + str(stddevdeg))
        print("Average edge weight = " + str(avgwt)  + "  St. Dev = " + str(stddevwt))

        _, hist = gt.label_components(graph)
        print("Strongly connected components: " + str(len(hist)))

        # Make the graph undirected (for weak components, and a simpler drawing)
        graph.set_directed(False)
        _, hist = gt.label_components(graph)
        print("Weakly connected components: " + str(len(hist)))

        # Plotting the graph
        gt.remove_parallel_edges(graph) # Removing any superfluous edges

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
                colorprops[graph.vertex(v)] = "cyan"
                vertexsize[graph.vertex(v)] = 10
            elif pd.isGoalVertex(v):
                goal = v
                colorprops[graph.vertex(v)] = "green"
                vertexsize[graph.vertex(v)] = 10
            else:
                colorprops[graph.vertex(v)] = "yellow"
                vertexsize[graph.vertex(v)] = 5
            if pd.getVertex(v).getTag() == 2:
#                colorprops2[graph.vertex(v)] = [141.0/255.0, 211.0/255.0, 199.0/255.0, 1.0]
#                colorprops2[graph.vertex(v)] = [255.0/255.0, 237.0/255.0, 111.0/255.0, 1.0]
#                colorprops2[graph.vertex(v)] = [141.0/255.0, 211.0/255.0, 199.0/255.0, 1.0]
#                colorprops2[graph.vertex(v)] = [255.0/255.0, 244.0/255.0, 159.0/255.0, 1.0]
#                colorprops2[graph.vertex(v)] = [213.0/255.0, 164.0/255.0, 179.0/255.0, 1.0]
                colorprops2[graph.vertex(v)] = [25.0/255.0, 179.0/255.0, 64.0/255.0, 1.0]
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
            edgesize[e] = 0.5

        # using A* to find shortest path in planner data
        if False and start != -1 and goal != -1:
            _, pred = gt.astar_search(graph, graph.vertex(start), edgeweights)

            # Color edges along shortest path red with size 3.0
            v = graph.vertex(goal)
            while v != graph.vertex(start):
                p = graph.vertex(pred[v])
                for e in p.out_edges():
                    if e.target() == v:
                        edgecolor[e] = "red"
                        edgesize[e] = 2.0
                v = p

        pos = graph.new_vertex_property("vector<double>")
        for v in range(graph.num_vertices()):
            vtx = pd.getVertex(v);
            st = vtx.getState()
            pos[graph.vertex(v)] = [st[0], -st[1]]
            if False:
                xc = st[0]
                yc = -st[1]
                tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
                ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
                tx = [x + xc for x in tx]
                ty = [y + yc for y in ty]
                ax.fill(tx, ty, '#1f9c3a', edgecolor='black', linewidth=0.8, alpha=0.5)
                ax.scatter(xc, yc, s=2.5, c='#B01E2F')

        gt.graph_draw(graph, pos=pos, vertex_shape=shapeprops, vertex_size=0.0125, vertex_fill_color=colorprops2, vertex_pen_width=0.001, edge_pen_width=0.003, edge_color=edgecolor2, mplfig=ax)
    if args.contact_test:
        for line in open(args.contact_test, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xy= [float(x) for x in l.split(' ')]
            xc = xy[0]
            yc = xy[1]
            tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
            ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
            tx = [x + xc for x in tx]
            ty = [y + yc for y in ty]
            ax.fill(tx, ty, '#1f9c3a', edgecolor='black', linewidth=0.8, alpha=0.5)
            ax.scatter(xc, yc, s=2.5, c='#B01E2F')

            xy = xy[2:]
            x  = xy[::2]
            y  = xy[1::2]
            ax.scatter(x, y)
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
    if args.safety_certificate:
        xyr = []
        for line in open(args.safety_certificate, 'r').readlines():
            l = line.strip()
            if not l:
                continue
            xyr.append([float(x) for x in l.split(' ')])
        xyr = np.array(xyr)
        xy = xyr[:, :2]
        xy, ind = np.unique(xy, axis=0, return_index=True)
        xyr = xyr[ind]

        color1 = '#DD5D5F'
        color2 = '#1f9c3a'
        patches_circle = []
        for xy in xyr:
            if invert_yaxis:
                circle = Circle((xy[0], -xy[1]), xy[2], facecolor = color2, alpha=0.5, linestyle='--', linewidth=1, edgecolor='blue')
                patches_circle.append(circle)
            else:
                circle = Circle((xy[0], xy[1]), xy[2], facecolor = color2, alpha=0.5, linestyle='--', linewidth=1, edgecolor='blue')
                patches_circle.append(circle)
            ax.scatter(xy[0], xy[1], color=color1)
        pcc = PatchCollection(patches_circle, match_original=True)
        ax.add_collection(pcc)
    if False:
        """heat map"""
        delta = 0.0005
        if invert_yaxis:
            x, y, z = func_inverty(0.59, 0.45, 0.03, delta)
        else:
            x, y, z = func(0.59, 0.45, 0.03, delta)
        x, y = np.meshgrid(x, y)
        cmap = 'RdBu'
        pcm = plt.pcolormesh(x, y, z, cmap=cmap, vmin=0.0, vmax=2.0)
        if invert_yaxis:
            pc = Circle((0.59, -0.45), 0.03, transform=ax.transData)
        else:
            pc = Circle((0.59, 0.45), 0.03, transform=ax.transData)
        pcm.set_clip_path(pc)
        ax.set_title(cmap, fontsize=14)
        plt.colorbar()
    if False:
        """start and goal circle"""
        if invert_yaxis:
            ax.scatter(0.05, -0.05, color='green', s=150.0)
            ax.scatter(0.95, -0.95, color='red', s=150.0)
        else:
            ax.scatter(0.05, 0.05, color='green', s=150.0)
            ax.scatter(0.95, 0.95, color='red', s=150.0)

    if False:
        """wssc"""
        color1 = '#1f9c3a'
        color2 = '#C73232'
        xc = 0.50
        yc = 0.28
        tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
        ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
        tx = [x + xc for x in tx]
        ty = [y + yc for y in ty]
        ax.fill(tx, ty, color1, edgecolor='black', linewidth=0.8)
        #ax.scatter(xc, yc, s=2.5, c='#B01E2F')
        circle = Circle((tx[2], ty[2]), 0.028, color = color2, alpha=0.5)
        ax.scatter(tx[2], ty[2], s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.49
        yc = 0.27
        tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
        ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
        tx = [x + xc for x in tx]
        ty = [y + yc for y in ty]
        ax.fill(tx, ty, '#8CE69F', edgecolor='black', linewidth=0.8, linestyle='--', alpha=0.5)
        #ax.scatter(xc, yc, s=2.5, c='#B01E2F')
        ax.scatter(tx[2], ty[2], s=7.5, c='#8CE69F')

        xc = 0.275
        yc = 0.35
        tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
        ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
        tx = [x + xc for x in tx]
        ty = [y + yc for y in ty]
        ax.fill(tx, ty, color1, edgecolor='black', linewidth=0.8)
        #ax.scatter(xc, yc, s=2.5, c='#B01E2F')
        circle = Circle((tx[0], ty[0]), 0.022, color = color2, alpha=0.5)
        ax.scatter(tx[0], ty[0], s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.54
        yc = 0.47
        tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
        ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
        tx = [x + xc for x in tx]
        ty = [y + yc for y in ty]
        ax.fill(tx, ty, color1, edgecolor='black', linewidth=0.8)
        #ax.scatter(xc, yc, s=2.5, c='#B01E2F')
        circle = Circle((tx[1], ty[1]), 0.025, color = color2, alpha=0.5)
        ax.scatter(tx[1], ty[1], s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.12
        yc = 0.43
        tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
        ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
        tx = [x + xc for x in tx]
        ty = [y + yc for y in ty]
        ax.fill(tx, ty, color1, edgecolor='black', linewidth=0.8)
        #ax.scatter(xc, yc, s=2.5, c='#B01E2F')
        circle = Circle((tx[0], ty[0]), 0.019, color = color2, alpha=0.5)
        ax.scatter(tx[0], ty[0], s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.40
        yc = 0.48
        tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
        ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
        tx = [x + xc for x in tx]
        ty = [y + yc for y in ty]
        ax.fill(tx, ty, color1, edgecolor='black', linewidth=0.8)
        #ax.scatter(xc, yc, s=2.5, c='#B01E2F')
        circle = Circle((tx[1], ty[1]), 0.016, color = color2, alpha=0.5)
        ax.scatter(tx[1], ty[1], s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.69
        yc = 0.25
        tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
        ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
        tx = [x + xc for x in tx]
        ty = [y + yc for y in ty]
        ax.fill(tx, ty, color1, edgecolor='black', linewidth=0.8)
        #ax.scatter(xc, yc, s=2.5, c='#B01E2F')
        circle = Circle((tx[2], ty[2]), 0.023, color = color2, alpha=0.5)
        ax.scatter(tx[2], ty[2], s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.67
        yc = 0.38
        tx = [0.0086579571682871, -0.02506512753291945, 0.012808997914287135, 0.0086579571682871];
        ty = [0.028723505664735693, 0.01648451945791818, -0.027128021904145316, 0.028723505664735693];
        tx = [x + xc for x in tx]
        ty = [y + yc for y in ty]
        ax.fill(tx, ty, color1, edgecolor='black', linewidth=0.8)
        #ax.scatter(xc, yc, s=2.5, c='#B01E2F')
        circle = Circle((tx[2], ty[2]), 0.023, color = color2, alpha=0.5)
        ax.scatter(tx[2], ty[2], s=7.5, c=color1)
        ax.add_patch(circle)

    if False:
        """cssc"""
        color1 = '#DD5D5F'
        color2 = '#1f9c3a'
        xc = 0.18
        yc = 0.15
        circle = Circle((xc, yc), 0.10, facecolor = color2, alpha=0.5, linestyle='--', linewidth=1, edgecolor='blue')
        ax.scatter(xc, yc, s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.50
        yc = 0.38
        circle = Circle((xc, yc), 0.10, facecolor = color2, alpha=0.5, linestyle='--', linewidth=1, edgecolor='blue')
        ax.scatter(xc, yc, s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.22
        yc = 0.30
        circle = Circle((xc, yc), 0.05, facecolor = color2, alpha=0.5, linestyle='--', linewidth=1, edgecolor='blue')
        ax.scatter(xc, yc, s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.80
        yc = 0.22
        circle = Circle((xc, yc), 0.08, facecolor = color2, alpha=0.5, linestyle='--', linewidth=1, edgecolor='blue')
        ax.scatter(xc, yc, s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.55
        yc = 0.15
        circle = Circle((xc, yc), 0.07, facecolor = color2, alpha=0.5, linestyle='--', linewidth=1, edgecolor='blue')
        ax.scatter(xc, yc, s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.60
        yc = 0.70
        circle = Circle((xc, yc), 0.15, facecolor = color2, alpha=0.5, linestyle='--', linewidth=1, edgecolor='blue')
        ax.scatter(xc, yc, s=7.5, c=color1)
        ax.add_patch(circle)

        xc = 0.10
        yc = 0.90
        circle = Circle((xc, yc), 0.25, facecolor = color2, alpha=0.5, linestyle='--', linewidth=1, edgecolor='blue')
        ax.scatter(xc, yc, s=7.5, c=color1)
        ax.add_patch(circle)

    ax.set_xlim(0.0, 1.0)
    if invert_yaxis:
        ax.set_ylim(0.0, -1.0)
    else:
        ax.set_ylim(0.0, 1.0)
    #if invert_yaxis:
    ax.set_axis_on()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    #setup(ax)
    plt.tight_layout()
    plt.savefig('random_scenarios.eps')
    plt.show()

    if False:
        import fitz
        pdf = fitz.open("random_scenarios.pdf")
        for page in pdf:
            pix = page.get_pixmap(alpha = False)
            pix.save("page-%i.png" % page.number)
