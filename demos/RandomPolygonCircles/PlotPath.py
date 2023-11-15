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
    parser.add_argument('-path', '--plannerpath', default=None, \
        help='(Optional) Filename of the planner path.')
    parser.add_argument('-pd', '--plannerdata', default=None, \
        help='(Optional) Filename of the planner data.')
    parser.add_argument('-pd2', '--plannerdata2', default=None, \
        help='(Optional) Filename of the planner data.')
    parser.add_argument('-paths', nargs='*')
    args = parser.parse_args()

    invert_yaxis = False
    if args.plannerdata or args.plannerdata2:
        from ompl import base as ob
        import graph_tool.all as gt
        plt.switch_backend("cairo")
        invert_yaxis = True

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']

    tx = [0.5 * 0.0086579571682871, -0.5 * 0.02506512753291945, 0.5 * 0.012808997914287135, 0.5 * 0.0086579571682871];
    ty = [0.5 * 0.028723505664735693, 0.5 * 0.01648451945791818, -0.5 * 0.027128021904145316, 0.5 * 0.028723505664735693];

    tx = [1.5*0.00232897858414355, -1.5*0.006532563766459726, 1.5*0.003404498957143567];
    ty = [1.5*0.007361752832367847, 1.5*0.00424225972895909, -1.5*0.007564010952072658];

    tx = [0.00432897858414355, -0.012532563766459726, 0.006404498957143567];
    ty = [0.014361752832367847, 0.00824225972895909, -0.013564010952072658];

    tx = [0.00232897858414355, -0.006532563766459726, 0.003404498957143567];
    ty = [0.007361752832367847, 0.00424225972895909, -0.007564010952072658];

    fig, ax = plt.subplots(figsize=(1.8, 1.8))
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
                polygon = Polygon(np.array([x,y]).transpose(), color = 'gray')
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
        if invert_yaxis:
            y = -dxy[1]
        if len(dxy) == 3:
            theta = dxy[2]
        if True:
            xx = list(x)
            yy = list(y)
            tt = list(theta)
            xxx = xx[1:-1]
            yyy = yy[1:-1]
            ttt = tt[1:-1]
            x = xx[0:1] + xxx[1:-1:10] + xx[-1:]
            y = yy[0:1] + yyy[1:-1:10] + yy[-1:]
            theta = tt[0:1] + ttt[1:-1:10] + tt[-1:]
        ax.plot(x, y, color='#B30059', linewidth=0.7)
        #ax.plot(x, y, color='green', linewidth=0.6)
        if True:
            patches_polygon = []
            if len(dxy) == 3:
                for xc, yc, t in zip(x, y, theta):
                    c  = cos(t)
                    s  = sin(t)
                    ttx = [c * x - s * y + xc for x, y in zip(tx, ty)]
                    if invert_yaxis:
                        tty = [-(s * x + c * y - yc) for x, y in zip(tx, ty)]
                    else:
                        tty = [s * x + c * y + yc for x, y in zip(tx, ty)]
                    polygon = Polygon(np.array([ttx, tty]).transpose(), color = 'gray')
                    patches_polygon.append(polygon)
            else:
                for xc, yc in zip(x, y):
                    ttx = [x + xc for x in tx]
                    if invert_yaxis:
                        tty = [-(y - yc) for y in ty]
                    else:
                        tty = [y + yc for y in ty]
                    polygon = Polygon(np.array([ttx, tty]).transpose(), color = 'gray')
                    patches_polygon.append(polygon)
            pcp = PatchCollection(patches_polygon, match_original=False, zorder=10)
            pcp.set_color('green')
            ax.add_collection(pcp)
        #ax.scatter(x, y, color='#B30059', s = 0.3)
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

        pos = graph.new_vertex_property("vector<float>")
        for v in range(graph.num_vertices()):
            vtx = pd.getVertex(v);
            st = vtx.getState()
            pos[graph.vertex(v)] = [st[0], -st[1]]

        gt.graph_draw(graph, pos=pos, vertex_shape=shapeprops, vertex_size=0.0125, vertex_fill_color=colorprops2, vertex_pen_width=0.001, edge_pen_width=0.003, edge_color=edgecolor2, mplfig=ax)
    if args.plannerdata2:
        pd = ob.PlannerData(ob.SpaceInformation(ob.SE2StateSpace()))
        pds= ob.PlannerDataStorage()
        pds.load(args.plannerdata2, pd)
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
                #colorprops2[graph.vertex(v)] = [141.0/255.0, 211.0/255.0, 199.0/255.0, 1.0]
                #colorprops2[graph.vertex(v)] = [255.0/255.0, 237.0/255.0, 111.0/255.0, 1.0]
                #colorprops2[graph.vertex(v)] = [141.0/255.0, 211.0/255.0, 199.0/255.0, 1.0]
                #colorprops2[graph.vertex(v)] = [255.0/255.0, 244.0/255.0, 159.0/255.0, 1.0]
                #colorprops2[graph.vertex(v)] = [213.0/255.0, 164.0/255.0, 179.0/255.0, 1.0]
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

        pos = graph.new_vertex_property("vector<float>")
        for v in range(graph.num_vertices()):
            vtx = pd.getVertex(v);
            st = vtx.getState()
            pos[graph.vertex(v)] = [st.getX(), -st.getY()]

        #gt.graph_draw(graph, pos=pos, vertex_shape=shapeprops, vertex_size=0.0125, vertex_fill_color=colorprops2, vertex_pen_width=0.001, edge_pen_width=0.003, edge_color=edgecolor2, mplfig=ax)
        gt.graph_draw(graph, pos=pos, vertex_shape=shapeprops, vertex_size=0.01, vertex_fill_color=colorprops2, vertex_pen_width=0.001, edge_pen_width=0.003, edge_color=edgecolor2, mplfig=ax)
    if args.paths:
        for path in args.paths:
            print(path)
            xy = []
            for line in open(path, 'r').readlines():
                l = line.strip()
                if not l:
                    continue
                xy.append([float(x) for x in l.split(' ')])
            dxy = np.array(xy).transpose()
            x = dxy[0]
            y = dxy[1]
            if invert_yaxis:
                y = -dxy[1]
            xx = list(x)
            yy = list(y)
            xxx = xx[1:-1]
            yyy = yy[1:-1]
            x = xx[0:1] + xxx[1:-1:25] + xx[-1:]
            y = yy[0:1] + yyy[1:-1:25] + yy[-1:]
            ax.scatter(x, y, s = 8)
            ax.plot(x, y, linewidth=2.0)

    if False:
        """start and goal polygon"""
        xc = 0.05
        yc = 0.05
        ttx = [x + xc for x in tx]
        tty = [y + yc for y in ty]
        if invert_yaxis:
            tty = [-(y + yc) for y in ty]
            yc = -0.05
        ax.fill(ttx, tty, 'green', zorder=20)
        ax.scatter(xc, yc, color='#B30059', s = 0.2)

        ax.annotate("",
                    xy=(0.05, 0.05), xycoords='data',
                    xytext=(0.15, 0.15), textcoords='data',
                    size=5, va="center", ha="center",
                    arrowprops=dict(arrowstyle="simple",
                                    connectionstyle="arc3,rad=0.0",
                                    ls = "dashed",
                                    lw = 0.1,
                                    color='blue')
                    )
        ax.text(.15, .15, r'$\mathrm{start}$', color='blue', transform=ax.transAxes, ha="left", va="top", fontsize=5)

        xc = 0.95
        yc = 0.95
        ttx = [x + xc for x in tx]
        tty = [y + yc for y in ty]
        if invert_yaxis:
            tty = [-(y + yc) for y in ty]
            yc = -0.95
        ax.fill(ttx, tty, 'red', zorder=20)
        ax.scatter(xc, yc, color='#B30059', s = 0.2)
        ax.annotate("",
                    xy=(0.95, 0.95), xycoords='data',
                    xytext=(0.85, 0.85), textcoords='data',
                    size=5, va="center", ha="center",
                    arrowprops=dict(arrowstyle="simple",
                                    connectionstyle="arc3,rad=0.0",
                                    ls = "dashed",
                                    lw = 0.1,
                                    color='blue')
                    )
        ax.text(.85, .85, r'$\mathrm{goal}$', color='blue', transform=ax.transAxes, ha="left", va="top", fontsize=5)

    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    if invert_yaxis:
        ax.set_ylim(0.0, -1.0)

    ax.set_axis_on()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    #setup(ax)
    plt.tight_layout()
    plt.savefig('random_scenarios.svg')
    if invert_yaxis:
        plt.savefig('random_scenarios.pdf')
    else:
        plt.savefig('random_scenarios.eps')
        plt.show()

