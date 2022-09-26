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

# Author: Ryan Luna

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

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og

import argparse

def isStateValid(state):
    x = state[0]
    y = state[1]
    return (x < 0.45 or x > 0.55 or (y > 0.45 and y < 0.55))

def useGraphTool(pd):
    # Extract the graphml representation of the planner data
    graphml = pd.printGraphML()
    f = open("graph.graphml", 'w')
    f.write(graphml)
    f.close()

    # Load the graphml data using graph-tool
    graph = gt.load_graph("graph.graphml", fmt="xml")
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

    # default edge color is black with size 0.5:
    edgecolor = graph.new_edge_property("string")
    edgesize = graph.new_edge_property("double")
    for e in graph.edges():
        edgecolor[e] = "black"
        edgesize[e] = 0.5

    # using A* to find shortest path in planner data
    if start != -1 and goal != -1:
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
        pos[graph.vertex(v)] = [st[0], st[1]]

    # Writing graph to file:
    # pos indicates the desired vertex positions, and pin=True says that we
    # really REALLY want the vertices at those positions
    # gt.graph_draw(graph, pos=pos, vertex_size=vertexsize, vertex_fill_color=colorprops,
    #               edge_pen_width=edgesize, edge_color=edgecolor,
    #               output="graph.pdf")
    gt.graph_draw(graph, pos=pos, output="graph.pdf")
    print('\nGraph written to graph.pdf')

    graph.vertex_properties["pos"] = pos
    graph.vertex_properties["vsize"] = vertexsize
    graph.vertex_properties["vcolor"] = colorprops
    graph.edge_properties["esize"] = edgesize
    graph.edge_properties["ecolor"] = edgecolor

    graph.save("mgraph.graphml")
    print('\nGraph saved to mgraph.graphml')

## Returns a structure representing the optimization objective to use
#  for optimal motion planning. This method returns an objective
#  which attempts to minimize the length in configuration space of
#  computed paths.
def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "rrt":
        return og.RRT(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")

# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
    if objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")

def plan(runTime, plannerType, objectiveType, plotData, fname, pdfname):
    space = ob.RealVectorStateSpace(2)
    space.setBounds(0.0, 1.0)
    ss = og.SimpleSetup(space)

    start = ob.State(space)
    start[0] = 0.0
    start[1] = 0.0
    goal = ob.State(space)
    goal[0] = 1.0
    goal[1] = 1.0

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    ss.setStartAndGoalStates(start, goal)

    si = ss.getSpaceInformation()
    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    ss.setOptimizationObjective(allocateObjective(si, objectiveType))
    ss.setPlanner(allocatePlanner(si, plannerType))
    ss.setup()

    # attempt to solve the problem
    solved = ss.solve(runTime)

    if solved:
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization ' \
            'objective value of {2:.4f}'.format( \
            ss.getSolutionPlannerName(), \
            ss.getSolutionPath().length(), \
            ss.getSolutionPath().cost(ss.getOptimizationObjective()).value()))

        # print the path to screen
        print("Found solution:\n%s" % ss.getSolutionPath())

        # Extracting planner data from most recent solve attempt
        pd = ob.PlannerData(si)
        ss.getPlannerData(pd)

        # Computing weights of all edges based on state space distance
        pd.computeEdgeWeights()

        if graphtool and plotData:
            useGraphTool(pd)

        # If a filename was specified, output the path as a matrix to
        # that file for visualization
        if fname:
            with open(fname, 'w') as outFile:
                outFile.write(ss.getSolutionPath().printAsMatrix())
        if pdfname:
            pds= ob.PlannerDataStorage()
            pds.store(pd, pdfname)
    else:
        print("No solution found.")

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Optimal motion planning demo program.')

    # Add a filename argument
    parser.add_argument('-t', '--runtime', type=float, default=1.0, help=\
        '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
    parser.add_argument('-p', '--planner', default='RRTstar', \
        choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar', 'RRT', \
        'SORRTstar'], \
        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    parser.add_argument('-o', '--objective', default='PathLength', \
        choices=['PathClearance', 'PathLength', 'ThresholdPathLength', \
        'WeightedLengthAndClearanceCombo'], \
        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')
    parser.add_argument('-f', '--file', default=None, \
        help='(Optional) Specify an output path for the found solution path.')
    parser.add_argument('-pdf', '--pdfile', default=None, \
        help='(Optional) Specify an output path for the planner data.')
    parser.add_argument('-pd', '--plotData', type=bool, default=False, help=\
        '(Optional) Specify if plot the planner data and path.')
    parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2], \
        help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
        ' Defaults to WARN.')

    # Parse the arguments
    args = parser.parse_args()

    # Check that time is positive
    if args.runtime <= 0:
        raise argparse.ArgumentTypeError(
            "argument -t/--runtime: invalid choice: %r (choose a positive number greater than 0)" \
            % (args.runtime,))

    # Set the log level
    if args.info == 0:
        ou.setLogLevel(ou.LOG_WARN)
    elif args.info == 1:
        ou.setLogLevel(ou.LOG_INFO)
    elif args.info == 2:
        ou.setLogLevel(ou.LOG_DEBUG)
    else:
        ou.OMPL_ERROR("Invalid log-level integer.")

    # Solve the planning problem
    plan(args.runtime, args.planner, args.objective, args.plotData, args.file, args.pdfile)
