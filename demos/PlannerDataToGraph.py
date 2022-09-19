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
    from ompl import base as ob
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob

import argparse
import os

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Change OMPL PlannerData type to GraphML.')
    parser.add_argument('-pd', '--plannerdata', default=None, \
        help='(Optional) Filename of the planner data.')
    parser.add_argument('-pos', '--position', type=bool, default=False, \
        help='(Optional) Specify if plot the graph with position property.')
    args = parser.parse_args()

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
        if args.position:
            gt.graph_draw(graph, pos=pos, output="graph.pdf")
        else:
            gt.graph_draw(graph, output="graph.pdf")
        print('\nGraph written to graph.pdf')

        graph.vertex_properties["pos"] = pos
        graph.vertex_properties["vsize"] = vertexsize
        graph.vertex_properties["vcolor"] = colorprops
        graph.edge_properties["esize"] = edgesize
        graph.edge_properties["ecolor"] = edgecolor
        graph.save("mgraph.graphml")
        print('\nGraph saved to mgraph.graphml')
    else:
        print("The input planner data file is error")
