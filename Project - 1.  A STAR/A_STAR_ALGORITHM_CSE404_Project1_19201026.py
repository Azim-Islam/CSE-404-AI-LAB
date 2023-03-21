from collections import defaultdict
from math import radians, cos, sin, asin, sqrt
from queue import PriorityQueue
from time import sleep
import igraph as ig
import matplotlib.pyplot as plt

input = open('input_map_graph.txt', "r").readline

# Returns the distance between two longitude latitudes.
def find_longitude_lattitude_distance(n1, n2):
        # The math module contains a function named
        # radians which converts from degrees to radians.
        lat1 = n1[0]
        lat2 = n2[0]
        lon1 = n1[1]
        lon2 = n2[1]
        lon1 = radians(lon1)
        lon2 = radians(lon2)
        lat1 = radians(lat1)
        lat2 = radians(lat2)
        
        # Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    
        c = 2 * asin(sqrt(a))
        
        # Radius of earth in kilometers. Use 3956 for miles
        r = 6371
        
        # calculate the result
        return(c * r)

# Convert a string to a CSM string.
def csm(s):
    return "\n".join(s.split(" "))

# Show a graph of nodes edges and huristics
def show_graph(nodes, edges, huristics, full_path):
    # Returns a dictionary containing the indexes of all nodes.
    d = defaultdict(int)
    for i,v  in enumerate(nodes.keys()):
         d[v] = i


    es = []
    costs = []
    vertex_color = []

    for e in edges:
        for i in edges[e]:
            es.append((d[e], d[i[0]]))
            costs.append(str(i[1])+" KM")

    # Plots a location graph.
    n_vertices = len(nodes)
         
    g = ig.Graph(n_vertices, es, directed=False)
    g["title"] = "Location Graph"
    g.vs["name"] = [csm(node+" H(n): "+str(round(huristics[node], 3))) for node in nodes.keys()]
    g.vs["huristic"] = [huristics[node] for node in huristics.keys()]
    g.es["label"] = costs
    fig, ax = plt.subplots(figsize=(20, 20))
    for i in range(len(g.es)):
        g.es[i]["label_size"] = 8

    for node in g.vs["name"]:
        node = node.split("H(n):")
        node = node[0].replace("\n", " ")
        node = node.strip()
        if goal_node in node:
            vertex_color.append("#6ab04c")
        elif start_node in node:
            vertex_color.append("#f0932b")
        elif node in full_path:
            vertex_color.append("#7ed6df")
        else:
            vertex_color.append("#dff9fb")
    edge_color = []
    # Plots a graph.
    ig.plot(
        g,
        target=ax,
        layout="graphopt",
        # bbox = (200, 200), # print nodes in a circular layout
        vertex_size=13,
        vertex_color=vertex_color,
        vertex_frame_width=4.0,
        vertex_frame_color="#686de0",
        vertex_label=g.vs["name"],
        vertex_label_size=9,
        edge_width=[1 for _ in range(len(edges))],
        edge_color='black',
        edge_label = g.es["label"],
        edge_lable_size = 5
    )
    # print(g.es)
    plt.show()

pc = lambda: print("-"*50)
s = lambda: sleep(0)

no_of_nodes, no_of_edges = map(int, input().split())

# Creates a dictionary of edges and nodes.
edges = defaultdict(list)
nodes = defaultdict(list)
heuristic = defaultdict(float)

start_node, goal_node = map(str.strip, input().split(" -> "))
pc()
print(f"Our start nodes is: {start_node}")
print(f"Our goal nodes is: {goal_node}")
pc()


# Split a string into a list of nodes and coordinates.
for i in range(no_of_nodes):
    node, coordinates = input().split(" = ")
    node = node.strip()
    coordinates = list(map(float, coordinates.split(",")))
    nodes[node] = coordinates

#Building the heuristic value of each node

for node in nodes:
      heuristic[node] = find_longitude_lattitude_distance(nodes[goal_node], nodes[node])
pc()
print("The co-ordinate values of the nodes are")
[(s(), print(f"NODE: {k} ".ljust(40)+f"Longitude: {v[1]:.4f}, Lattitude {v[0]:.4f}".rjust(25))) for k,v in nodes.items()]
pc()
print("The Heuristic values of the nodes are")
[(s(), print(f"NODE: {k} ".ljust(40)+f"Heuristic: {v:.4f} KM".rjust(0))) for k,v in heuristic.items()]

input()
for q in range(no_of_edges):
    node1, node2, path_cost = map(str.strip, input().split(" -> "))
    path_cost = float(path_cost)
    edges[node1].append((node2, path_cost))




#A* algorithm

#First we start from the starting node.
open_fringe = PriorityQueue()
closed_fringe = defaultdict(list)

# Finds a goal within a closed fringe.
#we add [g(n)+h(n), g(n), node, full_path]
open_fringe.put([heuristic[start_node]+0, 0, start_node, [start_node]])

# Returns a tuple of nodes and directories in the closed_fringe.
while not open_fringe.empty():
    f_n, g_n, node, full_path = open_fringe.get()
    # print(g_n, node, full_path)
    closed_fringe[node] = [f_n, g_n, full_path]
    print("Goin into the closed fringe ", node)
    if node == goal_node:
         pc()
         print("Reached goal node! Stopping the iteration here")
         pc()
         break
    for adj_node in edges[node]:
        if (adj_node[0] in closed_fringe and closed_fringe[adj_node[0]][0] > g_n+heuristic[adj_node[0]]+adj_node[1]) or (adj_node[0] not in closed_fringe):
            #[fn, gn, node, full_path]
            cp = full_path[:]
            cp.append(adj_node[0])
            # print("goin into closed fringe ", node)
            print(cp)
            open_fringe.put([g_n+adj_node[1]+heuristic[adj_node[0]], g_n+adj_node[1], adj_node[0], cp])


print("The final path:" + "->".join(closed_fringe[goal_node][2]))
pc()
print("Total path cost:", closed_fringe[goal_node][1], "KM")
pc()
show_graph(nodes, edges, heuristic, closed_fringe[goal_node][2])