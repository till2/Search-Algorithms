from matplotlib import pyplot as plt
import numpy as np
import networkx as nx

"""
Node attributes: in_goal_path, seen

Edge attributes: cost

"""

# node class with parent for tracking the goal path

class Node:
    def __init__(self, state, parent_node=None, cost=0, heuristic=0, path_cost=0):
        self.state = state
        self.parent = parent_node
        self.cost = cost
        self.heuristic = heuristic
        self.path_cost = path_cost

# plot/ graph functions

def show_plt(block,t):
    plt.show(block=block) # block==True means the window stays open until manually closed
    plt.pause(t) # if block==False -> show window for t seconds
    plt.close()

def build_graph(AL):
    G = nx.DiGraph() # Directed Graph
    G.add_edges_from(AL)
    return G

def show_graph(G, t=None):
    if t is None:
        t = 0.0
        block = True
    else:
        block = False
    pos = nx.kamada_kawai_layout(G) # alternative: fruchterman_reingold_layout
    node_color_map = [attr[1] for attr in list(G.nodes(data="in_goal_path", default=False))]
    node_colors = ['green' if attr == True else 'red' for attr in node_color_map]
    nx.draw(G, pos, with_labels=True, node_size=1800,
            alpha=0.9, node_color=node_colors, edge_color='c', arrowsize=20)
    edge_labels = dict([((u, v), f'{G.get_edge_data(u, v)["cost"]}')
                    for u, v in G.edges])
    node_labels = dict([(node, f'\n\n{G.nodes[node]["heuristic"]}')
                    for node in G.nodes])
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    nx.draw_networkx_labels(G, pos, labels=node_labels)
    show_plt(block,t)

# search algorithm functions

def expand_node(node, rev):
    neighbors = []
    if G.neighbors(node.state):
        for neighbor in G.neighbors(node.state):
            # path cost for the start node is 0
            if not node.parent:
                path_cost = G.get_edge_data(node.state, 
                                            neighbor)["cost"]
            else:
                path_cost = node.path_cost + G.get_edge_data(node.state, 
                                                            neighbor)["cost"]
            heuristic = G.nodes[neighbor]["heuristic"]
            total_cost = path_cost + heuristic
            neighbor_node = Node(neighbor, node, total_cost, heuristic, path_cost)
            neighbors.append(neighbor_node)
        
        # add neighbors ordered alphabetically
        neighbors_dict = {node.state:node for node in neighbors}
        if neighbors_dict:
            sorted_neighbors = [elem[1] for elem in
                                sorted(neighbors_dict.items(),
                                key=lambda element: element[0], reverse=rev)]
            return sorted_neighbors   
    # no child nodes
    return None

def check_goal(node, goal):
    return node.state == goal

def build_goal_path(goal):
    Path = [goal.state]
    current = goal
    
    while current.parent:
        nx.set_node_attributes(G,{current.state:True},name="in_goal_path")
        Path.append(current.parent.state)
        current = current.parent
    nx.set_node_attributes(G,{current.state:True},name="in_goal_path")
    Path.reverse()
    return Path

def a_star(start, goal, rev):
    # SearchGraph = nx.DiGraph()
    start_node = Node(start, None, 0, G.nodes[start]["heuristic"], 0)
    Frontier = [start_node]
    while(Frontier):
        # print(Frontier[np.argmin([G.nodes[node.state]["heuristic"] for node in Frontier])])
        max_node_index = np.argmin([node.cost for node in Frontier])
        current = Frontier[max_node_index]
        # check if current is a goal state
        if(check_goal(current, goal)):
            print(f'goal found: {current.state}')
            return (build_goal_path(current), current.cost)

        print(f'current:   ', current.state)
        # edge_cost = G.get_edge_data(current.parent.state, current.state)["cost"]

        # remove current node from frontier
        del Frontier[max_node_index]
        
        # expand frontier
        new_frontier = expand_node(current, rev)
        if new_frontier:
            Frontier += new_frontier

        print('frontier: ', [(node.state,f'heur:{node.heuristic}',f'path:{node.path_cost}',
                              f'total:{node.cost}') for node in Frontier], '\n')
        
    # no solution
    return None


if __name__ == '__main__':
    
    # Edges = [
    #     ('A', 'B', {"cost":5}),
    #     ('A', 'C', {"cost":1}),
    #     ('B', 'E', {"cost":2}),
    #     ('C', 'E', {"cost":4}),
    #     ('C', 'D', {"cost":7}),
    #     ('E', 'F', {"cost":1}),
    #     ('E', 'D', {"cost":2}),
    #     ('D', 'G', {"cost":1}),
    #     ('D', 'H', {"cost":4}),
    #     ('F', 'G', {"cost":3}),
    #     ('G', 'H', {"cost":2})
    # ]

    # Heuristics = {
    #     'A': {"heuristic": 8},
    #     'B': {"heuristic": 6},
    #     'C': {"heuristic": 7},
    #     'D': {"heuristic": 2},
    #     'E': {"heuristic": 5},
    #     'F': {"heuristic": 1},
    #     'G': {"heuristic": 4},
    #     'H': {"heuristic": 0},
    # }

    Edges = [
        ('A', 'B', {"cost":1}),
        ('A', 'C', {"cost":6}),
        ('B', 'C', {"cost":2}),
        ('B', 'E', {"cost":4}),
        ('C', 'E', {"cost":1}),
        ('E', 'F', {"cost":1}),
        ('C', 'G', {"cost":6}),
        ('C', 'D', {"cost":4}),
        ('D', 'C', {"cost":3}),
        ('D', 'G', {"cost":1})
    ]

    Heuristics = {
        'A': {"heuristic": 5},
        'B': {"heuristic": 3},
        'C': {"heuristic": 4},
        'D': {"heuristic": 1},
        'E': {"heuristic": 2},
        'F': {"heuristic": 5},
        'G': {"heuristic": 0}
    }

    # Edges = [
    #     ('A', 'C', {"cost":3}),
    #     ('A', 'B', {"cost":6}),
    #     ('C', 'D', {"cost":2}),
    #     ('C', 'E', {"cost":4}),
    #     ('C', 'B', {"cost":2}),
    #     ('B', 'E', {"cost":1}),
    #     ('B', 'F', {"cost":7}),
    #     ('E', 'F', {"cost":5}),
    #     ('D', 'G', {"cost":4}),
    #     ('E', 'G', {"cost":2}),
    #     ('F', 'H', {"cost":5}),
    #     ('G', 'H', {"cost":5})
    # ]
    
    # Edges = [
    #         ('A', 'B', {"cost":1}),
    #         ('B', 'D', {"cost":1}),
    #         ('B', 'C', {"cost":1}),
    #         ('C', 'E', {"cost":1}),
    #         ('D', 'F', {"cost":1}),
    #         ('F', 'E', {"cost":1})
    #     ]
    
    # Heuristics = {
    #     'A': {"heuristic": 20}, 
    #     'B': {"heuristic": 5},
    #     'C': {"heuristic": 4},
    #     'D': {"heuristic": 8},
    #     'E': {"heuristic": 3},
    #     'F': {"heuristic": 10},
    #     }

    G = build_graph(Edges)
    
    nx.set_node_attributes(G, Heuristics)
    # print(G.nodes['A']["heuristic"])
    
    start = 'A'
    goal = 'G'
    rev = 0
    
    print(f"start: {start}\ngoal: {goal}\n")

    goal_path = a_star(start, goal, rev)
    
    if goal_path:
        print(f"Goal path: {goal_path[0]}")
        print(f"Cost: {goal_path[1]}")
    else:
        print("Goal was not found.")
    
    show_graph(G)