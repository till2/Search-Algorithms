from matplotlib import pyplot as plt
import networkx as nx

"""
Node attributes: in_goal_path, seen

Edge attributes: cost

"""


# node class with parent for tracking the goal path

class Node:
    def __init__(self, state, parent_node=None):
        self.state = state
        self.parent = parent_node

# plot/ graph functions

def show_plt(block,t):
    plt.show(block=block) # block==True means the window stays open until manually closed
    plt.pause(t) # if block==False -> show window for t seconds
    plt.close()

def build_graph():
    G = nx.DiGraph() # Directed Graph
    G.add_edges_from([
        ('A', 'B', {"cost":1}),
        ('B', 'D', {"cost":1}),
        ('B', 'C', {"cost":1}),
        ('C', 'E', {"cost":1}),
        ('D', 'F', {"cost":1})
    ])
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
    nx.draw(G, pos, with_labels=True, node_size=600,
            alpha=0.9, node_color=node_colors, edge_color='c', arrowsize=20)
    edge_labels = dict([((u, v), f'{G.get_edge_data(u, v)["cost"]}')
                    for u, v in G.edges])
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    show_plt(block,t)

# search algorithm functions

def expand_node(node):
    neighbors = []
    for neighbor in G.neighbors(node.state):
        neighbor_node = Node(neighbor, node)
        neighbors.append(neighbor_node)
    
    # add neighbors ordered alphabetically
    neighbors_dict = {node.state:node for node in neighbors}
    sorted_neighbors = [elem[1] for elem in
                        sorted(neighbors_dict.items(),key=lambda element: element[0])]
    print(f'expand({node.state}): {[neighbor.state for neighbor in sorted_neighbors]}')
    return sorted_neighbors

def remove_node(Frontier):
    node = Frontier[0]
    del Frontier[0]
    return node

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

def bfs(start, goal):
    # SearchGraph = nx.DiGraph()
    start_node = Node(start)
    Frontier = [start_node]
    while(len(Frontier)):
        current = remove_node(Frontier)
        print('current:   ', current.state)

        # edge_cost = G.get_edge_data(current.parent.state, current.state)["cost"]
      
        # check if current is a goal state
        if(check_goal(current, goal)):
            print(f'goal found: {current.state}')
            return build_goal_path(current)

        # expand frontier
        Frontier += expand_node(current)
        print('frontier: ', [node.state for node in Frontier], '\n')
        
    # no solution
    return None


if __name__ == '__main__':
    G = build_graph()
    
    start = 'A'
    goal = 'E'
    
    print(f'start: {start}\ngoal: {goal}\n')
    
    goal_path = bfs(start, goal)
    
    if goal_path:
        print(f'Goal path: {goal_path}')
    else:
        print('Goal was not found.')
    
    show_graph(G)