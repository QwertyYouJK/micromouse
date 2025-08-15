import cv2
import numpy as np
import matplotlib.pyplot as plt

### CLASS ###
class Node:
    def __init__(self, node_id, x, y):
        self.id = node_id
        self.x = x
        self.y = y
    
    def get_point(self):
        return (self.x,self.y)
    
    def get_ID(self):
        return self.id

class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = {}

    def add_node(self, node_id, x, y):
        if node_id not in self.nodes:
            self.nodes[node_id] = Node(node_id, x, y)
            self.edges[node_id] = {}

    def add_edge(self, node_id1, node_id2, weight):
        self.edges[node_id1][node_id2] = weight
        self.edges[node_id2][node_id1] = weight
        
    def remove_edge(self, node_id1, node_id2):
        self.edges[node_id1].pop(node_id2)
        self.edges[node_id2].pop(node_id1)
    
    def get_nodes(self):
        nodes = []
        for key, val in self.nodes.items():
            nodes.append(Node(key))
        return nodes
    
    def get_edge_weight(self, node_id1, node_id2):
        return self.edges[node_id1][node_id2]

### FUNCTIONS ###
def path_clear(image, x1, y1, x2, y2):
    temp = image.copy()
    cv2.line(temp, (x1, y1), (x2, y2), (255, 255, 255), 1)
    difference = cv2.bitwise_xor(temp, image)
    return np.count_nonzero(difference) == 0


def draw_nodes_and_edges(image, graph):
    for node1, nbrs in graph.edges.items():
        for node2 in nbrs:
            (x1,y1) = graph.nodes[node1].get_point()
            (x2,y2) = graph.nodes[node2].get_point()
            cv2.circle(image, (x1, y1), 3, (0, 255, 0), -1) # pure green
            cv2.line(image, (x1, y1), (x2, y2), (0, 125, 0), 1) # light green
#             cv2.putText(image, str(node1), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            

def draw_blue_path(image, graph, path):
    # Draw path
    for i, node in enumerate(path):
        if i == (len(path) - 1):
            break

        next_node = path[i + 1]
        (x1,y1) = graph.nodes[node].get_point()
        (x2,y2) = graph.nodes[next_node].get_point()
        cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 3)


def bfs(graph, start_node_id, end_node_id):
    q = []
    cost = [-1] * len(graph.nodes)
    parent = [-1] * len(graph.nodes)
    
    # insert start node as visited
    q.append(start_node_id)
    cost[start_node_id] = 0
    
    # run search
    while len(q) != 0:
        x = q.pop(0)
        
        if x == end_node_id:
            break
        
        for nbr in graph.edges[x]:
            if cost[nbr] == -1:
                cost[nbr] = cost[x] + 1
                q.append(nbr)
                parent[nbr] = x
    
    # check if end is reachable
    if cost[end_node_id] == -1:
        print(f"No path between node {start_node_id} and node {end_node_id}")
        return
    
    # obtain path
    path = []
    i = end_node_id
    while i != -1:
        path.append(i)
        i = parent[i]

    path.reverse()
    return path


### CODE ###

IMAGE_FILE = "MicromouseMazeCamera.jpg"
img = cv2.imread(IMAGE_FILE)

width = img.shape[1] // 4
height = img.shape[0] // 4
img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

# Crop the image
img = img[60:492, 351:770]

HSV_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower = np.array([0, 0, 160], np.uint8)
upper = np.array([179, 255, 255], np.uint8)
maze_mask = cv2.inRange(HSV_img, lower, upper)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
maze_img = cv2.morphologyEx(maze_mask, cv2.MORPH_OPEN, kernel)

cv2.imwrite('maze.jpg', maze_img)

# Define ranges
x_start, x_end = 36, 385
y_start, y_end = 30, 390
sections = 9

# Generate equally spaced coordinates
x_coords = np.round(np.linspace(x_start, x_end, sections)).astype(int)
y_coords = np.round(np.linspace(y_start, y_end, sections)).astype(int)

# Create all combinations
bfs_pos = []
for y in y_coords:
    for x in x_coords:
        bfs_pos.append((x, y))

# graph creation
bfs_n = 9
bfs_image = cv2.cvtColor(maze_img, cv2.COLOR_GRAY2BGR)
bfs_graph = Graph()

# add nodes into bfs_graph
for i, (x, y) in enumerate(bfs_pos):
    bfs_graph.add_node(i, x, y)
    

# add edges into bfs_graph
for i, (x, y) in enumerate(bfs_pos):
    # check right node
    next_node = i + 1
    if (i % bfs_n != (bfs_n - 1)):
        (x2, y2) = bfs_pos[next_node] # node on the right
        is_clear = path_clear(maze_img, x, y, x2, y2)
        if is_clear is True:
            bfs_graph.add_edge(i, i + 1, 1)
    
    # check down node
    next_node = i + bfs_n
    if next_node > (bfs_n * bfs_n - 1):
        continue
    (x2, y2) = bfs_pos[next_node] # node directly below
    is_clear = path_clear(maze_img, x, y, x2, y2)
    if is_clear is True:
        bfs_graph.add_edge(i, i + bfs_n, 1)

# Display the image
draw_nodes_and_edges(img, bfs_graph)

# Label first and last positions
x_first, y_first = bfs_pos[0]
x_last, y_last = bfs_pos[-1]
last_num = bfs_n * bfs_n - 1
# cv2.putText(img, '0', (x_first, y_first), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
# cv2.putText(img, str(last_num), (x_last, y_last), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# Breadth first search algo
bfs_start_node = 74 # turn this into x,y cell? node 0 is (0,0) node 80 is (8,8)
bfs_end_node = 77
path = bfs(bfs_graph,bfs_start_node,bfs_end_node)

# Display the resulting image
# print(f"Path: {path}")
# draw_blue_path(img, bfs_graph, path)
# cv2.imshow('bfs_image', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

seq = ''
left = False
right = False
for i in range(0, len(path)):
    node = path[i]
    next_node = path[i+1]
    diff = next_node - node
    if diff == -9 and right is True:
        seq.append('lf')
        right = False
    elif diff == -9 and left is True:
        seq.append('rf')
        left = False
    elif diff == -9:
        seq.append('f')
    
    if diff == 1 and right is True:
        seq.append('f')
    elif diff == 1:
        seq.append('rf')

    if diff == -1 and left is True:
        seq.append('f')
    elif diff == -1:
        seq.append('lf')

print(seq)
    


# start
# if number -9, then move forward
    # turn right flag on? turn left and move forward (turn off right flag)
    # turn left flag on? turn right and move forward (turn off left flag)

# if number +1, turn right and move forward (turn right flag on)
    # turn right flag on? just move foward

# if number -1, turn left and move forward (turn left flag on)
    # turn left flag on? just move forward