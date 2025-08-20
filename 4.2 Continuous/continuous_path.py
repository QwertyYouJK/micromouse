import cv2
from pathlib import Path
import numpy as np
from collections import deque

#======================== CLASSES ===========================#
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

    def remove_node(self, node_id):
        self.nodes.pop(node_id)

    def add_edge(self, node_id1, node_id2, weight):
        self.edges[node_id1][node_id2] = weight
        self.edges[node_id2][node_id1] = weight

    def remove_edge(self, node_id1, node_id2):
        self.edges[node_id1].pop(node_id2)
        self.edges[node_id2].pop(node_id1)
    
    def get_nodes(self):
        return list(self.nodes.values())
    
    def get_edge_weight(self, node_id1, node_id2):
        return self.edges[node_id1][node_id2]

#======================== FUNCTIONS ===========================#
def path_clear(image, x1, y1, x2, y2):
    temp = image.copy()
    cv2.line(temp, (x1, y1), (x2, y2), (255, 255, 255), 1)
    difference = cv2.bitwise_xor(temp, image)
    return np.count_nonzero(difference) == 0


def draw_nodes_and_edges(image, graph):
    for node_id, node in graph.nodes.items():
        x, y = node.get_point()
        cv2.circle(image, (int(x), int(y)), 3, (0, 255, 0), -1)

    for node1, nbrs in graph.edges.items():
        for node2 in nbrs:
            (x1,y1) = graph.nodes[node1].get_point()
            (x2,y2) = graph.nodes[node2].get_point()
            #cv2.circle(image, (x1, y1), 3, (0, 255, 0), -1) # pure green
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


def diff_to_h(diff):
    if diff == -9:
        return 0
    elif diff == 1:
        return 1
    elif diff == 9:
        return 2
    elif diff == -1:
        return 3

def in_roi(node_id, n, row, col, size):
    r, c = divmod(node_id, n)
    return (row <= r < row + size) and (col <= c < col + size)

#========================= CODE =========================#
img_path = Path(__file__).parent / "RealMaze.jpg"   # same folder as script
img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
if img is None:
    raise IOError(f"OpenCV could not read the image: {img_path.resolve()}")

width = img.shape[1] // 4
height = img.shape[0] // 4
img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

# Crop the image
img = img[88:459, 349:714]

HSV_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower = np.array([0, 0, 130], np.uint8)
upper = np.array([142, 40, 255], np.uint8)
maze_mask = cv2.inRange(HSV_img, lower, upper)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
maze_img = cv2.morphologyEx(maze_mask, cv2.MORPH_OPEN, kernel)

# Save the image
script_dir = Path(__file__).parent
output_path = script_dir / "maze.jpg"
cv2.imwrite(str(output_path), maze_img)

# Define ranges
tl = (4, 2) # top left of 5x5 grid
x_start, x_end = 21, 340
y_start, y_end = 23, 350
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
    print(f"added node {i} to the graph, its position is at {x},{y}")
    bfs_graph.add_node(i, x, y)

# add edges into bfs_graph
for i, (x, y) in enumerate(bfs_pos):

    # check right node j = i+1
    j = i + 1
    if i % bfs_n != (bfs_n - 1):
        (x2, y2) = bfs_pos[j] # node on the right
        is_clear = path_clear(maze_img, x, y, x2, y2)

        if is_clear is True and (not in_roi(i, bfs_n, tl[0], tl[1], 5) or not in_roi(j, bfs_n, tl[0], tl[1], 5)):
            bfs_graph.add_edge(i, j, 1)
    
    # check down node
    j = i + bfs_n
    if j > (bfs_n * bfs_n - 1):
        continue
    (x2, y2) = bfs_pos[j] # node directly below
    is_clear = path_clear(maze_img, x, y, x2, y2)

    if is_clear is True and (not in_roi(i, bfs_n, tl[0], tl[1], 5) or not in_roi(j, bfs_n, tl[0], tl[1], 5)):
        bfs_graph.add_edge(i, j, 1)

# Display the image
draw_nodes_and_edges(bfs_image, bfs_graph)
script_dir = Path(__file__).parent
output_path = script_dir / "dots.jpg"
cv2.imwrite(str(output_path), bfs_image)

# Label first and last positions
x_first, y_first = bfs_pos[0]
x_last, y_last = bfs_pos[-1]
last_num = bfs_n * bfs_n - 1
# cv2.putText(img, '0', (x_first, y_first), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
# cv2.putText(img, str(last_num), (x_last, y_last), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# Breadth first search algo
bfs_start = (0,2)
bfs_end = (8,5)

bfs_start_node = bfs_start[0] * bfs_n + bfs_start[1]
bfs_end_node = bfs_end[0] * bfs_n + bfs_end[1]
path = bfs(bfs_graph,bfs_start_node,bfs_end_node)

if path is None:
    print("No path")
    exit()
if len(path) < 2:
    print("No path")
    exit()
# Display the resulting image
print(f"Path: {path}")
draw_blue_path(img, bfs_graph, path)
# Save the image
script_dir = Path(__file__).parent
output_path = script_dir / "path.jpg"
cv2.imwrite(str(output_path), img)
# cv2.imshow('bfs_image', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

seq = []
left = False
right = False

prev_h = diff_to_h(path[1] - path[0])  # 0 = north, 1 = east, 2 = south, 3 = west
curr_h = 0 
seq.append('fd')
for i in range(1, len(path) - 1):
    node = path[i]
    next_node = path[i+1]
    diff = next_node - node
    curr_h = diff_to_h(diff)
    if (curr_h - prev_h) == 0:
        seq.append('fd')
    elif (curr_h - prev_h) == 1 or (curr_h - prev_h) == -3:
        seq.append('rfd')
    elif (curr_h - prev_h) == -1 or (curr_h - prev_h) == 3:
        seq.append('lfd')
    prev_h = curr_h

seq.append('s')
sequence = "".join(seq)
print(sequence)

script_dir = Path(__file__).parent
task4_dir = script_dir.parent / "Task_4"
out_path = task4_dir / "sequence.txt"
with open(out_path, "w") as f:
    f.write(sequence)

print(f"Sequence saved to {out_path.resolve()}")

