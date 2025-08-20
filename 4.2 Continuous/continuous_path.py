import cv2
from pathlib import Path
import random
import math
import numpy as np

#======================== VARIABLES ===========================#
# Top left and bottom right of the maze to crop the image
cropX1, cropX2 = (361, 767)
cropY1, cropY2 = (69, 475)
# cropX1, cropX2 = (349, 714)
# cropY1, cropY2 = (88, 459)

# occupancy map vars
unsafe_kernel_size = 7
unsafe_iterations = 3

# Define 5x5 grid 
tl = (2, 2) # top left of 5x5 grid
br = (6, 6) # bottom right of 5x5 grid

# (0,0) and (8,8) position, evenly spaced nodes will be placed
x_start, x_end = 25, 377
y_start, y_end = 25, 381
graph_n = 9
# x_start, x_end = 21, 340
# y_start, y_end = 23, 350

# Graph variables
prm_total_nodes_count = 69
prm_connection_radius = 100

# start and end cell (0,0) ~ (8,8)
start = (4,8)
end = (5,8)

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
        cv2.putText(image, str(node_id), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

    for node1, nbrs in graph.edges.items():
        for node2 in nbrs:
            (x1,y1) = graph.nodes[node1].get_point()
            (x2,y2) = graph.nodes[node2].get_point()
            cv2.line(image, (x1, y1), (x2, y2), (0, 125, 0), 1) # light green
            
def draw_blue_path(image, graph, path):
    # Draw path
    for i, node in enumerate(path):
        if i == (len(path) - 1):
            break

        next_node = path[i + 1]
        (x1,y1) = graph.nodes[node].get_point()
        (x2,y2) = graph.nodes[next_node].get_point()
        cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 3)

def dijkstra(graph, start_id, end_id):    
    n = len(graph.nodes)
    p_q = []
    cost = [-1] * n
    parent = [-1] * n
    # Set start_id and end_id as positive (last two indexes of list)
    start = n + start_id
    end = n + end_id
    
    # Insert start node as visited
    p_q.append(start)
    cost[start] = 0
    
    # Run search
    while len(p_q) != 0:
        p_q.sort(key = lambda a : cost[a])
        x = p_q.pop(0)
        
        if x == end_id:
            break
        
        temp = x
        # Change index back to original negative index
        if temp == start:
            temp = -1
        elif temp == end:
            temp = -2
        
        for nbr, w in graph.edges[temp].items():
            
            if nbr == -1:
                nbr = start
            elif nbr == -2:
                nbr = end
            
            new_cost = cost[temp] + w
            if cost[nbr] == -1:
                cost[nbr] = new_cost
                parent[nbr] = x
                p_q.append(nbr)
            elif new_cost < cost[nbr]:
                cost[nbr] = new_cost
                parent[nbr] = x
                p_q.append(nbr)
        
                
    # Check if end is reachable
    if cost[end] == -1:
        print(f"No path between node {start_id} and node {end_id}")
        return (0, 0)
    
    # Obtain path and cost
    total_cost = cost[end]
    path = []
    i = end
    while i != -1:
        temp = i
        if temp == start:
            temp = -1
        elif temp == end:
            temp = -2

        path.append(temp)
        i = parent[i]    

    path.reverse()
    return (path, total_cost)

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
img_path = Path(__file__).parent / "Micromouse_continuous_1.jpg" 
img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
if img is None:
    raise IOError(f"OpenCV could not read the image: {img_path.resolve()}")

width = img.shape[1] // 4
height = img.shape[0] // 4
img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
script_dir = Path(__file__).parent
output_path = script_dir / "shrunk_maze.jpg"
cv2.imwrite(str(output_path), img)

# Crop the image
img = img[cropY1:cropY2, cropX1:cropX2]

# Maze mask
HSV_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower = np.array([0, 0, 130], np.uint8)
upper = np.array([151, 41, 255], np.uint8)
maze_mask = cv2.inRange(HSV_img, lower, upper)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
maze_img = cv2.morphologyEx(maze_mask, cv2.MORPH_OPEN, kernel)

# Save the image
script_dir = Path(__file__).parent
output_path = script_dir / "maze.jpg"
cv2.imwrite(str(output_path), maze_img)

# occupancy map
kernel = np.ones((unsafe_kernel_size, unsafe_kernel_size), np.uint8)
erosion = cv2.erode(maze_img, kernel, iterations = unsafe_iterations)

edges = cv2.bitwise_xor(maze_img, erosion)
base_bgr = cv2.cvtColor(erosion, cv2.COLOR_GRAY2BGR)

red_overlay = np.zeros_like(base_bgr)
red_overlay[edges > 0] = (0, 0, 255)

occ_map = cv2.bitwise_or(base_bgr, red_overlay)

# Save resulting image
script_dir = Path(__file__).parent
output_path = script_dir / "occupancy.jpg"
cv2.imwrite(str(output_path), occ_map)

# Generate equally spaced nodes
x_coords = np.round(np.linspace(x_start, x_end, graph_n)).astype(int)
y_coords = np.round(np.linspace(y_start, y_end, graph_n)).astype(int)

# Create all combinations
out_node_pos = []
for y in y_coords:
    for x in x_coords:
        out_node_pos.append((x, y))

# graph creation
graph = Graph()

# add outside grid nodes into graph
for i, (x, y) in enumerate(out_node_pos):
    # print(f"added node {i} to the graph, its position is at {x},{y}")
    graph.add_node(i, x, y)

# add edges into graph
for i, (x, y) in enumerate(out_node_pos):
    # check right node j = i+1
    j = i + 1
    if i % graph_n != (graph_n - 1):
        (x2, y2) = out_node_pos[j] # node on the right
        is_clear = path_clear(maze_img, x, y, x2, y2)

        if is_clear is True and (not in_roi(i, graph_n, tl[0], tl[1], 5) or not in_roi(j, graph_n, tl[0], tl[1], 5)):
            dist = math.sqrt((x2 - x) ** 2  + (y2 - y) ** 2)
            graph.add_edge(i, j, dist)
    
    # check down node
    j = i + graph_n
    if j > (graph_n * graph_n - 1):
        continue
    (x2, y2) = out_node_pos[j] # node directly below
    is_clear = path_clear(maze_img, x, y, x2, y2)

    if is_clear is True and (not in_roi(i, graph_n, tl[0], tl[1], 5) or not in_roi(j, graph_n, tl[0], tl[1], 5)):
        dist = math.sqrt((x2 - x) ** 2  + (y2 - y) ** 2)
        graph.add_edge(i, j, dist)

# Generate random nodes within 5x5 grid
prm_pos = []
prm_ids = []
tlX, tlY = out_node_pos[tl[0] * graph_n + tl[1]]
brX, brY = out_node_pos[br[0] * graph_n + br[1]]
while len(prm_pos) < prm_total_nodes_count:
    x = random.randint(min(tlX, brX), max(tlX, brX))
    y = random.randint(min(tlY, brY), max(tlY, brY))
    
    # Check if node is in free space and add node to graph
    if path_clear(occ_map, x, y, x, y) is True:
        prm_pos.append((x, y))
        node_id = len(graph.nodes)
        graph.add_node(node_id, x, y)
        prm_ids.append(node_id)

# Add edges in PRM
for a, (x1, y1) in enumerate(prm_pos):
    for b, (x2, y2) in enumerate(prm_pos):
        if a == b:
            continue
        
        # Check if nodes is within radius
        dist = math.sqrt((x2 - x1) ** 2  + (y2 - y1) ** 2)
        if dist > prm_connection_radius:
            continue
            
        # Add edge if  path is clear
        is_clear = path_clear(occ_map, x1, y1, x2, y2)
        if is_clear is True:
            n1 = prm_ids[a]
            n2 = prm_ids[b]
            graph.add_edge(n1, n2, dist)

# Bridge PRM and outside grid
grid_ids = list(range(graph_n * graph_n))

for a, (x1, y1) in enumerate(prm_pos):
    n1 = prm_ids[a]
    for gid in grid_ids:
        x2, y2 = graph.nodes[gid].get_point()
        dist = math.sqrt((x2 - x1) ** 2  + (y2 - y1) ** 2)
        if dist > prm_connection_radius:
            continue
        if path_clear(occ_map, x1, y1, x2, y2):
            graph.add_edge(n1, gid, dist)

# Add start (node id = -1) and end (node_id = -2) nodes
start_x, start_y = out_node_pos[start[0] * graph_n + start[1]]
goal_x, goal_y = out_node_pos[end[0] * graph_n + end[1]]
graph.add_node(-2, goal_x, goal_y) # end node
graph.add_node(-1, start_x, start_y) # start node

# Add edges near start and end nodes
for i, (x1, y1) in enumerate(out_node_pos):
    # Check if nodes is within radius of start node
    dist = math.sqrt((start_x - x1) ** 2  + (start_y - y1) ** 2)
    if dist <= prm_connection_radius:
        # Add edge if path is clear
        is_clear = path_clear(occ_map, x1, y1, start_x, start_y)
        if is_clear is True:
            graph.add_edge(-1, i, dist)
        
    # Check nearby nodes of end node
    dist = math.sqrt((goal_x - x1) ** 2  + (goal_y - y1) ** 2)
    if dist > prm_connection_radius:
        continue
    
    is_clear = path_clear(occ_map, x1, y1, goal_x, goal_y)
    if is_clear is True:
        graph.add_edge(-2, i, dist)


# Dijkstra algo
path, cost = dijkstra(graph, -1, -2)

draw_nodes_and_edges(occ_map, graph)
script_dir = Path(__file__).parent
output_path = script_dir / "dots.jpg"
cv2.imwrite(str(output_path), occ_map)

if path == 0:
    print("No path")
    exit()
if len(path) < 2:
    print("No path")
    exit()
# Display the resulting image
print(f"Path: {path}")
draw_blue_path(img, graph, path)
# Save the image
script_dir = Path(__file__).parent
output_path = script_dir / "path.jpg"
cv2.imwrite(str(output_path), img)

# TODO: redo sequence
'''
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
'''
