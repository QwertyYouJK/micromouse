import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
from pathlib import Path
import numpy as np
import serial, time
#======================== VARIABLES ===========================#
# Top left and bottom right of the maze to crop the image
cropX1, cropX2 = (361, 767)
cropY1, cropY2 = (69, 475)
# cropX1, cropX2 = (349, 714)
# cropY1, cropY2 = (88, 459)

# (0,0) and (8,8) position, evenly spaced nodes will be placed
x0, x8 = 25, 377
y0, y8 = 25, 381
graph_n = 9
# x0, x8 = 21, 340
# y0, y8 = 23, 350

# start and end cell (0,0) ~ (8,8)
start = (1,1)
end = (4,4)

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

#======================== FUNCTIONS ===========================#
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
        return {}
    
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

#========================= CODE =========================#

# cam = cv2.VideoCapture(0) # use 0 if your device has no webcam
# ret, img = cam.read()
# cam.release() # release the camera

# if not ret:
#   print("error: couldn't take photo")
#   exit(1)

# # save image
# script_dir = Path(__file__).parent
# output_path = script_dir / "maze_img.jpg"
# cv2.imwrite(str(output_path), img)

img_path = Path(__file__).parent / "MicromouseMazeCamera.jpg"   # same folder as script
img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
if img is None:
    raise IOError(f"OpenCV could not read the image: {img_path.resolve()}")

width = img.shape[1] // 4
height = img.shape[0] // 4
img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

# Crop the image
img = img[cropY1:cropY2, cropX1:cropX2]

HSV_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower = np.array([0, 0, 160], np.uint8)
upper = np.array([179, 255, 255], np.uint8)
maze_mask = cv2.inRange(HSV_img, lower, upper)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
maze_img = cv2.morphologyEx(maze_mask, cv2.MORPH_OPEN, kernel)

# Save the image
script_dir = Path(__file__).parent
output_path = script_dir / "maze.jpg"
cv2.imwrite(str(output_path), maze_img)

# Generate equally spaced coordinates
x_coords = np.round(np.linspace(x0, x8, graph_n)).astype(int)
y_coords = np.round(np.linspace(y0, y8, graph_n)).astype(int)

# Create all combinations
node_pos = []
for y in y_coords:
    for x in x_coords:
        node_pos.append((x, y))

# graph creation
graph = Graph()

# add nodes into graph
for i, (x, y) in enumerate(node_pos):
    graph.add_node(i, x, y)
    

# add edges into graph
for i, (x, y) in enumerate(node_pos):
    # check right node
    next_node = i + 1
    if (i % graph_n != (graph_n - 1)):
        (x2, y2) = node_pos[next_node] # node on the right
        is_clear = path_clear(maze_img, x, y, x2, y2)
        if is_clear is True:
            graph.add_edge(i, i + 1, 1)
    
    # check down node
    next_node = i + graph_n
    if next_node > (graph_n * graph_n - 1):
        continue
    (x2, y2) = node_pos[next_node] # node directly below
    is_clear = path_clear(maze_img, x, y, x2, y2)
    if is_clear is True:
        graph.add_edge(i, i + graph_n, 1)

# Display the image
draw_nodes_and_edges(img, graph)
script_dir = Path(__file__).parent
output_path = script_dir / "dots.jpg"
cv2.imwrite(str(output_path), img)

# Label first and last positions
x_first, y_first = node_pos[0]
x_last, y_last = node_pos[-1]
last_num = graph_n * graph_n - 1
# cv2.putText(img, '0', (x_first, y_first), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
# cv2.putText(img, str(last_num), (x_last, y_last), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# Breadth first search algo
start_node = start[0] * graph_n + start[1]
end_node = end[0] * graph_n + end[1]
path = bfs(graph,start_node,end_node)

if len(path) < 2:
    print("No path")
    exit()
# Display the resulting image
print(f"Path: {path}")
draw_blue_path(img, graph, path)
script_dir = Path(__file__).parent
output_path = script_dir / "path.jpg"
cv2.imwrite(str(output_path), img)

seq = []
left = False
right = False

prev_h = diff_to_h(path[1] - path[0])  # 0 = north, 1 = east, 2 = south, 3 = west
curr_h = 0 
seq.append('f')
for i in range(1, len(path) - 1):
    node = path[i]
    next_node = path[i+1]
    diff = next_node - node
    curr_h = diff_to_h(diff)
    if (curr_h - prev_h) == 0:
        seq.append('f')
    elif (curr_h - prev_h) == 1 or (curr_h - prev_h) == -3:
        seq.append('rf')
    elif (curr_h - prev_h) == -1 or (curr_h - prev_h) == 3:
        seq.append('lf')
    prev_h = curr_h

seq.append('s')
sequence = "".join(seq)
print(sequence)

#flflffrffrflflfrfrflfrffrflfrfflflfrfrfffrflfrfs