import pygame
import random
import math
import heapq
from typing import List, Tuple, Set
import sys

# Initialize pygame
pygame.init()

# Constants
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
BACKGROUND_COLOR = (255, 255, 255)
ROBOT_COLOR = (0, 255, 0)
GOAL_COLOR = (255, 0, 0)
OBSTACLE_COLOR = (100, 100, 100)
OBSTACLE_BORDER = (0, 0, 0)
PATH_COLOR = (255, 0, 255)
VISIBILITY_EDGE_COLOR = (200, 200, 200)
NODE_COLOR = (0, 0, 255)
NODE_RADIUS = 3

class Point:
    """Helper class for 2D points"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def distance_to(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))

class Robot:
    """Mobile robot class with coordinates"""
    def __init__(self, x, y):
        self.position = Point(x, y)
    
    def get_position(self):
        return self.position

class Obstacle:
    """Obstacle defined by list of points (polygon)"""
    def __init__(self, points: List[Point]):
        self.points = points
        self.edges = []
        # Create edges from points
        for i in range(len(points)):
            self.edges.append((points[i], points[(i + 1) % len(points)]))
    
    def contains_point(self, point: Point) -> bool:
        """Check if point is inside the polygon using ray casting algorithm"""
        inside = False
        n = len(self.points)
        for i in range(n):
            p1 = self.points[i]
            p2 = self.points[(i + 1) % n]
            if ((p1.y > point.y) != (p2.y > point.y)) and \
               (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x):
                inside = not inside
        return inside
    
    def draw(self, screen):
        """Draw obstacle on screen"""
        if len(self.points) < 3:
            return
        points = [(p.x, p.y) for p in self.points]
        pygame.draw.polygon(screen, OBSTACLE_COLOR, points)
        pygame.draw.polygon(screen, OBSTACLE_BORDER, points, 2)

class Node:
    """Node for visibility graph with incoming and outgoing edges"""
    def __init__(self, point: Point, node_type="vertex"):
        self.point = point
        self.node_type = node_type  # "start", "goal", or "vertex"
        self.incoming_edges = []  # List of nodes that have edge to this node
        self.outgoing_edges = []  # List of nodes this node has edge to
        self.cost = float('inf')
        self.previous = None
    
    def add_edge(self, target_node, cost):
        """Add outgoing edge to target node"""
        self.outgoing_edges.append((target_node, cost))
        target_node.incoming_edges.append((self, cost))
    
    def __lt__(self, other):
        return self.cost < other.cost

class Graph:
    """Visibility graph containing nodes and edges"""
    def __init__(self):
        self.nodes = []
        self.start_node = None
        self.goal_node = None
    
    def add_node(self, node: Node):
        self.nodes.append(node)
    
    def find_shortest_path(self) -> List[Node]:
        """Find shortest path using Dijkstra's algorithm"""
        if not self.start_node or not self.goal_node:
            return []
        
        # Reset costs and previous nodes
        for node in self.nodes:
            node.cost = float('inf')
            node.previous = None
        
        self.start_node.cost = 0
        priority_queue = [(0, self.start_node)]
        
        while priority_queue:
            current_cost, current_node = heapq.heappop(priority_queue)
            
            if current_node == self.goal_node:
                break
            
            if current_cost > current_node.cost:
                continue
            
            for neighbor, edge_cost in current_node.outgoing_edges:
                new_cost = current_node.cost + edge_cost
                if new_cost < neighbor.cost:
                    neighbor.cost = new_cost
                    neighbor.previous = current_node
                    heapq.heappush(priority_queue, (new_cost, neighbor))
        
        # Reconstruct path
        path = []
        current = self.goal_node
        while current:
            path.append(current)
            current = current.previous
        path.reverse()
        
        return path if path and path[0] == self.start_node else []
    
    def draw(self, screen, show_edges=True):
        """Draw graph nodes and edges"""
        if show_edges:
            for node in self.nodes:
                for neighbor, _ in node.outgoing_edges:
                    pygame.draw.line(screen, VISIBILITY_EDGE_COLOR, 
                                   (node.point.x, node.point.y), 
                                   (neighbor.point.x, neighbor.point.y), 1)
        
        for node in self.nodes:
            color = NODE_COLOR
            if node == self.start_node:
                color = ROBOT_COLOR
            elif node == self.goal_node:
                color = GOAL_COLOR
            pygame.draw.circle(screen, color, 
                             (int(node.point.x), int(node.point.y)), 
                             NODE_RADIUS)

def point_on_segment(p: Point, a: Point, b: Point) -> bool:
    """Check if point p lies on segment ab"""
    cross = (p.y - a.y) * (b.x - a.x) - (p.x - a.x) * (b.y - a.y)
    if abs(cross) > 1e-9:
        return False
    dot = (p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)
    if dot < 0:
        return False
    squared_len = (b.x - a.x)**2 + (b.y - a.y)**2
    if dot > squared_len:
        return False
    return True

def segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool:
    """Check if two segments intersect"""
    def orientation(p, q, r):
        val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
        if abs(val) < 1e-9:
            return 0
        return 1 if val > 0 else 2
    
    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)
    
    if o1 != o2 and o3 != o4:
        return True
    
    if o1 == 0 and point_on_segment(p3, p1, p2):
        return True
    if o2 == 0 and point_on_segment(p4, p1, p2):
        return True
    if o3 == 0 and point_on_segment(p1, p3, p4):
        return True
    if o4 == 0 and point_on_segment(p2, p3, p4):
        return True
    
    return False

def segment_intersects_obstacle(p1: Point, p2: Point, obstacles: List[Obstacle]) -> bool:
    """Check if segment intersects any obstacle"""
    for obstacle in obstacles:
        for edge in obstacle.edges:
            if segments_intersect(p1, p2, edge[0], edge[1]):
                # Check if intersection is at endpoints
                if (p1 == edge[0] or p1 == edge[1] or 
                    p2 == edge[0] or p2 == edge[1]):
                    continue
                return True
    return False

def is_visible_from_vertex(point: Point, vertex: Point, obstacles: List[Obstacle], 
                          obstacle_points: Set[Point]) -> bool:
    """Check if point is visible from vertex"""
    if point == vertex:
        return False
    
    # Check if line passes through any obstacle
    if segment_intersects_obstacle(point, vertex, obstacles):
        return False
    
    return True

def generate_random_polygon(x_min, x_max, y_min, y_max, num_vertices=5):
    """Generate random convex polygon"""
    points = []
    center_x = random.uniform(x_min, x_max)
    center_y = random.uniform(y_min, y_max)
    radius = random.uniform(30, 60)
    
    angles = sorted([random.uniform(0, 2 * math.pi) for _ in range(num_vertices)])
    
    for angle in angles:
        r = radius * random.uniform(0.8, 1.2)
        x = center_x + r * math.cos(angle)
        y = center_y + r * math.sin(angle)
        points.append(Point(x, y))
    
    return points

def build_visibility_graph(start: Point, goal: Point, obstacles: List[Obstacle]) -> Graph:
    """Build visibility graph from start and goal points and obstacles"""
    graph = Graph()
    
    # Collect all vertices
    vertices = [start, goal]
    obstacle_points = set()
    
    for obstacle in obstacles:
        for point in obstacle.points:
            vertices.append(point)
            obstacle_points.add(point)
    
    # Create nodes for all vertices
    nodes_dict = {}
    for vertex in vertices:
        node_type = "vertex"
        if vertex == start:
            node_type = "start"
        elif vertex == goal:
            node_type = "goal"
        node = Node(vertex, node_type)
        nodes_dict[vertex] = node
        graph.add_node(node)
        
        if node_type == "start":
            graph.start_node = node
        elif node_type == "goal":
            graph.goal_node = node
    
    # Check visibility between all pairs of vertices
    for i, v1 in enumerate(vertices):
        for j, v2 in enumerate(vertices):
            if i >= j:
                continue
            
            # Check if segment is visible
            visible = True
            
            # Check if line goes through obstacles
            if segment_intersects_obstacle(v1, v2, obstacles):
                visible = False
            
            # Check if either endpoint is inside an obstacle
            for obstacle in obstacles:
                if (obstacle.contains_point(v1) or obstacle.contains_point(v2)):
                    visible = False
                    break
            
            if visible:
                cost = v1.distance_to(v2)
                nodes_dict[v1].add_edge(nodes_dict[v2], cost)
                nodes_dict[v2].add_edge(nodes_dict[v1], cost)
    
    return graph

def generate_obstacles(num_obstacles=5):
    """Generate random obstacles"""
    obstacles = []
    min_distance_from_start_goal = 50
    start = Point(100, 100)
    goal = Point(500, 500)
    
    for _ in range(num_obstacles):
        while True:
            # Generate polygon in valid area
            x_min = random.uniform(50, WINDOW_WIDTH - 50)
            x_max = x_min + random.uniform(50, 100)
            y_min = random.uniform(50, WINDOW_HEIGHT - 50)
            y_max = y_min + random.uniform(50, 100)
            
            # Adjust to keep within bounds
            x_min = max(50, min(x_min, WINDOW_WIDTH - 100))
            x_max = min(WINDOW_WIDTH - 50, max(x_max, x_min + 50))
            y_min = max(50, min(y_min, WINDOW_HEIGHT - 100))
            y_max = min(WINDOW_HEIGHT - 50, max(y_max, y_min + 50))
            
            points = generate_random_polygon(x_min, x_max, y_min, y_max)
            obstacle = Obstacle(points)
            
            # Check if obstacle doesn't contain start or goal
            if not obstacle.contains_point(start) and not obstacle.contains_point(goal):
                # Check if obstacle doesn't overlap too much with others
                overlap = False
                for existing in obstacles:
                    # Simple overlap check - check if any point is inside existing obstacle
                    for point in points:
                        if existing.contains_point(point):
                            overlap = True
                            break
                    if overlap:
                        break
                
                if not overlap:
                    obstacles.append(obstacle)
                    break
    
    return obstacles

def draw_robot(screen, robot: Robot):
    """Draw robot"""
    pos = robot.get_position()
    pygame.draw.circle(screen, ROBOT_COLOR, (int(pos.x), int(pos.y)), 8)
    pygame.draw.circle(screen, (0, 0, 0), (int(pos.x), int(pos.y)), 8, 2)

def draw_goal(screen, goal: Point):
    """Draw goal point"""
    pygame.draw.circle(screen, GOAL_COLOR, (int(goal.x), int(goal.y)), 10)
    pygame.draw.circle(screen, (0, 0, 0), (int(goal.x), int(goal.y)), 10, 2)

def draw_path(screen, path: List[Node]):
    """Draw the found path"""
    if not path:
        return
    
    points = [(node.point.x, node.point.y) for node in path]
    if len(points) > 1:
        pygame.draw.lines(screen, PATH_COLOR, False, points, 3)

def main():
    """Main function"""
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Robot Path Planning - Visibility Graph")
    clock = pygame.time.Clock()
    
    # Initialize start and goal
    start = Point(100, 100)
    goal = Point(500, 500)
    robot = Robot(start.x, start.y)
    
    # Generate obstacles
    obstacles = generate_obstacles(5)
    
    # Build visibility graph
    print("Building visibility graph...")
    graph = build_visibility_graph(start, goal, obstacles)
    print(f"Graph built with {len(graph.nodes)} nodes")
    
    # Find shortest path
    print("Finding shortest path...")
    path = graph.find_shortest_path()
    if path:
        print(f"Path found with {len(path)} waypoints")
        total_distance = 0
        for i in range(len(path) - 1):
            total_distance += path[i].point.distance_to(path[i+1].point)
        print(f"Total path distance: {total_distance:.2f} pixels")
    else:
        print("No path found!")
    
    # Main loop
    running = True
    show_edges = False
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    show_edges = not show_edges
                    print(f"Visibility edges: {'ON' if show_edges else 'OFF'}")
                elif event.key == pygame.K_r:
                    # Regenerate obstacles and path
                    obstacles = generate_obstacles(5)
                    graph = build_visibility_graph(start, goal, obstacles)
                    path = graph.find_shortest_path()
                    if path:
                        print("New path found!")
                    else:
                        print("No path found with new obstacles!")
        
        # Clear screen
        screen.fill(BACKGROUND_COLOR)
        
        # Draw obstacles
        for obstacle in obstacles:
            obstacle.draw(screen)
        
        # Draw visibility graph
        graph.draw(screen, show_edges)
        
        # Draw start and goal
        draw_robot(screen, robot)
        draw_goal(screen, goal)
        
        # Draw path
        draw_path(screen, path)
        
        # Update display
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()