import pygame
import random
import math
import sys

# Инициализация Pygame
pygame.init()

# Константы окна
WIDTH, HEIGHT = 800, 600
FPS = 60
BACKGROUND_COLOR = (255, 255, 255)
ROBOT_COLOR = (0, 200, 0)
TARGET_COLOR = (255, 0, 0)
OBSTACLE_COLOR = (100, 100, 100)
TREE_COLOR = (200, 200, 200)
PATH_COLOR = (0, 0, 255)
NODE_RADIUS = 3
STEP_SIZE = 25
GOAL_RADIUS = 10
MAX_ITER = 3000
GOAL_SAMPLE_RATE = 0.05

#  1. Класс узла (Node) 
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None  # для обратного прохода
        self.children = []  # исходящие дуги (для полноты)
        self.incoming = []  # входящие дуги (для полноты)

    def get_pos(self):
        return (self.x, self.y)

    def distance_to(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)

#  2. Класс графа (Graph) 
class Graph:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)
        return node

    def nearest(self, target_node):
        nearest_node = None
        min_dist = float('inf')
        for node in self.nodes:
            dist = node.distance_to(target_node)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

#  3. Класс робота (Robot) 
class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_pos(self):
        return (self.x, self.y)

#  4. Класс препятствия (Obstacle) 
class Obstacle:
    def __init__(self, vertices):
        # vertices: список кортежей (x, y)
        self.vertices = vertices

    def get_edges(self):
        edges = []
        n = len(self.vertices)
        for i in range(n):
            edges.append((self.vertices[i], self.vertices[(i+1) % n]))
        return edges

#  5. Функция проверки пересечения отрезка и отрезка 
def segments_intersect(p1, p2, p3, p4):
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if abs(val) < 1e-9: return 0
        return 1 if val > 0 else 2

    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)

    if o1 != o2 and o3 != o4:
        return True
    return False

#  6. Проверка пересечения отрезка и многоугольника 
def segment_intersects_polygon(p1, p2, obstacle):
    edges = obstacle.get_edges()
    for edge in edges:
        if segments_intersect(p1, p2, edge[0], edge[1]):
            return True
    return False

#  7. Функция построения ветви RRT 
def rrt_extend(graph, target_point, obstacles, step_size=STEP_SIZE):
    # target_point - (x, y) случайной точки
    random_node = Node(target_point[0], target_point[1])
    nearest = graph.nearest(random_node)

    # Направление и новый узел
    dx = random_node.x - nearest.x
    dy = random_node.y - nearest.y
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return None

    # Шаг к случайной точке
    new_x = nearest.x + (dx / dist) * min(step_size, dist)
    new_y = nearest.y + (dy / dist) * min(step_size, dist)
    new_node = Node(new_x, new_y)

    # Проверка столкновений с препятствиями
    collision = False
    for obs in obstacles:
        if segment_intersects_polygon(nearest.get_pos(), new_node.get_pos(), obs):
            collision = True
            break

    if not collision:
        graph.add_node(new_node)
        new_node.parent = nearest
        nearest.children.append(new_node)
        new_node.incoming.append(nearest)
        return new_node
    return None

#  8. Формирование маршрута обратным проходом 
def get_path(goal_node):
    path = []
    node = goal_node
    while node is not None:
        path.append(node.get_pos())
        node = node.parent
    path.reverse()
    return path

#  9. Генерация случайных препятствий-многоугольников 
def generate_random_polygon(center_x, center_y, avg_radius=40, num_vertices=5):
    angles = sorted([random.uniform(0, 2*math.pi) for _ in range(num_vertices)])
    vertices = []
    for angle in angles:
        r = random.uniform(avg_radius * 0.6, avg_radius * 1.4)
        x = center_x + r * math.cos(angle)
        y = center_y + r * math.sin(angle)
        vertices.append((x, y))
    return vertices

#  10. Проверка достижения цели 
def is_goal_reached(node, goal_pos, threshold=GOAL_RADIUS):
    return math.hypot(node.x - goal_pos[0], node.y - goal_pos[1]) < threshold

#  ГЛАВНАЯ ФУНКЦИЯ 
def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("RRT Path Planning for Mobile Robot")
    clock = pygame.time.Clock()

    # Параметры входа
    start_pos = (100, 100)
    goal_pos = (500, 500)

    # Генерация 5 случайных препятствий
    obstacles = []
    for _ in range(5):
        center = (random.randint(150, 650), random.randint(150, 500))
        verts = generate_random_polygon(center[0], center[1], random.randint(35, 60), random.randint(4, 7))
        obstacles.append(Obstacle(verts))

    # Инициализация графа и робота
    robot = Robot(start_pos[0], start_pos[1])
    graph = Graph()
    start_node = Node(start_pos[0], start_pos[1])
    graph.add_node(start_node)

    goal_node = None
    running = True
    iterations = 0

    # Основной цикл RRT
    while running and iterations < MAX_ITER:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()

        # Выбор случайной точки (иногда выбираем цель)
        if random.random() < GOAL_SAMPLE_RATE:
            rand_point = goal_pos
        else:
            rand_point = (random.uniform(0, WIDTH), random.uniform(0, HEIGHT))

        new_node = rrt_extend(graph, rand_point, obstacles, STEP_SIZE)

        if new_node is not None and is_goal_reached(new_node, goal_pos, GOAL_RADIUS):
            goal_node = new_node
            print("Goal reached!")
            break

        iterations += 1

        # Отрисовка
        screen.fill(BACKGROUND_COLOR)

        # Рисуем препятствия
        for obs in obstacles:
            pygame.draw.polygon(screen, OBSTACLE_COLOR, obs.vertices, 0)
            pygame.draw.polygon(screen, (0, 0, 0), obs.vertices, 2)

        # Рисуем дерево RRT (рёбра)
        for node in graph.nodes:
            if node.parent is not None:
                pygame.draw.line(screen, TREE_COLOR, node.get_pos(), node.parent.get_pos(), 2)
            pygame.draw.circle(screen, (0, 0, 0), node.get_pos(), NODE_RADIUS)

        # Рисуем робота (начальную точку)
        pygame.draw.circle(screen, ROBOT_COLOR, start_pos, 8)
        # Целевую точку
        pygame.draw.circle(screen, TARGET_COLOR, goal_pos, GOAL_RADIUS)

        pygame.display.flip()
        clock.tick(FPS)

    # Если найден путь — рисуем его
    if goal_node is not None:
        path = get_path(goal_node)
        # Утолщённая линия пути
        for i in range(len(path) - 1):
            pygame.draw.line(screen, PATH_COLOR, path[i], path[i+1], 4)
        print("Path found and displayed")
    else:
        print("Goal not reached within iterations")

    # Ожидание закрытия окна после построения
    waiting = True
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                waiting = False
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()