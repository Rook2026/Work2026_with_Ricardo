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
ROBOT_COLOR = (0, 200, 0)      # начальная точка
TARGET_COLOR = (255, 0, 0)      # целевая точка
OBSTACLE_COLOR = (100, 100, 100)
TREE_START_COLOR = (0, 200, 0)   # дерево от старта (зелёное)
TREE_GOAL_COLOR = (0, 0, 200)    # дерево от цели (синее)
PATH_COLOR = (255, 255, 0)       # итоговый путь (жёлтый)
CONNECTION_COLOR = (255, 0, 255) # точка встречи (пурпурный)
NODE_RADIUS = 3
STEP_SIZE = 25
MAX_ITER = 2000

# ========== 1. Класс узла (Node) ==========
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.children = []
        self.incoming = []

    def get_pos(self):
        return (self.x, self.y)

    def distance_to(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)

# ========== 2. Класс графа (дерева) ==========
class Graph:
    def __init__(self):
        self.nodes = []
        self.root = None

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

# ========== 3. Класс робота ==========
class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_pos(self):
        return (self.x, self.y)

# ========== 4. Класс препятствия ==========
class Obstacle:
    def __init__(self, vertices):
        self.vertices = vertices

    def get_edges(self):
        edges = []
        n = len(self.vertices)
        for i in range(n):
            edges.append((self.vertices[i], self.vertices[(i+1) % n]))
        return edges

# ========== 5. Проверка пересечения двух отрезков ==========
def segments_intersect(p1, p2, p3, p4):
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if abs(val) < 1e-9:
            return 0
        return 1 if val > 0 else 2

    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)

    if o1 != o2 and o3 != o4:
        return True
    return False

# ========== 6. Проверка пересечения отрезка с многоугольником ==========
def segment_intersects_polygon(p1, p2, obstacle):
    for edge in obstacle.get_edges():
        if segments_intersect(p1, p2, edge[0], edge[1]):
            return True
    return False

# ========== 7. Расширение одного дерева (шаг RRT) ==========
def rrt_extend(tree, target_point, obstacles, step_size=STEP_SIZE):
    random_node = Node(target_point[0], target_point[1])
    nearest = tree.nearest(random_node)

    dx = random_node.x - nearest.x
    dy = random_node.y - nearest.y
    dist = math.hypot(dx, dy)

    if dist < 1e-6:
        return None

    new_x = nearest.x + (dx / dist) * min(step_size, dist)
    new_y = nearest.y + (dy / dist) * min(step_size, dist)
    new_node = Node(new_x, new_y)

    # Проверка столкновений
    for obs in obstacles:
        if segment_intersects_polygon(nearest.get_pos(), new_node.get_pos(), obs):
            return None

    tree.add_node(new_node)
    new_node.parent = nearest
    nearest.children.append(new_node)
    new_node.incoming.append(nearest)
    return new_node

# ========== 8. Проверка возможности соединения двух узлов ==========
def can_connect(node_a, node_b, obstacles, max_distance=STEP_SIZE):
    dist = node_a.distance_to(node_b)
    if dist > max_distance:
        return False
    for obs in obstacles:
        if segment_intersects_polygon(node_a.get_pos(), node_b.get_pos(), obs):
            return False
    return True

# ========== 9. Поиск пути от корня до узла ==========
def get_path_from_root(node):
    path = []
    current = node
    while current is not None:
        path.append(current.get_pos())
        current = current.parent
    path.reverse()
    return path

# ========== 10. Стыковка двух маршрутов ==========
def merge_paths(path_start_to_connection, path_goal_to_connection):
    # path_start_to_connection: от старта до точки встречи
    # path_goal_to_connection: от цели до точки встречи (нужно инвертировать)
    path_connection_to_goal = list(reversed(path_goal_to_connection))
    # Объединяем, исключая дублирование точки встречи
    full_path = path_start_to_connection + path_connection_to_goal[1:]
    return full_path

# ========== 11. Генерация случайного препятствия-многоугольника ==========
def generate_random_polygon(center_x, center_y, avg_radius=40, num_vertices=5):
    angles = sorted([random.uniform(0, 2*math.pi) for _ in range(num_vertices)])
    vertices = []
    for angle in angles:
        r = random.uniform(avg_radius * 0.6, avg_radius * 1.4)
        x = center_x + r * math.cos(angle)
        y = center_y + r * math.sin(angle)
        vertices.append((x, y))
    return vertices

# ========== ГЛАВНАЯ ФУНКЦИЯ ==========
def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Bidirectional RRT (RRT-Connect) Path Planning")
    clock = pygame.time.Clock()

    # Входные параметры
    start_pos = (100, 100)
    goal_pos = (500, 500)

    # Генерация 5 случайных препятствий
    obstacles = []
    for _ in range(5):
        center = (random.randint(150, 650), random.randint(150, 500))
        verts = generate_random_polygon(center[0], center[1], random.randint(35, 60), random.randint(4, 7))
        obstacles.append(Obstacle(verts))

    # Создание двух деревьев
    tree_start = Graph()
    tree_goal = Graph()

    # Корневые узлы
    start_root = Node(start_pos[0], start_pos[1])
    goal_root = Node(goal_pos[0], goal_pos[1])

    tree_start.add_node(start_root)
    tree_start.root = start_root
    tree_goal.add_node(goal_root)
    tree_goal.root = goal_root

    connection_node_start = None
    connection_node_goal = None
    running = True
    iterations = 0

    # Основной цикл RRT-Connect
    while running and iterations < MAX_ITER:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()

        # Генерация случайной точки
        rand_point = (random.uniform(0, WIDTH), random.uniform(0, HEIGHT))

        # Расширяем дерево от старта
        new_node_start = rrt_extend(tree_start, rand_point, obstacles, STEP_SIZE)
        if new_node_start is not None:
            # Проверяем соединение с деревом цели
            for node_goal in tree_goal.nodes:
                if can_connect(new_node_start, node_goal, obstacles, STEP_SIZE):
                    connection_node_start = new_node_start
                    connection_node_goal = node_goal
                    print("Connection found!")
                    running = False
                    break

        # Если уже нашли соединение - выходим
        if not running:
            break

        # Расширяем дерево от цели
        new_node_goal = rrt_extend(tree_goal, rand_point, obstacles, STEP_SIZE)
        if new_node_goal is not None:
            # Проверяем соединение с деревом старта
            for node_start in tree_start.nodes:
                if can_connect(new_node_goal, node_start, obstacles, STEP_SIZE):
                    connection_node_start = node_start
                    connection_node_goal = new_node_goal
                    print("Connection found!")
                    running = False
                    break

        iterations += 1

        # ========== ВИЗУАЛИЗАЦИЯ ==========
        screen.fill(BACKGROUND_COLOR)

        # Рисуем препятствия
        for obs in obstacles:
            pygame.draw.polygon(screen, OBSTACLE_COLOR, obs.vertices, 0)
            pygame.draw.polygon(screen, (0, 0, 0), obs.vertices, 2)

        # Рисуем дерево от старта (зелёное)
        for node in tree_start.nodes:
            if node.parent is not None:
                pygame.draw.line(screen, TREE_START_COLOR, node.get_pos(), node.parent.get_pos(), 2)
            pygame.draw.circle(screen, TREE_START_COLOR, node.get_pos(), NODE_RADIUS)

        # Рисуем дерево от цели (синее)
        for node in tree_goal.nodes:
            if node.parent is not None:
                pygame.draw.line(screen, TREE_GOAL_COLOR, node.get_pos(), node.parent.get_pos(), 2)
            pygame.draw.circle(screen, TREE_GOAL_COLOR, node.get_pos(), NODE_RADIUS)

        # Стартовая и целевая точки
        pygame.draw.circle(screen, ROBOT_COLOR, start_pos, 8)
        pygame.draw.circle(screen, TARGET_COLOR, goal_pos, 10)

        pygame.display.flip()
        clock.tick(FPS)

    # ========== ПОСТРОЕНИЕ ИТОГОВОГО ПУТИ ==========
    if connection_node_start is not None and connection_node_goal is not None:
        # Получаем путь от старта до точки встречи
        path_start_to_conn = get_path_from_root(connection_node_start)
        # Получаем путь от цели до точки встречи
        path_goal_to_conn = get_path_from_root(connection_node_goal)

        # Стыкуем маршруты
        full_path = merge_paths(path_start_to_conn, path_goal_to_conn)

        # Отрисовка итогового пути
        for i in range(len(full_path) - 1):
            pygame.draw.line(screen, PATH_COLOR, full_path[i], full_path[i+1], 5)

        # Отображаем точку встречи
        if len(path_start_to_conn) > 0:
            meeting_point = path_start_to_conn[-1]
            pygame.draw.circle(screen, CONNECTION_COLOR, meeting_point, 8)

        print(f"Path found! Path length: {len(full_path)} points")
    else:
        print(f"Goal not reached within {MAX_ITER} iterations")

    # Ожидание закрытия окна
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