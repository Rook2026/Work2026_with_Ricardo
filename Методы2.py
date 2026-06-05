import pygame
import sys
from collections import deque

# Константы
WIDTH, HEIGHT = 800, 600
MIN_X, MAX_X = 0, 20
SCALE = (WIDTH - 200) / (MAX_X - MIN_X)
OFFSET_X = 100
ROBOT_COLOR = (0, 255, 0)
OBJECT_COLOR = (255, 0, 0)
BG_COLOR = (30, 30, 30)

# ---------------------- Классы ----------------------
class Robot:
    def __init__(self, x):
        self.x = x

class Obj:
    def __init__(self, name, x):
        self.name = name
        self.x = x

class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = []   # (from, to, action)

# ----------- Вспомогательные функции -----------
def get_nearest_object(robot, objects):
    nearest = None
    min_dist = float('inf')
    for obj in objects:
        dist = abs(robot.x - obj.x)
        if dist < min_dist:
            min_dist = dist
            nearest = obj
    return nearest

def get_state(robot, objects):
    return (robot.x, tuple(obj.x for obj in objects))

def set_state(state, robot, objects):
    robot.x = state[0]
    for obj, x in zip(objects, state[1]):
        obj.x = x

def apply_action(action, robot, objects):
    """Применяет действие без проверок коллизий (объекты могут накладываться)."""
    old_state = get_state(robot, objects)

    if action == "GoRight":
        if robot.x < MAX_X:
            robot.x += 1
        else:
            return None
    elif action == "GoLeft":
        if robot.x > MIN_X:
            robot.x -= 1
        else:
            return None
    elif action == "MoveRight":
        obj = get_nearest_object(robot, objects)
        if obj and obj.x < MAX_X:
            obj.x += 1
        else:
            return None
    elif action == "MoveLeft":
        obj = get_nearest_object(robot, objects)
        if obj and obj.x > MIN_X:
            obj.x -= 1
        else:
            return None
    else:
        return None

    new_state = get_state(robot, objects)
    set_state(old_state, robot, objects)   # откат
    return new_state

def is_goal(state, names, goal_order):
    """Цель: порядок имен после сортировки по координатам = goal_order."""
    _, coords = state
    items = sorted(((coords[i], names[i]) for i in range(len(names))), key=lambda t: t[0])
    order = [name for _, name in items]
    return order == goal_order

def bfs_plan(initial_state, goal_check, max_states=5_000_000):
    graph = Graph()
    parent = {}
    action_from = {}
    queue = deque([initial_state])
    parent[initial_state] = None
    graph.nodes.add(initial_state)
    visited = 0

    while queue and visited < max_states:
        state = queue.popleft()
        visited += 1
        if visited % 100_000 == 0:
            print(f"  Посещено {visited} состояний (в очереди {len(queue)})")

        if goal_check(state):
            # Восстановление плана
            plan = []
            cur = state
            while parent[cur] is not None:
                plan.append(action_from[cur])
                cur = parent[cur]
            plan.reverse()
            return graph, plan, state

        for act in ("GoRight", "GoLeft", "MoveRight", "MoveLeft"):
            temp_robot = Robot(state[0])
            temp_objs = [Obj(name, x) for name, x in zip(["A","B","C","D"], state[1])]
            new_state = apply_action(act, temp_robot, temp_objs)
            if new_state is not None and new_state not in parent:
                parent[new_state] = state
                action_from[new_state] = act
                graph.nodes.add(new_state)
                graph.edges.append((state, new_state, act))
                queue.append(new_state)

    print(f"  BFS завершён после {visited} состояний")
    return None, None, None

# ------------------ Визуализация ------------------
def draw_scene(screen, robot, objects, font):
    screen.fill(BG_COLOR)
    for x in range(MIN_X, MAX_X + 1):
        sx = OFFSET_X + x * SCALE
        pygame.draw.line(screen, (100,100,100), (sx, 100), (sx, 500), 2)
    rx = OFFSET_X + robot.x * SCALE
    pygame.draw.circle(screen, ROBOT_COLOR, (int(rx), 300), 20)
    for obj in objects:
        ox = OFFSET_X + obj.x * SCALE
        pygame.draw.rect(screen, OBJECT_COLOR, (ox - 15, 270, 30, 60))
        text = font.render(obj.name, True, (255,255,255))
        screen.blit(text, (ox - 8, 275))
    pygame.display.flip()

def animate_plan(robot, objects, plan, screen, font):
    clock = pygame.time.Clock()
    for act in plan:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        if act == "GoRight" and robot.x < MAX_X:
            robot.x += 1
        elif act == "GoLeft" and robot.x > MIN_X:
            robot.x -= 1
        elif act == "MoveRight":
            obj = get_nearest_object(robot, objects)
            if obj and obj.x < MAX_X:
                obj.x += 1
        elif act == "MoveLeft":
            obj = get_nearest_object(robot, objects)
            if obj and obj.x > MIN_X:
                obj.x -= 1
        draw_scene(screen, robot, objects, font)
        clock.tick(5)
        pygame.time.wait(200)

# ------------------ Главная функция ------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Планирование робота (МИРЭА)")
    font = pygame.font.SysFont("Arial", 24)

    robot = Robot(0)
    objects = [Obj("A", 1), Obj("B", 2), Obj("C", 3), Obj("D", 4)]
    initial_state = get_state(robot, objects)
    names = ["A","B","C","D"]
    goal_order = ["D","C","B","A"]

    print("Поиск плана (BFS) – может занять до 1 минуты...")
    graph, plan, goal_state = bfs_plan(initial_state,
                                       lambda s: is_goal(s, names, goal_order),
                                       max_states=5_000_000)

    if plan is None:
        print("План не найден. Увеличьте MAX_X или max_states.")
        pygame.quit()
        sys.exit()

    print(f"\n=== ГРАФ СОБЫТИЙ ===")
    print(f"Всего узлов: {len(graph.nodes)}")
    print(f"Всего переходов: {len(graph.edges)}")
    print("Пример переходов (первые 10):")
    for i, (f, t, a) in enumerate(graph.edges[:10]):
        print(f"  {f} --{a}--> {t}")
    if len(graph.edges) > 10:
        print(f"  ... и ещё {len(graph.edges)-10}")

    print(f"\n=== ПЛАН ДЕЙСТВИЙ ({len(plan)} шагов) ===")
    for i, act in enumerate(plan, 1):
        print(f"{i:3d}. {act}")

    # Сброс для анимации
    robot = Robot(0)
    objects = [Obj("A",1), Obj("B",2), Obj("C",3), Obj("D",4)]
    draw_scene(screen, robot, objects, font)
    input("\nНажмите Enter в консоли для запуска анимации...")
    animate_plan(robot, objects, plan, screen, font)

    # Ожидание закрытия окна
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

if __name__ == "__main__":
    main()