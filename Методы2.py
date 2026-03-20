import pygame
import collections

# --- Классы системы ---

class Robot:
    def __init__(self, x):
        self.x = x
        self.holding = None  # Объект в "руках"

class Obj:
    def __init__(self, name, x):
        self.name = name
        self.x = x

class Event:
    def __init__(self, action, state):
        self.action = action  # Строка описания действия
        self.state = state    # Состояние после действия (кортеж координат объектов)

# --- Логика планирования ---

def get_state(robot, objects):
    # Состояние — это кортеж из позиции робота и позиций всех объектов
    return (robot.x, tuple(obj.x for obj in objects))

def solve_plan(start_robot_x, start_obj_positions, target_positions):
    # BFS для поиска кратчайшего пути в графе состояний
    queue = collections.deque([(start_robot_x, start_obj_positions, [])])
    visited = set([(start_robot_x, start_obj_positions)])
    
    while queue:
        r_x, o_pos, path = queue.popleft()
        
        if o_pos == target_positions:
            return path
        
        # Возможные действия: GoLeft, GoRight, MoveLeft (объекта), MoveRight (объекта)
        # Для упрощения: робот перемещается на +/- 100 единиц
        possible_moves = [
            ("GoRight", r_x + 100, o_pos),
            ("GoLeft", r_x - 100, o_pos)
        ]
        
        # Если робот стоит там же, где объект, он может его "двинуть"
        for i, obj_x in enumerate(o_pos):
            if r_x == obj_x:
                new_pos_r = list(o_pos)
                new_pos_l = list(o_pos)
                new_pos_r[i] += 100
                new_pos_l[i] -= 100
                possible_moves.append((f"MoveRight({i})", r_x + 100, tuple(new_pos_r)))
                possible_moves.append((f"MoveLeft({i})", r_x - 100, tuple(new_pos_l)))

        for action, next_rx, next_opos in possible_moves:
            if (next_rx, next_opos) not in visited and 0 <= next_rx <= 800:
                visited.add((next_rx, next_opos))
                queue.append((next_rx, next_opos, path + [action]))
    return None

# --- Визуализация Pygame ---

def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    clock = pygame.time.Clock()
    
    # Инициализация
    robot = Robot(0)
    objects = [Obj('A', 100), Obj('B', 200), Obj('C', 300), Obj('D', 400)]
    target = (400, 300, 200, 100) # Состояние {D, C, B, A}
    
    initial_pos = tuple(o.x for o in objects)
    plan = solve_plan(robot.x, initial_pos, target)
    
    print("Граф событий построен. План действий:")
    for step in plan:
        print(f" -> {step}")

    running = True
    step_idx = 0
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))
        
        # Отрисовка робота
        pygame.draw.rect(screen, (0, 0, 255), (robot.x, 500, 40, 40))
        # Отрисовка объектов
        for obj in objects:
            pygame.draw.circle(screen, (255, 0, 0), (obj.x + 20, 520), 15)
            
        pygame.display.flip()
        clock.tick(10) # 10 FPS для наглядности
        
    pygame.quit()

if __name__ == "__main__":
    main()
