import pygame
import sys
from enum import Enum
import time

# Инициализация Pygame
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Планирование маршрута - Итерация значений")
clock = pygame.time.Clock()
FPS = 60

# Цвета
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
DARK_GRAY = (100, 100, 100)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)
LIGHT_BLUE = (173, 216, 230)

class Direction(Enum):
    """Направления движения"""
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3

class Cell:
    """Класс ячейки виртуального мира"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.neighbours = {}
        self.hasObstacle = False
        self.value = 0.0
        self.reward = 0.0
        self.best_action = None
        self.is_start = False
        self.is_goal = False

    def set_neighbour(self, direction, cell):
        """Установка соседней ячейки"""
        self.neighbours[direction] = cell

    def get_possible_actions(self):
        """Получение возможных действий из ячейки"""
        return [dir for dir, cell in self.neighbours.items() 
                if cell is not None and not cell.hasObstacle]

class Grid:
    """Класс сетки мира"""
    def __init__(self, width, height, cell_size):
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.cells = {}
        self.start_cell = None
        self.goal_cell = None
        self.create_grid()
        
    def create_grid(self):
        """Создание сетки ячеек"""
        for x in range(self.width):
            for y in range(self.height):
                self.cells[(x, y)] = Cell(x, y)
        
        for x in range(self.width):
            for y in range(self.height):
                cell = self.cells[(x, y)]
                
                if y > 0:
                    cell.set_neighbour(Direction.UP, self.cells[(x, y-1)])
                if y < self.height - 1:
                    cell.set_neighbour(Direction.DOWN, self.cells[(x, y+1)])
                if x > 0:
                    cell.set_neighbour(Direction.LEFT, self.cells[(x-1, y)])
                if x < self.width - 1:
                    cell.set_neighbour(Direction.RIGHT, self.cells[(x+1, y)])
    
    def set_start(self, x, y):
        """Установка начальной ячейки"""
        self.start_cell = self.cells[(x, y)]
        self.start_cell.is_start = True
    
    def set_goal(self, x, y):
        """Установка целевой ячейки"""
        self.goal_cell = self.cells[(x, y)]
        self.goal_cell.is_goal = True
        self.goal_cell.reward = 1.0
    
    def set_obstacle(self, x, y):
        """Установка препятствия"""
        self.cells[(x, y)].hasObstacle = True
        self.cells[(x, y)].reward = -1.0
    
    def get_cell(self, x, y):
        """Получение ячейки по координатам"""
        return self.cells.get((x, y))

class Robot:
    """Класс мобильного робота"""
    def __init__(self, start_cell):
        self.current_cell = start_cell
        self.path = []
        self.path.append(start_cell)
        
    def move_to(self, cell):
        """Перемещение в указанную ячейку"""
        self.current_cell = cell
        self.path.append(cell)
    
    def get_position(self):
        """Получение текущей позиции"""
        return (self.current_cell.x, self.current_cell.y)

class ValueIteration:
    """Класс алгоритма итерации значений"""
    def __init__(self, grid, gamma=0.9, threshold=0.0001):
        self.grid = grid
        self.gamma = gamma
        self.threshold = threshold
        self.iteration_history = []
        
    def calculate_value(self, cell):
        """Расчет значения для одной ячейки"""
        if cell.hasObstacle or cell.is_goal:
            return cell.reward
        
        possible_actions = cell.get_possible_actions()
        if not possible_actions:
            return cell.value
        
        max_value = float('-inf')
        best_action = None
        
        for action in possible_actions:
            next_cell = cell.neighbours[action]
            value = cell.reward + self.gamma * next_cell.value
            
            if value > max_value:
                max_value = value
                best_action = action
        
        cell.best_action = best_action
        return max_value
    
    def run(self, max_iterations=1000):
        """Запуск алгоритма итерации значений"""
        print("Запуск алгоритма итерации значений...")
        
        for iteration in range(max_iterations):
            max_delta = 0.0
            current_values = {}
            
            for cell in self.grid.cells.values():
                current_values[(cell.x, cell.y)] = cell.value
            
            for cell in self.grid.cells.values():
                if not cell.hasObstacle:
                    old_value = cell.value
                    cell.value = self.calculate_value(cell)
                    delta = abs(old_value - cell.value)
                    max_delta = max(max_delta, delta)
            
            if iteration % 10 == 0 or iteration < 10:
                values_snapshot = {}
                for cell in self.grid.cells.values():
                    values_snapshot[(cell.x, cell.y)] = cell.value
                self.iteration_history.append(values_snapshot)
            
            if max_delta < self.threshold:
                print(f"Алгоритм сошелся за {iteration + 1} итераций")
                break
        
        print("Расчет значений завершен")
    
    def find_path(self, start_cell, goal_cell):
        """Поиск маршрута по градиенту функции значений"""
        path = [start_cell]
        current = start_cell
        visited = set()
        max_steps = self.grid.width * self.grid.height
        
        for _ in range(max_steps):
            visited.add((current.x, current.y))
            
            if current == goal_cell:
                print(f"Маршрут найден! Длина пути: {len(path)}")
                return path
            
            best_next = None
            best_value = float('-inf')
            
            for action in current.get_possible_actions():
                next_cell = current.neighbours[action]
                if (next_cell.x, next_cell.y) not in visited:
                    if next_cell.value > best_value:
                        best_value = next_cell.value
                        best_next = next_cell
            
            if best_next is None:
                print("Путь не найден - зашли в тупик")
                return path
            
            current = best_next
            path.append(current)
        
        print("Достигнуто максимальное число шагов")
        return path

def draw_grid(screen, grid, offset_x, offset_y):
    """Отрисовка сетки мира"""
    cell_size = grid.cell_size
    
    for (x, y), cell in grid.cells.items():
        rect = pygame.Rect(
            offset_x + x * cell_size,
            offset_y + y * cell_size,
            cell_size,
            cell_size
        )
        
        if cell.hasObstacle:
            color = DARK_GRAY
        elif cell.is_goal:
            color = GREEN
        elif cell.is_start:
            color = BLUE
        else:
            value = cell.value
            if value > 0:
                intensity = min(255, int(value * 200))
                color = (intensity, 255, intensity)
            else:
                intensity = max(0, int(255 + value * 100))
                color = (255, intensity, intensity)
        
        pygame.draw.rect(screen, color, rect)
        pygame.draw.rect(screen, BLACK, rect, 2)
        
        font = pygame.font.Font(None, 20)
        if cell.hasObstacle:
            text = font.render("X", True, WHITE)
        else:
            text = font.render(f"{cell.value:.2f}", True, BLACK)
        
        text_rect = text.get_rect(center=rect.center)
        screen.blit(text, text_rect)
        
        if cell.best_action and not cell.hasObstacle and not cell.is_goal:
            center = rect.center
            arrow_size = cell_size // 4
            
            if cell.best_action == Direction.UP:
                end_point = (center[0], center[1] - arrow_size)
            elif cell.best_action == Direction.DOWN:
                end_point = (center[0], center[1] + arrow_size)
            elif cell.best_action == Direction.LEFT:
                end_point = (center[0] - arrow_size, center[1])
            elif cell.best_action == Direction.RIGHT:
                end_point = (center[0] + arrow_size, center[1])
            
            pygame.draw.line(screen, RED, center, end_point, 3)

def draw_path(screen, path, grid, offset_x, offset_y):
    """Отрисовка найденного маршрута"""
    if len(path) < 2:
        return
    
    cell_size = grid.cell_size
    
    points = []
    for cell in path:
        center_x = offset_x + cell.x * cell_size + cell_size // 2
        center_y = offset_y + cell.y * cell_size + cell_size // 2
        points.append((center_x, center_y))
    
    if len(points) > 1:
        pygame.draw.lines(screen, YELLOW, False, points, 5)
    
    for i, cell in enumerate(path):
        center_x = offset_x + cell.x * cell_size + cell_size // 2
        center_y = offset_y + cell.y * cell_size + cell_size // 2
        
        if i == 0:
            color = BLUE
        elif i == len(path) - 1:
            color = GREEN
        else:
            color = ORANGE
        
        pygame.draw.circle(screen, color, (center_x, center_y), 10)
        pygame.draw.circle(screen, BLACK, (center_x, center_y), 10, 2)

def draw_robot(screen, robot, grid, offset_x, offset_y):
    """Отрисовка робота"""
    cell_size = grid.cell_size
    center_x = offset_x + robot.current_cell.x * cell_size + cell_size // 2
    center_y = offset_y + robot.current_cell.y * cell_size + cell_size // 2
    
    pygame.draw.circle(screen, PURPLE, (center_x, center_y), 15)
    pygame.draw.circle(screen, BLACK, (center_x, center_y), 15, 3)
    
    eye_offset = 5
    pygame.draw.circle(screen, WHITE, (center_x - eye_offset, center_y - eye_offset), 4)
    pygame.draw.circle(screen, WHITE, (center_x + eye_offset, center_y - eye_offset), 4)

def draw_legend(screen, x, y):
    """Отрисовка легенды слева"""
    font_title = pygame.font.Font(None, 28)
    font_text = pygame.font.Font(None, 22)
    
    # Фон для легенды
    legend_rect = pygame.Rect(x - 10, y - 10, 200, 200)
    pygame.draw.rect(screen, WHITE, legend_rect)
    pygame.draw.rect(screen, BLACK, legend_rect, 2)
    
    # Заголовок легенды
    title = font_title.render("ЛЕГЕНДА", True, BLACK)
    screen.blit(title, (x + 25, y))
    
    y += 25
    
    # Элементы легенды
    legend_items = [
        ("Начальная точка", BLUE),
        ("Целевая точка", GREEN),
        ("Препятствие", DARK_GRAY),
        ("Робот", PURPLE),
        ("Маршрут", YELLOW),
        ("Направление", RED),
        ("Значение V(s)", LIGHT_BLUE)
    ]
    
    for text, color in legend_items:
        # Отрисовка цветного квадрата
        pygame.draw.rect(screen, color, (x, y, 25, 25))
        pygame.draw.rect(screen, BLACK, (x, y, 25, 25), 1)
        
        # Текст
        text_surface = font_text.render(text, True, BLACK)
        screen.blit(text_surface, (x + 35, y + 2))
        
        y += 20

def draw_info_panel(screen, x, y, grid, robot, path, path_index):
    """Отрисовка информационной панели слева"""
    font_title = pygame.font.Font(None, 25)
    font_text = pygame.font.Font(None, 20)
    
    # Фон для информационной панели
    info_rect = pygame.Rect(x - 10, y - 30, 210, 220)
    pygame.draw.rect(screen, WHITE, info_rect)
    pygame.draw.rect(screen, BLACK, info_rect, 2)
    
    # Заголовок
    title = font_title.render("ИНФОРМАЦИЯ", True, BLACK)
    screen.blit(title, (x + 10, y - 20))
    
    y += 12
    
    # Информационные строки
    info_lines = [
        f"Сетка: {grid.width}×{grid.height}",
        f"Старт: (0,0)",
        f"Цель: (4,4)",
        f"Препятствия: (2,2), (2,3)",
        f"Шаг: {path_index + 1}/{len(path)}",
        f"Длина пути: {len(path)}",
        f"γ = 0.9"
    ]
    
    for line in info_lines:
        text_surface = font_text.render(line, True, BLACK)
        screen.blit(text_surface, (x, y))
        y += 25

def draw_controls(screen, x, y):
    """Отрисовка панели управления слева"""
    font_title = pygame.font.Font(None, 28)
    font_text = pygame.font.Font(None, 20)
    
    # Фон для панели управления
    control_rect = pygame.Rect(x - 10, y - 165, 210, 100)
    pygame.draw.rect(screen, WHITE, control_rect)
    pygame.draw.rect(screen, BLACK, control_rect, 2)
    
    # Заголовок
    # title = font_title.render("УПРАВЛЕНИЕ", True, BLACK)
    # screen.blit(title, (x + 40, y))
    
    y += -150
    
    # Клавиши управления
    controls = [
        "ПРОБЕЛ - следующий шаг",
        "V - показать/скрыть значения",
        "R - перезапуск"
    ]
    
    for control in controls:
        text_surface = font_text.render(control, True, BLACK)
        screen.blit(text_surface, (x, y))
        y += 25

def main():
    """Основная функция"""
    # Параметры сетки
    GRID_WIDTH = 5
    GRID_HEIGHT = 5
    CELL_SIZE = 100
    
    # Смещение сетки вправо для размещения легенды слева
    offset_x = 300
    offset_y = (HEIGHT - GRID_HEIGHT * CELL_SIZE) // 2
    
    # Создание сетки
    grid = Grid(GRID_WIDTH, GRID_HEIGHT, CELL_SIZE)
    grid.set_start(0, 0)
    grid.set_goal(4, 4)
    grid.set_obstacle(2, 2)
    grid.set_obstacle(2, 3)
    
    # Создание робота
    robot = Robot(grid.start_cell)
    
    # Запуск алгоритма итерации значений
    value_iteration = ValueIteration(grid, gamma=0.9, threshold=0.0001)
    value_iteration.run()
    
    # Поиск маршрута
    path = value_iteration.find_path(grid.start_cell, grid.goal_cell)
    
    # Перемещение робота
    path_index = 0
    move_delay = 500
    last_move_time = 0
    
    # Вывод результатов в консоль
    print("\n=== РЕЗУЛЬТАТЫ ===")
    print(f"Размер сетки: {GRID_WIDTH}x{GRID_HEIGHT}")
    print(f"Начальная точка: (0, 0)")
    print(f"Конечная точка: (4, 4)")
    print(f"Препятствия: (2, 2), (2, 3)")
    print(f"\nЗначения в ячейках:")
    for y in range(GRID_HEIGHT):
        for x in range(GRID_WIDTH):
            cell = grid.get_cell(x, y)
            if cell.hasObstacle:
                print(f"  [{x},{y}]: ПРЕПЯТСТВИЕ")
            else:
                print(f"  [{x},{y}]: V={cell.value:.4f}", end="")
                if cell.best_action:
                    print(f", Действие={cell.best_action.name}")
                else:
                    print()
    
    print(f"\nНайденный маршрут:")
    for i, cell in enumerate(path):
        print(f"  Шаг {i}: ({cell.x}, {cell.y})")
    
    # Главный цикл
    running = True
    show_values = True
    
    while running:
        current_time = pygame.time.get_ticks()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if path_index < len(path) - 1:
                        path_index += 1
                        robot.move_to(path[path_index])
                
                elif event.key == pygame.K_v:
                    show_values = not show_values
                
                elif event.key == pygame.K_r:
                    # Перезапуск
                    grid = Grid(GRID_WIDTH, GRID_HEIGHT, CELL_SIZE)
                    grid.set_start(0, 0)
                    grid.set_goal(4, 4)
                    grid.set_obstacle(2, 2)
                    grid.set_obstacle(2, 3)
                    robot = Robot(grid.start_cell)
                    value_iteration = ValueIteration(grid, gamma=0.9, threshold=0.0001)
                    value_iteration.run()
                    path = value_iteration.find_path(grid.start_cell, grid.goal_cell)
                    path_index = 0
                    last_move_time = 0
        
        # Автоматическое движение
        if current_time - last_move_time > move_delay and path_index < len(path) - 1:
            path_index += 1
            robot.move_to(path[path_index])
            last_move_time = current_time
        
        # Очистка экрана
        screen.fill(WHITE)
        
        # Заголовок
        font_title = pygame.font.Font(None, 26)
        title = font_title.render("Планирование маршрута мобильного робота | Метод итерации значений", True, BLACK)
        screen.blit(title, (WIDTH // 2 - title.get_width() // 2, 10))
        
        # Отрисовка легенды и панелей слева
        draw_legend(screen, 20, 80)
        draw_info_panel(screen, 20, 410, grid, robot, path, path_index)
        draw_controls(screen, 20, 440)
        
        # Отрисовка сетки и маршрута справа
        if show_values:
            draw_grid(screen, grid, offset_x, offset_y)
        else:
            # Отрисовка пустой сетки
            for x in range(GRID_WIDTH):
                for y in range(GRID_HEIGHT):
                    rect = pygame.Rect(
                        offset_x + x * CELL_SIZE,
                        offset_y + y * CELL_SIZE,
                        CELL_SIZE,
                        CELL_SIZE
                    )
                    pygame.draw.rect(screen, WHITE, rect)
                    pygame.draw.rect(screen, BLACK, rect, 2)
                    
                    cell = grid.get_cell(x, y)
                    if cell.hasObstacle:
                        pygame.draw.rect(screen, DARK_GRAY, rect)
                        font = pygame.font.Font(None, 20)
                        text = font.render("X", True, WHITE)
                        text_rect = text.get_rect(center=rect.center)
                        screen.blit(text, text_rect)
        
        draw_path(screen, path, grid, offset_x, offset_y)
        draw_robot(screen, robot, grid, offset_x, offset_y)
        
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
