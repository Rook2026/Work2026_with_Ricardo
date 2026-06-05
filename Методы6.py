import pygame
import math
import random
import numpy as np
from collections import defaultdict
import time

# Инициализация Pygame
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Манипуляционный робот с ассоциативной памятью")
clock = pygame.time.Clock()
FPS = 60

# Цвета
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (128, 128, 128)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)

class Robot:
    """Класс манипуляционного робота"""
    def __init__(self, x, y, L1=150, L2=100):
        self.x = x  # Координата X основания
        self.y = y  # Координата Y основания
        self.L1 = L1  # Длина первого звена
        self.L2 = L2  # Длина второго звена
        self.a1 = 0  # Угол первого звена (в радианах)
        self.a2 = 0  # Угол второго звена (в радианах)
        self.v1 = 0  # Скорость первого звена
        self.v2 = 0  # Скорость второго звена
        self.gripper_open = 15  # Раскрытие захвата
        
    def forward_kinematics(self):
        """Прямая кинематика - вычисление позиции схвата"""
        # Конец первого звена
        x1 = self.x + self.L1 * math.cos(self.a1)
        y1 = self.y - self.L1 * math.sin(self.a1)
        
        # Конец второго звена (схват)
        x2 = x1 + self.L2 * math.cos(self.a1 + self.a2)
        y2 = y1 - self.L2 * math.sin(self.a1 + self.a2)
        
        return (self.x, self.y), (x1, y1), (x2, y2)
    
    def get_end_effector_pos(self):
        """Получение позиции схвата"""
        _, _, end_pos = self.forward_kinematics()
        return end_pos
    
    def move(self, v1, v2, dt=0.1):
        """Движение робота с заданными скоростями"""
        self.v1 = v1
        self.v2 = v2
        self.a1 += v1 * dt
        self.a2 += v2 * dt
        
        # Ограничения углов
        self.a1 = max(-math.pi, min(math.pi, self.a1))
        self.a2 = max(-math.pi * 0.8, min(math.pi * 0.8, self.a2))

class Object:
    """Класс целевого объекта"""
    def __init__(self, x, y, R=10):
        self.x = x  # Координата X
        self.y = y  # Координата Y
        self.R = R  # Размер объекта
        self.captured = False  # Флаг захвата
    
    def distance_to(self, point):
        """Расстояние до точки"""
        return math.sqrt((self.x - point[0])**2 + (self.y - point[1])**2)

class AssociativeMemory:
    """Класс ассоциативной памяти"""
    def __init__(self):
        self.memory = defaultdict(list)  # Память: ключ -> список значений
        
    def discretize_state(self, a1, a2, b, bins=(8, 8, 8)):
        """Дискретизация состояния для табличной памяти"""
        a1_range = (-math.pi, math.pi)
        a2_range = (-math.pi * 0.8, math.pi * 0.8)
        b_range = (-math.pi, math.pi)
        
        # Нормализация и дискретизация
        a1_idx = int((a1 - a1_range[0]) / (a1_range[1] - a1_range[0]) * bins[0])
        a2_idx = int((a2 - a2_range[0]) / (a2_range[1] - a2_range[0]) * bins[1])
        b_idx = int((b - b_range[0]) / (b_range[1] - b_range[0]) * bins[2])
        
        # Ограничение индексов
        a1_idx = max(0, min(bins[0] - 1, a1_idx))
        a2_idx = max(0, min(bins[1] - 1, a2_idx))
        b_idx = max(0, min(bins[2] - 1, b_idx))
        
        return (a1_idx, a2_idx, b_idx)
    
    def store(self, a1, a2, b, v1, v2):
        """Сохранение в память"""
        key = self.discretize_state(a1, a2, b)
        self.memory[key].append((v1, v2))
        
        # Ограничение размера памяти для каждого ключа
        if len(self.memory[key]) > 50:
            self.memory[key].pop(0)
    
    def recall(self, a1, a2, b):
        """Извлечение из памяти (интерполяция)"""
        key = self.discretize_state(a1, a2, b)
        
        if key in self.memory and len(self.memory[key]) > 0:
            # Усреднение всех сохраненных значений для данного состояния
            values = self.memory[key]
            avg_v1 = sum(v[0] for v in values) / len(values)
            avg_v2 = sum(v[1] for v in values) / len(values)
            return avg_v1, avg_v2
        
        # Поиск ближайшего соседа
        min_dist = float('inf')
        nearest_values = None
        
        for mem_key, mem_values in self.memory.items():
            # Расстояние между ключами
            dist = sum((a - b)**2 for a, b in zip(key, mem_key))
            if dist < min_dist:
                min_dist = dist
                nearest_values = mem_values
        
        if nearest_values:
            avg_v1 = sum(v[0] for v in nearest_values) / len(nearest_values)
            avg_v2 = sum(v[1] for v in nearest_values) / len(nearest_values)
            return avg_v1, avg_v2
        
        # Если память пуста, возвращаем случайные значения
        return random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5)

def draw_robot(screen, robot, capture_threshold=15):
    """Отрисовка робота"""
    base, joint, end_effector = robot.forward_kinematics()
    
    # Отрисовка основания
    pygame.draw.circle(screen, BLACK, (int(base[0]), int(base[1])), 10)
    
    # Отрисовка первого звена
    pygame.draw.line(screen, BLUE, base, joint, 5)
    pygame.draw.circle(screen, BLACK, (int(joint[0]), int(joint[1])), 6)
    
    # Отрисовка второго звена
    pygame.draw.line(screen, GREEN, joint, end_effector, 5)
    
    # Отрисовка захвата
    gripper_size = robot.gripper_open
    angle = robot.a1 + robot.a2
    
    # Левая губка захвата
    left_gripper = (
        end_effector[0] + gripper_size * math.cos(angle + math.pi/2),
        end_effector[1] - gripper_size * math.sin(angle + math.pi/2)
    )
    
    # Правая губка захвата
    right_gripper = (
        end_effector[0] + gripper_size * math.cos(angle - math.pi/2),
        end_effector[1] - gripper_size * math.sin(angle - math.pi/2)
    )
    
    pygame.draw.line(screen, RED, end_effector, left_gripper, 3)
    pygame.draw.line(screen, RED, end_effector, right_gripper, 3)
    
    # Отрисовка круга захвата
    pygame.draw.circle(screen, YELLOW, (int(end_effector[0]), int(end_effector[1])), 
                      capture_threshold, 1)
    
    return end_effector

def draw_object(screen, obj):
    """Отрисовка объекта"""
    color = GREEN if obj.captured else RED
    pygame.draw.circle(screen, color, (int(obj.x), int(obj.y)), obj.R)
    pygame.draw.circle(screen, BLACK, (int(obj.x), int(obj.y)), obj.R, 2)

def calculate_b(robot, target_point):
    """Вычисление углового рассогласования b"""
    end_effector = robot.get_end_effector_pos()
    dx = target_point[0] - end_effector[0]
    dy = target_point[1] - end_effector[1]
    target_angle = math.atan2(-dy, dx)
    current_angle = robot.a1 + robot.a2
    b = target_angle - current_angle
    
    # Нормализация угла в [-pi, pi]
    while b > math.pi:
        b -= 2 * math.pi
    while b < -math.pi:
        b += 2 * math.pi
    
    return b

def generate_particles(screen, robot, num_particles=20):
    """Генерация частиц в зоне предполагаемого расположения ориентира"""
    end_effector = robot.get_end_effector_pos()
    particles = []
    
    for _ in range(num_particles):
        # Случайное смещение от схвата
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(20, 100)
        
        px = end_effector[0] + distance * math.cos(angle)
        py = end_effector[1] + distance * math.sin(angle)
        
        # Ограничение в пределах экрана
        px = max(0, min(WIDTH, px))
        py = max(0, min(HEIGHT, py))
        
        particles.append((px, py))
        
        # Отрисовка частицы
        pygame.draw.circle(screen, CYAN, (int(px), int(py)), 2)
    
    return particles

def main():
    """Основная функция"""
    # Инициализация робота и объекта
    robot = Robot(400, 300)
    target_object = Object(500, 200, R=10)
    associative_memory = AssociativeMemory()
    
    # Режимы работы
    MANUAL_MODE = 0
    AUTO_MODE = 1
    current_mode = MANUAL_MODE
    
    # Переменные для ручного управления
    manual_v1 = 0
    manual_v2 = 0
    
    # История движения
    movement_history = []
    
    # Для автоматического режима
    auto_start_time = 0
    capture_timeout = 10  # Таймаут захвата в секундах
    
    running = True
    font = pygame.font.Font(None, 24)
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    # Переключение режимов
                    current_mode = AUTO_MODE if current_mode == MANUAL_MODE else MANUAL_MODE
                    if current_mode == AUTO_MODE:
                        auto_start_time = time.time()
                        print("Переключение в автоматический режим")
                    else:
                        print("Переключение в ручной режим")
                
                elif event.key == pygame.K_r:
                    # Сброс позиции робота и объекта
                    robot.a1 = 0
                    robot.a2 = 0
                    target_object.captured = False
                    target_object.x = random.randint(300, 500)
                    target_object.y = random.randint(150, 350)
                    movement_history.clear()
                    print("Сброс позиции")
                
                # Ручное управление
                if current_mode == MANUAL_MODE:
                    if event.key == pygame.K_LEFT:
                        manual_v1 = -1.0
                    elif event.key == pygame.K_RIGHT:
                        manual_v1 = 1.0
                    elif event.key == pygame.K_UP:
                        manual_v2 = 0.5
                    elif event.key == pygame.K_DOWN:
                        manual_v2 = -0.5
            
            elif event.type == pygame.KEYUP:
                if current_mode == MANUAL_MODE:
                    if event.key in [pygame.K_LEFT, pygame.K_RIGHT]:
                        manual_v1 = 0
                    elif event.key in [pygame.K_UP, pygame.K_DOWN]:
                        manual_v2 = 0
        
        # Очистка экрана
        screen.fill(WHITE)
        
        # Движение робота
        if current_mode == MANUAL_MODE:
            # Ручной режим
            robot.move(manual_v1, manual_v2)
            
            # Сохранение данных в память
            end_effector = robot.get_end_effector_pos()
            b = calculate_b(robot, (target_object.x, target_object.y))
            
            movement_history.append({
                't': time.time(),
                'a1': robot.a1,
                'a2': robot.a2,
                'v1': manual_v1,
                'v2': manual_v2,
                'b': b
            })
            
            associative_memory.store(robot.a1, robot.a2, b, manual_v1, manual_v2)
            
            # Проверка захвата
            dist = target_object.distance_to(end_effector)
            if dist < 15 and not target_object.captured:
                target_object.captured = True
                print("Объект захвачен!")
        
        else:
            # Автоматический режим
            end_effector = robot.get_end_effector_pos()
            b = calculate_b(robot, (target_object.x, target_object.y))
            
            # Извлечение управляющих сигналов из памяти
            v1, v2 = associative_memory.recall(robot.a1, robot.a2, b)
            
            # Корректировка движения к цели
            dist_to_target = target_object.distance_to(end_effector)
            
            if dist_to_target > 15:
                # Усиление сигнала для приближения к цели
                angle_to_target = math.atan2(
                    -(target_object.y - end_effector[1]),
                    target_object.x - end_effector[0]
                )
                
                # Простое пропорциональное управление
                angle_diff = angle_to_target - (robot.a1 + robot.a2)
                v1 += 0.1 * math.cos(angle_diff)
                v2 += 0.1 * math.sin(angle_diff)
                
                robot.move(v1, v2)
                
                # Проверка захвата
                if dist_to_target < 15:
                    target_object.captured = True
                    print("Автоматический захват выполнен!")
            else:
                target_object.captured = True
            
            # Таймаут автоматического режима
            if time.time() - auto_start_time > capture_timeout:
                current_mode = MANUAL_MODE
                print("Таймаут автоматического режима")
        
        # Отрисовка
        end_effector = draw_robot(screen, robot)
        draw_object(screen, target_object)
        
        # Генерация частиц
        if not target_object.captured:
            generate_particles(screen, robot)
        
        # Отображение информации
        mode_text = "Ручной режим" if current_mode == MANUAL_MODE else "Автоматический режим"
        info_texts = [
            f"Режим: {mode_text}",
            f"Углы: a1={math.degrees(robot.a1):.1f}°, a2={math.degrees(robot.a2):.1f}°",
            f"Скорости: v1={robot.v1:.2f}, v2={robot.v2:.2f}",
            f"Схват: ({end_effector[0]:.1f}, {end_effector[1]:.1f})",
            f"Объект: {'Захвачен' if target_object.captured else 'Свободен'}",
            f"Память: {len(associative_memory.memory)} состояний"
        ]
        
        y_offset = 10
        for text in info_texts:
            text_surface = font.render(text, True, BLACK)
            screen.blit(text_surface, (10, y_offset))
            y_offset += 25
        
        # Отрисовка траектории
        if len(movement_history) > 1:
            points = []
            for record in movement_history[-100:]:  # Последние 100 точек
                # Вычисление позиции схвата для исторических данных
                x1 = robot.x + robot.L1 * math.cos(record['a1'])
                y1 = robot.y - robot.L1 * math.sin(record['a1'])
                x2 = x1 + robot.L2 * math.cos(record['a1'] + record['a2'])
                y2 = y1 - robot.L2 * math.sin(record['a1'] + record['a2'])
                points.append((int(x2), int(y2)))
            
            if len(points) > 1:
                pygame.draw.lines(screen, GRAY, False, points, 1)
        
        # Управляющие подсказки
        help_texts = [
            "Управление: Стрелки - движение, SPACE - режим, R - сброс"
        ]
        for i, text in enumerate(help_texts):
            text_surface = font.render(text, True, BLACK)
            screen.blit(text_surface, (10, HEIGHT - 25 - i * 25))
        
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()

if __name__ == "__main__":
    main()
