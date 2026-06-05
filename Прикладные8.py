import pygame
import math
import random
import sys

# 1. Инициализация PyGame
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Метод потенциалов для планирования движений робота")
clock = pygame.time.Clock()
FPS = 60

# Цвета
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
GRAY = (128, 128, 128)
PURPLE = (128, 0, 128)

# 2. Классы
class Robot:
    """Мобильный робот с координатами, скоростью и ускорением."""
    def __init__(self, x, y, vx=0, vy=0, ax=0, ay=0):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.ax = ax
        self.ay = ay
        self.trail = []  # траектория движения
        self.mass = 1.0  # масса робота
    
    def update(self, dt=1.0):
        """Обновление состояния робота методом Эйлера."""
        # Обновление скорости
        self.vx += self.ax * dt
        self.vy += self.ay * dt
        
        # Ограничение максимальной скорости (для устойчивости)
        max_speed = 15.0
        speed = math.hypot(self.vx, self.vy)
        if speed > max_speed:
            self.vx = self.vx / speed * max_speed
            self.vy = self.vy / speed * max_speed
        
        # Обновление положения
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # Ограничение границами экрана
        self.x = max(0, min(WIDTH, self.x))
        self.y = max(0, min(HEIGHT, self.y))
        
        # Запись траектории
        self.trail.append((self.x, self.y))
        if len(self.trail) > 1000:
            self.trail.pop(0)
    
    def apply_force(self, fx, fy):
        """Применение силы к роботу (F = m*a)."""
        self.ax = fx / self.mass
        self.ay = fy / self.mass
    
    def distance_to_point(self, px, py):
        """Расстояние до точки."""
        return math.hypot(self.x - px, self.y - py)
    
    def draw(self, surface):
        """Отрисовка робота и его траектории."""
        # Отрисовка траектории
        if len(self.trail) > 1:
            points = [(int(x), int(y)) for x, y in self.trail]
            pygame.draw.lines(surface, GRAY, False, points, 2)
        
        # Отрисовка направления движения (вектор скорости)
        if math.hypot(self.vx, self.vy) > 0.1:
            end_x = self.x + self.vx * 8
            end_y = self.y + self.vy * 8
            pygame.draw.line(surface, BLUE, (self.x, self.y), (end_x, end_y), 3)
        
        # Отрисовка робота
        pygame.draw.circle(surface, BLUE, (int(self.x), int(self.y)), 10)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), 10, 2)
        pygame.draw.circle(surface, WHITE, (int(self.x), int(self.y)), 5)


class Obstacle:
    """Препятствие в виде окружности."""
    def __init__(self, x, y, radius=30):
        self.x = x
        self.y = y
        self.radius = radius
    
    def draw(self, surface):
        """Отрисовка препятствия."""
        pygame.draw.circle(surface, RED, (int(self.x), int(self.y)), self.radius)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), self.radius, 2)
    
    def distance_to(self, px, py):
        """Расстояние от точки до центра препятствия."""
        return math.hypot(px - self.x, py - self.y)
    
    def is_collision(self, px, py, robot_radius=10):
        """Проверка столкновения с роботом."""
        return self.distance_to(px, py) < (self.radius + robot_radius)


class Target:
    """Целевая точка."""
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def draw(self, surface):
        """Отрисовка цели."""
        pygame.draw.circle(surface, GREEN, (int(self.x), int(self.y)), 12)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), 12, 2)
        # Крест в центре
        pygame.draw.line(surface, WHITE, (self.x-8, self.y), (self.x+8, self.y), 2)
        pygame.draw.line(surface, WHITE, (self.x, self.y-8), (self.x, self.y+8), 2)


class PotentialFieldPlanner:
    """
    Планировщик траектории на основе метода искусственных потенциалов.
    """
    def __init__(self, robot, target, obstacles):
        self.robot = robot
        self.target = target
        self.obstacles = obstacles
        
        # Коэффициенты потенциалов
        self.K_att = 0.5       # коэффициент притяжения (сила постоянна по величине)
        self.K_rep = 1500.0    # коэффициент отталкивания (закон Кулона)
        self.d_max = 100.0     # максимальная дальность действия препятствий
        
        # Дополнительные параметры
        self.safe_distance = 35.0  # безопасное расстояние до препятствия
    
    def calculate_attraction_force(self):
        """
        Расчет силы притяжения к цели.
        Сила постоянна по величине и направлена к цели.
        """
        dx = self.target.x - self.robot.x
        dy = self.target.y - self.robot.y
        distance = math.hypot(dx, dy)
        
        if distance < 0.01:
            return 0, 0
        
        # Единичный вектор направления к цели
        fx = dx / distance
        fy = dy / distance
        
        # Сила притяжения постоянна по величине
        fx *= self.K_att
        fy *= self.K_att
        
        return fx, fy
    
    def calculate_repulsion_force(self):
        """
        Расчет силы отталкивания от препятствий.
        На основе модифицированного закона Кулона: F ~ 1/r^2, но направлена от препятствия.
        """
        total_fx = 0.0
        total_fy = 0.0
        
        for obs in self.obstacles:
            dx = self.robot.x - obs.x  # вектор от препятствия к роботу
            dy = self.robot.y - obs.y
            distance = math.hypot(dx, dy)
            
            # Если робот столкнулся с препятствием, возвращаем максимальную силу
            if distance < obs.radius + 5:
                total_fx += dx * 1000
                total_fy += dy * 1000
                continue
            
            # Зона действия препятствия
            if distance < self.d_max:
                # Модифицированный закон Кулона: сила ~ 1/r^2
                # Полный вектор: F = K_rep * (r_vec) / |r_vec|^3
                # то есть обратно пропорционально квадрату расстояния
                if distance > 0.01:
                    # Величина силы
                    magnitude = self.K_rep / (distance * distance)
                    # Нормализованный вектор от препятствия к роботу
                    nx = dx / distance
                    ny = dy / distance
                    
                    # Линейное затухание на границе зоны действия
                    if distance > self.safe_distance:
                        damping = (self.d_max - distance) / (self.d_max - self.safe_distance)
                        magnitude *= damping
                    
                    total_fx += nx * magnitude
                    total_fy += ny * magnitude
        
        return total_fx, total_fy
    
    def calculate_resultant_force(self):
        """
        Расчет результирующей силы как суммы сил притяжения и отталкивания.
        """
        fx_att, fy_att = self.calculate_attraction_force()
        fx_rep, fy_rep = self.calculate_repulsion_force()
        
        # Результирующая сила
        fx = fx_att + fx_rep
        fy = fy_att + fy_rep
        
        # Ограничение максимальной силы (для устойчивости)
        max_force = 10.0
        force_mag = math.hypot(fx, fy)
        if force_mag > max_force:
            fx = fx / force_mag * max_force
            fy = fy / force_mag * max_force
        
        return fx, fy
    
    def update(self, dt=1.0):
        """
        Обновление состояния робота под действием потенциалов.
        """
        # Расчёт результирующей силы
        fx, fy = self.calculate_resultant_force()
        
        # Применение силы к роботу
        self.robot.apply_force(fx, fy)
        
        # Обновление положения робота
        self.robot.update(dt)
        
        return fx, fy
    
    def is_target_reached(self, threshold=15.0):
        """Проверка достижения целевой точки."""
        return self.robot.distance_to_point(self.target.x, self.target.y) < threshold
    
    def is_collision_with_obstacles(self, robot_radius=10):
        """Проверка столкновения с препятствиями."""
        for obs in self.obstacles:
            if obs.is_collision(self.robot.x, self.robot.y, robot_radius):
                return True
        return False


def draw_force_vector(surface, robot, fx, fy, scale=20.0):
    """Отрисовка вектора силы для визуализации."""
    if math.hypot(fx, fy) > 0.01:
        end_x = robot.x + fx * scale
        end_y = robot.y + fy * scale
        pygame.draw.line(surface, ORANGE, (robot.x, robot.y), (end_x, end_y), 2)


def draw_influence_radius(surface, obstacles, d_max):
    """Отрисовка зоны влияния препятствий."""
    for obs in obstacles:
        surface.set_alpha(30)
        pygame.draw.circle(surface, (255, 100, 100, 50), 
                          (int(obs.x), int(obs.y)), int(d_max), 1)
        surface.set_alpha(255)


def main():
    # === Настройка начальных параметров ===
    # Начальные координаты робота (по заданию: x=100, y=100)
    robot = Robot(x=100, y=100)
    
    # Целевая точка (по заданию: x=500, y=500)
    target = Target(x=500, y=500)
    
    # Генерация 5 случайных препятствий
    obstacles = []
    num_obstacles = 5
    obstacle_radius = 30
    
    for _ in range(num_obstacles):
        # Генерация случайных координат с отступом от границ и от цели
        while True:
            x = random.randint(50, WIDTH - 50)
            y = random.randint(50, HEIGHT - 50)
            # Проверка, что препятствие не перекрывает старт и цель
            if math.hypot(x - robot.x, y - robot.y) > obstacle_radius + 20:
                if math.hypot(x - target.x, y - target.y) > obstacle_radius + 20:
                    # Проверка, что препятствия не пересекаются
                    overlap = False
                    for obs in obstacles:
                        if math.hypot(obs.x - x, obs.y - y) < obstacle_radius * 2:
                            overlap = True
                            break
                    if not overlap:
                        break
        obstacles.append(Obstacle(x, y, obstacle_radius))
    
    # Планировщик траектории
    planner = PotentialFieldPlanner(robot, target, obstacles)
    
    # Параметры отображения
    font = pygame.font.SysFont('monospace', 14)
    font_large = pygame.font.SysFont('monospace', 16, bold=True)
    
    running = True
    paused = False
    show_forces = True
    show_influence = True
    show_trail = True
    
    # Счётчик кадров
    frame = 0
    
    # Статус движения
    simulation_finished = False
    
    print("=" * 60)
    print("МОДЕЛИРОВАНИЕ МЕТОДА ПОТЕНЦИАЛОВ ЗАПУЩЕНО")
    print(f"Начальное положение робота: ({robot.x}, {robot.y})")
    print(f"Целевая точка: ({target.x}, {target.y})")
    print(f"Количество препятствий: {len(obstacles)}")
    print("=" * 60)
    print("Управление: ПРОБЕЛ - пауза, F - показать/скрыть силы, I - показать зоны влияния")
    print("=" * 60)
    
    while running:
        dt = 1.0  # шаг интегрирования (1 кадр = 1 единица времени)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_f:
                    show_forces = not show_forces
                elif event.key == pygame.K_i:
                    show_influence = not show_influence
                elif event.key == pygame.K_r:
                    # Сброс моделирования
                    robot = Robot(x=100, y=100)
                    planner.robot = robot
                    simulation_finished = False
                    print("Моделирование сброшено")
        
        if not paused and not simulation_finished:
            # Расчет сил и обновление робота
            fx, fy = planner.update(dt)
            
            # Проверка достижения цели
            if planner.is_target_reached(threshold=20.0):
                simulation_finished = True
                print(f"\n*** ЦЕЛЬ ДОСТИГНУТА! ***")
                print(f"Финальное положение робота: ({int(robot.x)}, {int(robot.y)})")
                print(f"Длина траектории: {len(robot.trail)} шагов")
            
            # Проверка столкновения
            if planner.is_collision_with_obstacles():
                print(f"ПРЕДУПРЕЖДЕНИЕ: Столкновение с препятствием на кадре {frame}!")
                # Не останавливаем симуляцию, просто предупреждаем
            
            frame += 1
            
            # Вывод информации в консоль каждые 60 кадров
            if frame % 60 == 0:
                dist_to_target = robot.distance_to_point(target.x, target.y)
                print(f"[Кадр {frame:4d}] Робот: ({int(robot.x):3d},{int(robot.y):3d}) | "
                      f"Расстояние до цели: {dist_to_target:.1f} px | "
                      f"Скорость: {math.hypot(robot.vx, robot.vy):.1f}")
        
        # === Отрисовка ===
        screen.fill(WHITE)
        
        # Сетка (для удобства)
        for x in range(0, WIDTH, 100):
            pygame.draw.line(screen, (220, 220, 220), (x, 0), (x, HEIGHT), 1)
        for y in range(0, HEIGHT, 100):
            pygame.draw.line(screen, (220, 220, 220), (0, y), (WIDTH, y), 1)
        
        # Зоны влияния препятствий
        if show_influence:
            draw_influence_radius(screen, obstacles, planner.d_max)
        
        # Препятствия
        for obs in obstacles:
            obs.draw(screen)
        
        # Цель
        target.draw(screen)
        
        # Робот и его траектория
        robot.draw(screen)
        
        # Вектор силы (если включено и не на паузе)
        if show_forces and not paused:
            fx, fy = planner.calculate_resultant_force()
            draw_force_vector(screen, robot, fx, fy, scale=25.0)
            
            # Также отрисовка силы притяжения и отталкивания для наглядности
            fx_att, fy_att = planner.calculate_attraction_force()
            fx_rep, fy_rep = planner.calculate_repulsion_force()
            draw_force_vector(screen, robot, fx_att, fy_att, scale=20.0)
            draw_force_vector(screen, robot, fx_rep, fy_rep, scale=15.0)
        
        # === Текстовая информация ===
        dist_to_target = robot.distance_to_point(target.x, target.y)
        
        # Состояние моделирования
        status = "ДОСТИГНУТА!" if simulation_finished else "ПАУЗА" if paused else "ДВИЖЕНИЕ"
        status_color = GREEN if simulation_finished else ORANGE if paused else BLUE
        
        texts = [
            f"МЕТОД ПОТЕНЦИАЛОВ - ПЛАНИРОВАНИЕ ДВИЖЕНИЯ",
            f"Робот: ({int(robot.x)}, {int(robot.y)})",
            f"Цель: ({target.x}, {target.y})",
            f"Управление: ПРОБЕЛ - пауза | F - силы | I - зоны влияния | R - сброс"
        ]
        
        for i, text in enumerate(texts):
            color = BLACK
            if "ЦЕЛЬ ДОСТИГНУТА" in text:
                color = GREEN
            elif "ДОСТИГНУТА" in text and i == 5:
                color = GREEN
            elif "ПАУЗА" in text and i == 5:
                color = ORANGE
            elif i == 0:
                color = PURPLE
            surf = font_large if i == 0 else font
            rendered = surf.render(text, True, color)
            screen.blit(rendered, (10, 10 + i * 22))
        
        # Отображение параметров потенциалов
        param_texts = [
            f"K_att (притяжение) = {planner.K_att}",
            f"K_rep (отталкивание) = {planner.K_rep}",
            f"d_max = {planner.d_max} px",
            f"Расстояние до цели: {dist_to_target:.1f} px",
            f"Скорость: {math.hypot(robot.vx, robot.vy):.1f} px/кадр",
            f"Статус: {status}",
            f"Препятствий: {len(obstacles)}",
            f""
        ]
        for i, text in enumerate(param_texts):
            rendered = font.render(text, True, GRAY)
            screen.blit(rendered, (WIDTH - 200, 10 + i * 20))
        
        # Обновление дисплея
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()