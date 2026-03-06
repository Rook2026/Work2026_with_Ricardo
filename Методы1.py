import pygame
import numpy as np
import math
import random

# Константы
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
BACKGROUND_COLOR = (255, 255, 255)
ROBOT_COLOR = (0, 0, 255)
LANDMARK_COLOR = (255, 0, 0)
PARTICLE_COLOR = (0, 255, 0)
ESTIMATE_COLOR = (0, 255, 255)
TRUE_PATH_COLOR = (100, 100, 100)
GOAL_COLOR = (255, 255, 0)

# Параметры фильтра
NUM_PARTICLES = 500
SENSOR_NOISE = 5.0  # Погрешность измерения расстояния
MOVEMENT_NOISE = 2.0  # Погрешность движения
ARRIVAL_THRESHOLD = 3.0  # Порог прибытия (пиксели)

class Robot:
    """Класс реального робота"""
    def __init__(self, x, y, alpha):
        self.x = x
        self.y = y
        self.alpha = alpha  # Угол ориентации в градусах
        self.arrived = False
        
    def move(self, distance, turn):
        """Перемещение робота с учетом шума"""
        if self.arrived:
            return
            
        # Добавляем шум к движению
        distance += random.gauss(0, MOVEMENT_NOISE)
        turn += random.gauss(0, MOVEMENT_NOISE)
        
        # Обновляем угол
        self.alpha += turn
        
        # Обновляем координаты
        self.x += distance * math.cos(math.radians(self.alpha))
        self.y += distance * math.sin(math.radians(self.alpha))
        
    def measure_distance(self, landmark):
        """Измерение расстояния до ориентира с погрешностью"""
        true_distance = math.sqrt((self.x - landmark.x)**2 + (self.y - landmark.y)**2)
        # Добавляем гауссов шум
        measured_distance = true_distance + random.gauss(0, SENSOR_NOISE)
        return max(0, measured_distance)  # Расстояние не может быть отрицательным
    
    def distance_to(self, target_x, target_y):
        """Расстояние до целевой точки"""
        return math.sqrt((self.x - target_x)**2 + (self.y - target_y)**2)

class Landmark:
    """Класс ориентира на местности"""
    def __init__(self, x, y, name=""):
        self.x = x
        self.y = y
        self.name = name

class Particle:
    """Класс частицы (предполагаемое положение робота)"""
    def __init__(self, x, y, alpha):
        self.x = x
        self.y = y
        self.alpha = alpha
        self.weight = 1.0  # Вес частицы (достоверность)
        
    def move(self, distance, turn):
        """Перемещение частицы с шумом"""
        # Добавляем больший шум для частиц
        distance += random.gauss(0, MOVEMENT_NOISE * 1.5)
        turn += random.gauss(0, MOVEMENT_NOISE * 1.5)
        
        self.alpha += turn
        self.x += distance * math.cos(math.radians(self.alpha))
        self.y += distance * math.sin(math.radians(self.alpha))
        
    def expected_distance(self, landmark):
        """Ожидаемое расстояние до ориентира"""
        return math.sqrt((self.x - landmark.x)**2 + (self.y - landmark.y)**2)

class ParticleFilter:
    """Класс фильтра частиц"""
    def __init__(self, num_particles, initial_x, initial_y, initial_alpha):
        self.particles = []
        self.num_particles = num_particles
        self.initialize_particles(initial_x, initial_y, initial_alpha)
        
    def initialize_particles(self, x, y, alpha):
        """Функция генерации начального набора частиц"""
        self.particles = []
        for _ in range(self.num_particles):
            # Добавляем случайный разброс вокруг начального положения
            px = x + random.gauss(0, 50)
            py = y + random.gauss(0, 50)
            pa = alpha + random.gauss(0, 30)  # Разброс угла
            self.particles.append(Particle(px, py, pa))
            
    def update_weights(self, landmarks, measurements):
        """Оценка достоверности частиц на основе измерений"""
        for particle in self.particles:
            particle.weight = 1.0
            # Сравниваем с каждым ориентиром
            for i, landmark in enumerate(landmarks):
                expected_dist = particle.expected_distance(landmark)
                measured_dist = measurements[i]
                
                # Вероятность по гауссовому распределению
                diff = expected_dist - measured_dist
                likelihood = math.exp(-(diff**2) / (2 * SENSOR_NOISE**2))
                particle.weight *= likelihood
            
            # Добавляем небольшой вес для избежания вырождения
            particle.weight += 1e-6
            
    def resample(self):
        """Регенерация частиц вблизи лучших положений"""
        new_particles = []
        
        # Нормализация весов
        weights = [p.weight for p in self.particles]
        total_weight = sum(weights)
        if total_weight > 0:
            weights = [w / total_weight for w in weights]
        else:
            weights = [1.0 / self.num_particles] * self.num_particles
        
        # Ресемплинг методом колеса рулетки
        for _ in range(self.num_particles):
            # Выбираем случайную частицу с учетом весов
            index = random.choices(range(self.num_particles), weights=weights)[0]
            selected = self.particles[index]
            
            # Создаем новую частицу с небольшим шумом (регенерация вблизи лучших)
            new_particle = Particle(
                selected.x + random.gauss(0, 10),
                selected.y + random.gauss(0, 10),
                selected.alpha + random.gauss(0, 5)
            )
            new_particles.append(new_particle)
            
        self.particles = new_particles
        
    def move_particles(self, distance, turn):
        """Смещение частиц сонаправленно с движением робота"""
        for particle in self.particles:
            particle.move(distance, turn)
            
    def estimate_position(self):
        """Оценка положения робота (среднее по частицам)"""
        if not self.particles:
            return (0, 0)
        avg_x = sum(p.x for p in self.particles) / len(self.particles)
        avg_y = sum(p.y for p in self.particles) / len(self.particles)
        return (avg_x, avg_y)
    
    def get_covariance(self):
        """Вычисление ковариации (разброса частиц)"""
        if len(self.particles) < 2:
            return 0
        est_x, est_y = self.estimate_position()
        variance = sum((p.x - est_x)**2 + (p.y - est_y)**2 for p in self.particles) / len(self.particles)
        return math.sqrt(variance)  # Среднеквадратичное отклонение

class Simulation:
    """Основной класс симуляции"""
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Локализация робота фильтром частиц")
        self.clock = pygame.time.Clock()
        self.running = True
        self.font = pygame.font.Font(None, 24)
        self.big_font = pygame.font.Font(None, 36)
        
        # Инициализация объектов
        self.robot = Robot(100, 100, 45)  # Начальные координаты x=100, y=100, угол 45°
        self.landmarks = [
            Landmark(200, 500, "R1"),  # Ориентир 1
            Landmark(400, 300, "R2")   # Ориентир 2
        ]
        self.pf = ParticleFilter(NUM_PARTICLES, 100, 100, 45)
        
        # Траектория движения (для теста)
        self.path_points = [(100, 100)]
        self.target = (500, 500)  # Конечные координаты
        
        # Шаг движения
        self.move_step = 0
        self.max_steps = 300  # Увеличим количество шагов
        
        # Статистика
        self.arrival_time = None
        self.arrival_position = None
        
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    # Перезапуск симуляции
                    self.__init__()
                elif event.key == pygame.K_ESCAPE:
                    self.running = False
                
    def update(self):
        # Проверка прибытия
        dist_to_target = self.robot.distance_to(self.target[0], self.target[1])
        
        if dist_to_target < ARRIVAL_THRESHOLD and not self.robot.arrived:
            self.robot.arrived = True
            self.arrival_time = self.move_step
            self.arrival_position = (self.robot.x, self.robot.y)
            print(f"\n Робот прибыл к цели!")
            print(f"   Шагов: {self.move_step}")
            print(f"   Позиция: ({self.robot.x:.2f}, {self.robot.y:.2f})")
            print(f"   Ошибка: {dist_to_target:.2f} пикселей")
        
        # Плановое движение робота к цели
        if self.move_step < self.max_steps and not self.robot.arrived:
            # Вычисляем направление к цели
            dx = self.target[0] - self.robot.x
            dy = self.target[1] - self.robot.y
            
            # Адаптивная скорость: замедление при приближении
            distance_to_target = math.sqrt(dx**2 + dy**2)
            speed_factor = min(1.0, distance_to_target / 50.0)
            
            target_angle = math.degrees(math.atan2(dy, dx))
            
            # Поворачиваем к цели
            angle_diff = target_angle - self.robot.alpha
            # Нормализация угла
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
            
            turn = angle_diff * 0.1  # Плавный поворот
            # Не превышаем расстояние до цели
            distance = min(3.0 * speed_factor, distance_to_target)
            
            # Двигаем реального робота
            self.robot.move(distance, turn)
            
            # Двигаем частицы так же, как робота
            self.pf.move_particles(distance, turn)
            
            # Выполняем измерения от робота до ориентиров
            measurements = [self.robot.measure_distance(lm) for lm in self.landmarks]
            
            # Обновляем веса частиц
            self.pf.update_weights(self.landmarks, measurements)
            
            # Ресемплинг
            self.pf.resample()
            
            # Сохраняем точку траектории
            self.path_points.append((self.robot.x, self.robot.y))
            
            self.move_step += 1
            
    def draw(self):
        self.screen.fill(BACKGROUND_COLOR)
        
        # Отрисовка траектории
        if len(self.path_points) > 1:
            pygame.draw.lines(self.screen, TRUE_PATH_COLOR, False, self.path_points, 2)
        
        # Отрисовка частиц
        for particle in self.pf.particles:
            pygame.draw.circle(self.screen, PARTICLE_COLOR, 
                             (int(particle.x), int(particle.y)), 2)
        
        # Отрисовка ориентиров
        for lm in self.landmarks:
            pygame.draw.circle(self.screen, LANDMARK_COLOR, (int(lm.x), int(lm.y)), 10)
            pygame.draw.circle(self.screen, (0, 0, 0), (int(lm.x), int(lm.y)), 10, 2)
            
            # Подпись ориентира
            text = self.font.render(f"{lm.name}({lm.x}, {lm.y})", True, LANDMARK_COLOR)
            self.screen.blit(text, (lm.x + 15, lm.y - 10))
        
        # Отрисовка цели
        pygame.draw.circle(self.screen, GOAL_COLOR, self.target, 12, 3)
        goal_text = self.font.render("ЦЕЛЬ", True, GOAL_COLOR)
        self.screen.blit(goal_text, (self.target[0] + 20, self.target[1] - 20))
        
        # Отрисовка робота (истинное положение)
        if self.robot.arrived:
            # Робот прибыл - рисуем с ореолом
            pygame.draw.circle(self.screen, GOAL_COLOR, 
                             (int(self.robot.x), int(self.robot.y)), 12, 2)
        
        pygame.draw.circle(self.screen, ROBOT_COLOR, 
                         (int(self.robot.x), int(self.robot.y)), 8)
        pygame.draw.circle(self.screen, (0, 0, 0), 
                         (int(self.robot.x), int(self.robot.y)), 8, 2)
        
        # Направление робота
        end_x = self.robot.x + 20 * math.cos(math.radians(self.robot.alpha))
        end_y = self.robot.y + 20 * math.sin(math.radians(self.robot.alpha))
        pygame.draw.line(self.screen, ROBOT_COLOR, 
                        (int(self.robot.x), int(self.robot.y)), 
                        (int(end_x), int(end_y)), 3)
        
        # Оцененное положение
        est_x, est_y = self.pf.estimate_position()
        pygame.draw.circle(self.screen, ESTIMATE_COLOR, (int(est_x), int(est_y)), 5, 2)
        
        # Линия от оценки к реальному положению (ошибка)
        pygame.draw.line(self.screen, (255, 165, 0), 
                        (int(est_x), int(est_y)), 
                        (int(self.robot.x), int(self.robot.y)), 1)
        
        # Текстовая информация
        info_y = 100
        
        # Позиция робота
        info1 = self.font.render(f"Робот: ({self.robot.x:.1f}, {self.robot.y:.1f})", 
                                True, (0, 0, 0))
        self.screen.blit(info1, (550, info_y))
        info_y += 25
        
        # Оценка
        error = math.sqrt((self.robot.x - est_x)**2 + (self.robot.y - est_y)**2)
        info2 = self.font.render(f"Оценка: ({est_x:.1f}, {est_y:.1f}) [ош:{error:.1f}]", 
                                True, (0, 0, 0))
        self.screen.blit(info2, (550, info_y))
        info_y += 25
        
        # Частицы и разброс
        spread = self.pf.get_covariance()
        info3 = self.font.render(f"Частиц: {len(self.pf.particles)}  Разброс: {spread:.1f}", 
                                True, (0, 0, 0))
        self.screen.blit(info3, (550, info_y))
        info_y += 25
        
        # Расстояние до цели
        dist_to_target = self.robot.distance_to(self.target[0], self.target[1])
        info4 = self.font.render(f"До цели: {dist_to_target:.1f} пикс.", 
                                True, (0, 0, 0))
        self.screen.blit(info4, (550, info_y))
        info_y += 25
        
        # Шаг
        info5 = self.font.render(f"Шаг: {self.move_step}/{self.max_steps}", 
                                True, (0, 0, 0))
        self.screen.blit(info5, (550, info_y))
        info_y += 25
        
        # Статус прибытия
        if self.robot.arrived:
            arrived_text = self.big_font.render("ЦЕЛЬ ДОСТИГНУТА", True, (100, 250, 100))
            text_rect = arrived_text.get_rect(center=(WINDOW_WIDTH//6, 30))
            self.screen.blit(arrived_text, text_rect)
        
        # Инструкции
        inst1 = self.font.render("ПРОБЕЛ: перезапуск", True, (100, 100, 100))
        inst2 = self.font.render("ESC: выход", True, (100, 100, 100))
        self.screen.blit(inst1, (WINDOW_WIDTH - 250, 10))
        self.screen.blit(inst2, (WINDOW_WIDTH - 250, 35))
        
        pygame.display.flip()
        
    def run(self):
        while self.running:
            self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(60)  # 60 FPS
        
        # Итоговая статистика
        if self.arrival_time:
            print("\n" + "="*50)
            print("ИТОГОВАЯ СТАТИСТИКА:")
            print(f"Время прибытия: {self.arrival_time} шагов")
            print(f"Финальная позиция: ({self.robot.x:.2f}, {self.robot.y:.2f})")
            target_dist = self.robot.distance_to(self.target[0], self.target[1])
            print(f"Точность: {target_dist:.2f} пикселей")
            print("="*50)
        
        pygame.quit()

# Запуск симуляции
if __name__ == "__main__":
    print("="*50)
    print("ЛОКАЛИЗАЦИЯ РОБОТА ФИЛЬТРОМ ЧАСТИЦ")
    print("="*50)
    print(f"Начальная позиция: (100, 100)")
    print(f"Ориентиры: (200, 500) и (400, 300)")
    print(f"Целевая позиция: (500, 500)")
    print(f"Количество частиц: {NUM_PARTICLES}")
    print("-"*50)
    print("УПРАВЛЕНИЕ:")
    print("ПРОБЕЛ - перезапуск")
    print("ESC - выход")
    print("="*50)
    
    sim = Simulation()
    sim.run()