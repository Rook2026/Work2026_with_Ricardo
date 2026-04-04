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
ROBOT_TRAIL_COLOR = (0, 100, 0)
LANDMARK_TRUE_COLOR = (255, 0, 0)      # истинное положение ориентира
LANDMARK_EST_COLOR = (0, 0, 255)       # оцененное положение ориентира
PARTICLE_COLOR = (200, 200, 200)
TARGET_COLOR = (255, 165, 0)           # целевая точка

# Параметры робота
ROBOT_RADIUS = 8
LANDMARK_RADIUS = 8
PARTICLE_RADIUS = 2

# Параметры фильтра частиц
NUM_PARTICLES = 200
INITIAL_PARTICLE_SPREAD = 100          # начальный разброс частиц (пикселей)
MEASUREMENT_NOISE = 15.0               # шум измерения дальности (пикселей)
MOTION_NOISE = 3.0                     # шум движения (пикселей)
RESAMPLE_THRESHOLD = 0.7               # порог репопуляции (доля эффективных частиц)

# Параметры движения
ROBOT_SPEED = 3.0
TARGET_POS = (700, 100)                # конечная точка движения робота

# ========== 1. Класс мобильного робота (Robot) ==========
class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.trail = [(x, y)]           # траектория движения

    def update(self, target_x, target_y):
        """Движение робота к целевой точке с шумом"""
        # Вычисляем направление к цели
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        if dist > 1:
            # Нормализованное направление
            self.vx = (dx / dist) * ROBOT_SPEED
            self.vy = (dy / dist) * ROBOT_SPEED

            # Добавляем шум движения
            self.vx += random.gauss(0, MOTION_NOISE * 0.3)
            self.vy += random.gauss(0, MOTION_NOISE * 0.3)

            # Обновляем позицию
            self.x += self.vx
            self.y += self.vy

        # Ограничиваем границами экрана
        self.x = max(0, min(WIDTH, self.x))
        self.y = max(0, min(HEIGHT, self.y))

        # Добавляем точку в траекторию
        self.trail.append((self.x, self.y))
        if len(self.trail) > 500:
            self.trail.pop(0)

    def get_pos(self):
        return (int(self.x), int(self.y))

    def distance_to(self, other_x, other_y):
        return math.hypot(self.x - other_x, self.y - other_y)

# ========== 2. Класс ориентира (Landmark) ==========
class Landmark:
    def __init__(self, x, y):
        self.true_x = x
        self.true_y = y
        self.estimated_x = x
        self.estimated_y = y
        self.particle_filter = None

    def get_true_pos(self):
        return (int(self.true_x), int(self.true_y))

    def get_estimated_pos(self):
        return (int(self.estimated_x), int(self.estimated_y))

# ========== 3. Класс фильтра частиц (ParticleFilter) ==========
class ParticleFilter:
    def __init__(self, robot, num_particles=NUM_PARTICLES):
        self.particles = []          # список частиц [(x, y, weight), ...]
        self.num_particles = num_particles
        self.robot = robot

    def initialize_particles(self, initial_guess_x, initial_guess_y, spread=INITIAL_PARTICLE_SPREAD):
        """Генерация частиц в зоне предполагаемого расположения ориентира"""
        self.particles = []
        for _ in range(self.num_particles):
            x = initial_guess_x + random.gauss(0, spread)
            y = initial_guess_y + random.gauss(0, spread)
            # Ограничиваем границами экрана
            x = max(0, min(WIDTH, x))
            y = max(0, min(HEIGHT, y))
            self.particles.append([x, y, 1.0 / self.num_particles])  # [x, y, weight]

    def shift_particles(self):
        """Смещение частиц сонаправленно с движением робота"""
        # Частицы представляют положение ориентира в мире.
        # При движении робота ориентир остаётся неподвижным,
        # поэтому частицы НЕ должны смещаться.
        # Вместо этого измерение дальности меняется из-за движения робота.
        # Эта функция оставлена для совместимости с заданием,
        # но фактически частицы остаются на месте.
        pass

    def measure_distance(self, robot_x, robot_y, landmark_true_x, landmark_true_y):
        """Зашумленное измерение дальности от робота до ориентира"""
        true_distance = math.hypot(robot_x - landmark_true_x, robot_y - landmark_true_y)
        noisy_distance = true_distance + random.gauss(0, MEASUREMENT_NOISE)
        return max(0, noisy_distance)

    def update_weights(self, measured_distance):
        """Оценка частиц по критерию рассогласования прогнозируемых и фактических измерений"""
        for i, particle in enumerate(self.particles):
            # Прогнозируемая дальность от робота до частицы (гипотетический ориентир)
            predicted_distance = math.hypot(self.robot.x - particle[0],
                                            self.robot.y - particle[1])

            # Ошибка между измеренным и прогнозируемым расстоянием
            error = abs(predicted_distance - measured_distance)

            # Вес частицы (вероятность) - используем нормальное распределение
            # Чем меньше ошибка, тем больше вес
            likelihood = math.exp(-(error ** 2) / (2 * MEASUREMENT_NOISE ** 2))

            # Обновляем вес (с учётом предыдущего веса)
            particle[2] = particle[2] * likelihood

        # Нормализация весов (сумма весов = 1)
        total_weight = sum(p[2] for p in self.particles)
        if total_weight > 0:
            for particle in self.particles:
                particle[2] /= total_weight

    def resample_particles(self):
        """Репопуляция частиц с фильтрацией по величине оценок (систематическая выборка)"""
        # Вычисляем эффективное количество частиц
        weights = [p[2] for p in self.particles]
        eff = 1.0 / sum(w ** 2 for w in weights)

        # Если эффективное количество слишком мало, выполняем ресемплинг
        if eff < RESAMPLE_THRESHOLD * self.num_particles:
            new_particles = []

            # Систематическая выборка
            step = 1.0 / self.num_particles
            r = random.uniform(0, step)
            cumulative_weight = 0
            index = 0

            # Сортируем частицы для кумулятивной суммы (можно и без сортировки)
            # Создаём список кумулятивных весов
            cumulative_weights = []
            cum_sum = 0
            for w in weights:
                cum_sum += w
                cumulative_weights.append(cum_sum)

            for i in range(self.num_particles):
                target = r + i * step
                while index < len(cumulative_weights) and cumulative_weights[index] < target:
                    index += 1
                if index >= len(self.particles):
                    index = len(self.particles) - 1
                # Копируем частицу с небольшим шумом (диверсификация)
                px, py, _ = self.particles[index]
                px += random.gauss(0, MOTION_NOISE)
                py += random.gauss(0, MOTION_NOISE)
                px = max(0, min(WIDTH, px))
                py = max(0, min(HEIGHT, py))
                new_particles.append([px, py, 1.0 / self.num_particles])

            self.particles = new_particles

    def get_estimated_position(self):
        """Расчёт предполагаемого положения ориентира как геометрического центра набора гипотез"""
        if not self.particles:
            return (WIDTH // 2, HEIGHT // 2)

        # Взвешенное среднее (центр масс)
        total_weight = sum(p[2] for p in self.particles)
        if total_weight == 0:
            total_weight = 1

        est_x = sum(p[0] * p[2] for p in self.particles) / total_weight
        est_y = sum(p[1] * p[2] for p in self.particles) / total_weight

        return (int(est_x), int(est_y))

    def draw(self, screen):
        """Отрисовка частиц"""
        for particle in self.particles:
            # Интенсивность зависит от веса
            intensity = min(255, int(particle[2] * 500))
            color = (intensity, intensity, intensity)
            pygame.draw.circle(screen, color,
                             (int(particle[0]), int(particle[1])),
                             PARTICLE_RADIUS)

# ========== 4. Главная функция ==========
def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Particle Filter - Landmark Mapping")
    clock = pygame.time.Clock()

    # Входные параметры
    start_pos = (100, 100)      # начальные координаты робота
    target_pos = TARGET_POS     # конечные координаты робота (700, 100)
    landmark_true_pos = (400, 300)  # истинное положение ориентира

    # Создание объектов
    robot = Robot(start_pos[0], start_pos[1])
    landmark = Landmark(landmark_true_pos[0], landmark_true_pos[1])

    # Создание фильтра частиц
    particle_filter = ParticleFilter(robot, NUM_PARTICLES)

    # Инициализация частиц (начальное предположение - центр экрана с разбросом)
    initial_guess = (WIDTH // 2, HEIGHT // 2)
    particle_filter.initialize_particles(initial_guess[0], initial_guess[1],
                                         INITIAL_PARTICLE_SPREAD)

    # Переменные для отображения
    running = True
    iteration = 0
    font = pygame.font.Font(None, 20)

    # Основной цикл
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Движение робота к целевой точке
        robot.update(target_pos[0], target_pos[1])

        # Проверка достижения цели
        dist_to_target = math.hypot(robot.x - target_pos[0], robot.y - target_pos[1])

        # Если робот достиг цели, завершаем движение
        if dist_to_target < ROBOT_RADIUS * 2:
            # Останавливаем робота
            robot.vx = 0
            robot.vy = 0

        # Измерение дальности до ориентира (с шумом)
        measured_distance = particle_filter.measure_distance(
            robot.x, robot.y,
            landmark.true_x, landmark.true_y
        )

        # Обновление весов частиц
        particle_filter.update_weights(measured_distance)

        # Ресемплинг (репопуляция) частиц
        particle_filter.resample_particles()

        # Получение оценки положения ориентира
        est_x, est_y = particle_filter.get_estimated_position()
        landmark.estimated_x = est_x
        landmark.estimated_y = est_y

        # ========== ВИЗУАЛИЗАЦИЯ ==========
        screen.fill(BACKGROUND_COLOR)

        # Рисуем траекторию робота
        if len(robot.trail) > 1:
            for i in range(len(robot.trail) - 1):
                pygame.draw.line(screen, ROBOT_TRAIL_COLOR,
                                 (int(robot.trail[i][0]), int(robot.trail[i][1])),
                                 (int(robot.trail[i+1][0]), int(robot.trail[i+1][1])), 2)

        # Рисуем частицы
        particle_filter.draw(screen)

        # Рисуем истинное положение ориентира (красный)
        pygame.draw.circle(screen, LANDMARK_TRUE_COLOR,
                         landmark.get_true_pos(), LANDMARK_RADIUS)
        pygame.draw.circle(screen, (150, 0, 0),
                         landmark.get_true_pos(), LANDMARK_RADIUS - 2)

        # Рисуем оцененное положение ориентира (синий)
        pygame.draw.circle(screen, LANDMARK_EST_COLOR,
                         landmark.get_estimated_pos(), LANDMARK_RADIUS)
        pygame.draw.circle(screen, (0, 0, 150),
                         landmark.get_estimated_pos(), LANDMARK_RADIUS - 2)

        # Рисуем робота (зелёный)
        pygame.draw.circle(screen, ROBOT_COLOR, robot.get_pos(), ROBOT_RADIUS)
        pygame.draw.circle(screen, (0, 100, 0), robot.get_pos(), ROBOT_RADIUS - 2)

        # Рисуем целевую точку (оранжевый)
        pygame.draw.circle(screen, TARGET_COLOR, target_pos, 8)
        pygame.draw.circle(screen, (200, 100, 0), target_pos, 6)

        # Отображаем информацию
        error = math.hypot(landmark.estimated_x - landmark.true_x,
                          landmark.estimated_y - landmark.true_y)
        

        y_offset = 10
        for text in info_texts:
            surface = font.render(text, True, (0, 0, 0))
            screen.blit(surface, (10, y_offset))
            y_offset += 20

        pygame.display.flip()
        clock.tick(FPS)
        iteration += 1

        # Если робот достиг цели, можно остановить обновление, но продолжаем отображать
        if dist_to_target < ROBOT_RADIUS * 2 and iteration > 100:
            # Ждём 2 секунды для просмотра результата
            pygame.time.wait(2000)
            running = False

    # Финальная пауза для просмотра результата
    waiting = True
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                waiting = False
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()