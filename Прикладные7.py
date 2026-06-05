import pygame
import random
import math
import sys

# ========== 1. Инициализация PyGame ==========
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Моделирование картографирования: фильтр частиц")
clock = pygame.time.Clock()
FPS = 60

# Цвета
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
ORANGE = (255, 165, 0)
GRAY = (200, 200, 200)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)
LIGHT_GREEN = (150, 255, 150)

class Robot:
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.trail = []

    def update(self, dt=1.0):
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.trail.append((self.x, self.y))
        if len(self.trail) > 500:
            self.trail.pop(0)

    def distance_to(self, landmark):
        return math.hypot(self.x - landmark.x, self.y - landmark.y)

    def draw(self, surface):
        if len(self.trail) > 1:
            points = [(int(x), int(y)) for x, y in self.trail]
            pygame.draw.lines(surface, GRAY, False, points, 2)
        pygame.draw.circle(surface, BLUE, (int(self.x), int(self.y)), 10)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), 10, 2)


class Landmark:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def draw(self, surface):
        pygame.draw.circle(surface, RED, (int(self.x), int(self.y)), 12)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), 12, 3)


class Particle:
    def __init__(self, x, y, weight=1.0):
        self.x = x
        self.y = y
        self.weight = weight

    def draw(self, surface):
        intensity = min(200, int(50 + 150 * self.weight))
        size = 3
        color = (0, intensity, 0)
        pygame.draw.circle(surface, color, (int(self.x), int(self.y)), size)


class ParticleFilter:
    def __init__(self, num_particles, world_bounds=(WIDTH, HEIGHT)):
        self.num_particles = num_particles
        self.particles = []
        self.world_bounds = world_bounds

    def generate_particles_uniform(self):
        """Равномерная генерация частиц по всей карте (лучше для глобальной сходимости)."""
        self.particles = []
        for _ in range(self.num_particles):
            x = random.uniform(50, self.world_bounds[0] - 50)
            y = random.uniform(50, self.world_bounds[1] - 50)
            self.particles.append(Particle(x, y, 1.0 / self.num_particles))

    def move_particles_with_robot(self, robot_dx, robot_dy, process_noise=15.0):
        """Смещение частиц с существенным шумом для исследования пространства."""
        for p in self.particles:
            p.x += robot_dx + random.gauss(0, process_noise)
            p.y += robot_dy + random.gauss(0, process_noise)
            p.x = max(10, min(self.world_bounds[0] - 10, p.x))
            p.y = max(10, min(self.world_bounds[1] - 10, p.y))

    def evaluate_weights(self, robot, measured_distance, sigma=50.0):
        """Оценка весов на основе правдоподобия."""
        for p in self.particles:
            predicted_dist = math.hypot(robot.x - p.x, robot.y - p.y)
            diff = predicted_dist - measured_distance
            likelihood = math.exp(-(diff * diff) / (2 * sigma * sigma))
            p.weight = likelihood + 1e-8  # добавляем малый вес для выживания
        
        total = sum(p.weight for p in self.particles)
        if total > 0:
            for p in self.particles:
                p.weight /= total

    def resample(self):
        """Стратифицированный ресемплинг с добавлением случайных частиц (10% для разнообразия)."""
        if not self.particles:
            return
        
        weights = [p.weight for p in self.particles]
        
        # Стратифицированная выборка
        new_particles = []
        step = 1.0 / self.num_particles
        
        for i in range(self.num_particles):
            r = random.uniform(i * step, (i + 1) * step)
            cumulative = 0
            for j, p in enumerate(self.particles):
                cumulative += weights[j]
                if r < cumulative:
                    # Копируем частицу с небольшим шумом
                    new_x = p.x + random.gauss(0, 8)
                    new_y = p.y + random.gauss(0, 8)
                    new_x = max(10, min(self.world_bounds[0] - 10, new_x))
                    new_y = max(10, min(self.world_bounds[1] - 10, new_y))
                    new_particles.append(Particle(new_x, new_y, 1.0 / self.num_particles))
                    break
        
        # Добавляем 10% случайных частиц для предотвращения вырождения
        num_random = int(self.num_particles * 0.1)
        for _ in range(num_random):
            x = random.uniform(50, self.world_bounds[0] - 50)
            y = random.uniform(50, self.world_bounds[1] - 50)
            new_particles[random.randint(0, len(new_particles)-1)] = Particle(x, y, 1.0 / self.num_particles)
        
        self.particles = new_particles

    def estimate_position(self):
        """Оценка как среднее с отсечением выбросов."""
        # Берём 50% лучших частиц для устойчивости
        sorted_particles = sorted(self.particles, key=lambda p: p.weight, reverse=True)
        top_n = max(1, int(self.num_particles * 0.5))
        
        sum_x = sum(p.x for p in sorted_particles[:top_n])
        sum_y = sum(p.y for p in sorted_particles[:top_n])
        
        return sum_x / top_n, sum_y / top_n

    def draw(self, surface):
        for p in self.particles:
            p.draw(surface)


def noisy_distance_measurement(robot, landmark, noise_std=25.0):
    true_dist = robot.distance_to(landmark)
    noise = random.gauss(0, noise_std)
    return max(0.0, true_dist + noise)


def main():
    robot = Robot(x=100, y=100, vx=0, vy=0)
    landmark = Landmark(x=400, y=300)
    
    # Маршрут, который ОБЯЗАТЕЛЬНО пройдёт вокруг ориентира со всех сторон
    waypoints = [
        (100, 100),    # старт (северо-запад)
        (700, 100),    # северо-восток
        (700, 500),    # юго-восток
        (100, 500),    # юго-запад
        (100, 100),    # обратно
        (400, 80),     # над ориентиром
        (400, 520),    # под ориентиром
        (80, 300),     # слева от ориентира
        (720, 300),    # справа от ориентира
        (400, 300),    # прямо к ориентиру (для финальной точности)
    ]
    
    pf = ParticleFilter(num_particles=500)  # больше частиц = лучше
    pf.generate_particles_uniform()  # равномерно по всей карте
    
    current_wp = 1
    speed = 5.0
    waypoint_threshold = 20
    
    frame = 0
    measurement_interval = 10  # чаще измерения
    
    font = pygame.font.SysFont('monospace', 14)
    font_large = pygame.font.SysFont('monospace', 16, bold=True)
    
    running = True
    paused = False
    
    print("=" * 50)
    print("МОДЕЛИРОВАНИЕ ЗАПУЩЕНО")
    print(f"Истинный ориентир: ({landmark.x}, {landmark.y})")
    print("Фильтр частиц: 500 частиц, равномерная инициализация")
    print("=" * 50)
    
    error_history = []
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    pf.generate_particles_uniform()
                    robot = Robot(x=100, y=100, vx=0, vy=0)
                    current_wp = 1
                    error_history = []
                    print("СБРОС: частицы перегенерированы равномерно")
        
        if not paused:
            # Движение робота
            dx = waypoints[current_wp][0] - robot.x
            dy = waypoints[current_wp][1] - robot.y
            dist_to_wp = math.hypot(dx, dy)
            
            if dist_to_wp < waypoint_threshold:
                current_wp = (current_wp + 1) % len(waypoints)
                if current_wp == 0 and len(waypoints) > 1:
                    current_wp = 1
            else:
                if dist_to_wp > 0:
                    robot.vx = dx / dist_to_wp * speed
                    robot.vy = dy / dist_to_wp * speed
                robot.update(1.0)
            
            # Фильтр частиц
            pf.move_particles_with_robot(robot.vx * 1.0, robot.vy * 1.0, process_noise=12.0)
            
            if frame % measurement_interval == 0:
                meas_dist = noisy_distance_measurement(robot, landmark, noise_std=30.0)
                pf.evaluate_weights(robot, meas_dist, sigma=60.0)
                pf.resample()
            
            est_x, est_y = pf.estimate_position()
            error = math.hypot(est_x - landmark.x, est_y - landmark.y)
            error_history.append(error)
            if len(error_history) > 100:
                error_history.pop(0)
            
            frame += 1
            
            # Вывод в консоль каждые 30 кадров
            if frame % 30 == 0:
                avg_error = sum(error_history[-30:]) / min(30, len(error_history))
                print(f"[Кадр {frame:4d}] Робот: ({int(robot.x):3d},{int(robot.y):3d}) | "
                      f"Оценка: ({int(est_x):3d},{int(est_y):3d}) | "
                      f"Ошибка: {error:5.1f} px | Средняя: {avg_error:5.1f} px")
        
        # Отрисовка
        screen.fill(WHITE)
        
        # Сетка
        for x in range(0, WIDTH, 100):
            pygame.draw.line(screen, (230, 230, 230), (x, 0), (x, HEIGHT), 1)
        for y in range(0, HEIGHT, 100):
            pygame.draw.line(screen, (230, 230, 230), (0, y), (WIDTH, y), 1)
        
        pf.draw(screen)
        landmark.draw(screen)
        
        est_x, est_y = pf.estimate_position()
        
        # Оценка фильтра
        pygame.draw.circle(screen, ORANGE, (int(est_x), int(est_y)), 14)
        pygame.draw.circle(screen, BLACK, (int(est_x), int(est_y)), 14, 2)
        
        # Линия ошибки
        pygame.draw.line(screen, YELLOW, (est_x, est_y), (landmark.x, landmark.y), 2)
        
        robot.draw(screen)
        
        # Текст
        error = math.hypot(est_x - landmark.x, est_y - landmark.y)
        texts = [
            f"РОБОТ: ({int(robot.x)}, {int(robot.y)})",
            f"ИСТИННЫЙ ОРИЕНТИР: ({landmark.x}, {landmark.y})",
            f"ОЦЕНКА ФИЛЬТРА: ({int(est_x)}, {int(est_y)})",
            f"ОШИБКА ОЦЕНКИ: {error:.1f} px",
            f"ЧАСТИЦ: {pf.num_particles}",
            f"СТАТУС: {'ПАУЗА' if paused else 'РАБОТА'}"
        ]
        
        for i, text in enumerate(texts):
            color = RED if i == 1 else ORANGE if i == 2 else BLACK
            if i == 3:
                if error < 30:
                    color = GREEN
                elif error < 80:
                    color = ORANGE
                else:
                    color = RED
            surf = font_large if i == 2 else font
            screen.blit(surf.render(text, True, color), (10, 10 + i * 22))
        
        # График ошибки
        if len(error_history) > 1:
            graph_x = WIDTH - 210
            graph_y = 10
            graph_w = 200
            graph_h = 60
            pygame.draw.rect(screen, (240, 240, 240), (graph_x, graph_y, graph_w, graph_h))
            pygame.draw.rect(screen, BLACK, (graph_x, graph_y, graph_w, graph_h), 1)
            
            max_error = max(error_history[-graph_w:]) if error_history else 1
            if max_error < 1:
                max_error = 1
            
            for i, err in enumerate(error_history[-graph_w:]):
                bar_height = min(graph_h - 2, int((err / max_error) * (graph_h - 2)))
                bar_color = GREEN if err < 30 else ORANGE if err < 80 else RED
                pygame.draw.rect(screen, bar_color, 
                               (graph_x + i + 1, graph_y + graph_h - bar_height - 1, 1, bar_height))
            
            err_text = font.render(f"Ошибка: {error:.0f}", True, BLACK)
            screen.blit(err_text, (graph_x + 5, graph_y + 5))
        
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()