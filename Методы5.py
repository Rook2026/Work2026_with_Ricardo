"""
Практическое задание: Моделирование обучаемого конечного автомата (автомат Цетлина)
для выбора режимов функционирования мобильного робота в среде с препятствиями.

Управление: 
  ESC - выход
  R - сброс обучения
  P - пауза/возобновление
  T - показать/скрыть траекторию

Автомат обучается на сигналах подкрепления:
  +1 за быстрое движение (поощрение)
  -1 за столкновение (штраф)
Вероятности переходов адаптируются по алгоритму Цетлина.
"""

import pygame
import numpy as np
import math
from collections import defaultdict

# ======================== 1. НАСТРОЙКА ОКНА ========================
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Обучаемый конечный автомат Цетлина – мобильный робот")
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
CYAN = (0, 255, 255)

# ======================== 2. СОСТОЯНИЯ РОБОТА (9 состояний) ========================
STATES = [
    "Остановка",
    "Движение вперед",
    "Движение влево",
    "Движение вправо",
    "Движение назад",
    "Быстрое движение вперед",
    "Быстрое движение влево",
    "Быстрое движение вправо",
    "Быстрое движение назад"
]

# Скорости для обычных и быстрых движений (пикселей/кадр)
NORMAL_SPEED = 2
FAST_SPEED = 6

# Отображение состояния -> (линейная скорость, угловая скорость, название для вывода)
STATE_INFO = {
    "Остановка": (0, 0, "Стоп"),
    "Движение вперед": (NORMAL_SPEED, 0, "Вперёд"),
    "Движение влево": (0, -3, "Влево"),
    "Движение вправо": (0, 3, "Вправо"),
    "Движение назад": (-NORMAL_SPEED, 0, "Назад"),
    "Быстрое движение вперед": (FAST_SPEED, 0, "Б.Вперёд"),
    "Быстрое движение влево": (0, -5, "Б.Влево"),
    "Быстрое движение вправо": (0, 5, "Б.Вправо"),
    "Быстрое движение назад": (-FAST_SPEED, 0, "Б.Назад")
}

# ======================== 3. КЛАСС СОСТОЯНИЯ АВТОМАТА ========================
class State:
    def __init__(self, name, position):
        self.name = name
        self.position = position
        self.activation = 0.0
        self.incoming = []
        self.outgoing = []
        self.probabilities = {}
        self.visit_count = 0

    def add_outgoing(self, target, prob):
        if target not in self.outgoing:
            self.outgoing.append(target)
            self.probabilities[target] = prob
            target.incoming.append(self)

    def normalize_probabilities(self):
        total = sum(self.probabilities.values())
        if total > 0:
            for s in self.probabilities:
                self.probabilities[s] /= total

    def choose_next_state(self):
        if not self.probabilities:
            return None
        states = list(self.probabilities.keys())
        probs = list(self.probabilities.values())
        return np.random.choice(states, p=probs)


# ======================== 4. КЛАСС КОНЕЧНОГО АВТОМАТА ЦЕТЛИНА ========================
class FSM_Cetlin:
    def __init__(self):
        self.states = {}
        self.current_state = None
        self.learning_rate = 0.1
        self._init_states()
        self._init_transitions()

    def _init_states(self):
        """Расположение 9 состояний: центр, внутренний круг (обычные), внешний круг (быстрые)"""
        cx, cy = WIDTH // 2, HEIGHT // 2
        r_normal = 180
        r_fast = 280

        # Углы для направлений (0° = вправо, 90° = вниз, 180° = влево, 270° = вверх)
        angles = {
            "вперед": 270,
            "влево": 180,
            "вправо": 0,
            "назад": 90
        }

        # Обычные движения
        for direction, angle in angles.items():
            rad = math.radians(angle)
            x = cx + r_normal * math.cos(rad)
            y = cy + r_normal * math.sin(rad)
            name = f"Движение {direction}"
            self.states[name] = State(name, (int(x), int(y)))

        # Быстрые движения
        for direction, angle in angles.items():
            rad = math.radians(angle)
            x = cx + r_fast * math.cos(rad)
            y = cy + r_fast * math.sin(rad)
            name = f"Быстрое движение {direction}"
            self.states[name] = State(name, (int(x), int(y)))

        # Остановка в центре
        self.states["Остановка"] = State("Остановка", (cx, cy))

    def _init_transitions(self):
        """Инициализация равновероятных переходов (полносвязный автомат)"""
        all_states = list(self.states.values())
        prob = 1.0 / len(all_states)
        for s in all_states:
            for t in all_states:
                s.add_outgoing(t, prob)
            s.normalize_probabilities()

    def set_current_state(self, state_name):
        self.current_state = self.states[state_name]

    def update_activation(self):
        """Обновление активации для визуализации (текущее состояние = 100%, остальные = 0)"""
        for s in self.states.values():
            s.activation = 0.0
        if self.current_state:
            self.current_state.activation = 1.0

    def transition(self, reward):
        """
        Переход в следующее состояние с адаптацией вероятностей по алгоритму Цетлина.
        reward: +1 (хорошо), -1 (плохо), 0 (нейтрально)
        """
        if self.current_state is None:
            return

        # 1. Выбор следующего состояния на основе текущих вероятностей
        next_state = self.current_state.choose_next_state()
        chosen = next_state

        # 2. Корректировка вероятностей перехода из текущего состояния
        if reward != 0:
            all_targets = list(self.current_state.probabilities.keys())
            p_chosen = self.current_state.probabilities[chosen]
            if reward > 0:
                # Положительное подкрепление: увеличиваем вероятность выбранного перехода
                delta = self.learning_rate * (1 - p_chosen)
                new_p = p_chosen + delta
                # Уменьшаем остальные пропорционально
                if (1 - p_chosen) > 0:
                    scale = (1 - new_p) / (1 - p_chosen)
                else:
                    scale = 0
                for s in all_targets:
                    if s == chosen:
                        self.current_state.probabilities[s] = new_p
                    else:
                        self.current_state.probabilities[s] *= scale
            else:
                # Отрицательное подкрепление: уменьшаем вероятность выбранного перехода
                delta = self.learning_rate * p_chosen
                new_p = max(0.01, p_chosen - delta)
                diff = p_chosen - new_p
                other = [s for s in all_targets if s != chosen]
                if other:
                    inc = diff / len(other)
                    for s in other:
                        self.current_state.probabilities[s] += inc
                self.current_state.probabilities[chosen] = new_p

            self.current_state.normalize_probabilities()

        # 3. Переход в выбранное состояние
        self.current_state = chosen
        self.current_state.visit_count += 1

    def draw(self, surface):
        # Рисуем дуги
        for state in self.states.values():
            for target, prob in state.probabilities.items():
                start = state.position
                end = target.position
                width = max(1, int(prob * 5))
                color = (int(255 * (1 - prob)), int(255 * prob), 0)
                pygame.draw.line(surface, color, start, end, width)
                self._draw_arrow(surface, start, end, color)

        # Рисуем состояния
        for state in self.states.values():
            x, y = state.position
            if state.activation > 0.7:
                fill = GREEN
            elif state.activation > 0.3:
                fill = YELLOW
            else:
                fill = GRAY
            pygame.draw.circle(surface, BLACK, (x, y), 35, 2)
            pygame.draw.circle(surface, fill, (x, y), 33)

            # Название (сокращённое)
            font = pygame.font.Font(None, 18)
            if "Быстрое" in state.name:
                short = state.name.replace("Быстрое движение ", "Б.")
            elif state.name == "Остановка":
                short = "Стоп"
            else:
                short = state.name.replace("Движение ", "")
            text = font.render(short, True, BLACK)
            text_rect = text.get_rect(center=(x, y))
            surface.blit(text, text_rect)

            # Процент активации
            small = pygame.font.Font(None, 16)
            act_text = small.render(f"{int(state.activation*100)}%", True, BLUE)
            act_rect = act_text.get_rect(center=(x, y + 45))
            surface.blit(act_text, act_rect)

            # Выделение текущего состояния
            if state == self.current_state:
                pygame.draw.circle(surface, RED, (x, y), 38, 3)

    @staticmethod
    def _draw_arrow(surface, start, end, color):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        angle = math.atan2(dy, dx)
        t = 0.8
        ax = start[0] + t * dx
        ay = start[1] + t * dy
        L = 10
        left = (ax - L * math.cos(angle - math.pi/6),
                ay - L * math.sin(angle - math.pi/6))
        right = (ax - L * math.cos(angle + math.pi/6),
                 ay - L * math.sin(angle + math.pi/6))
        pygame.draw.polygon(surface, color, [(ax, ay), left, right])


# ======================== 5. ПРЕПЯТСТВИЕ ========================
class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, surface):
        pygame.draw.circle(surface, RED, (int(self.x), int(self.y)), self.radius)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), self.radius, 2)

    def check_collision(self, robot_x, robot_y, robot_radius=12):
        dist = math.hypot(self.x - robot_x, self.y - robot_y)
        return dist < self.radius + robot_radius


# ======================== 6. РОБОТ ========================
class Robot:
    def __init__(self, x, y, angle=0):
        self.x = x
        self.y = y
        self.angle = angle
        self.radius = 12
        self.trail = []

    def update(self, action_state):
        linear, angular, _ = STATE_INFO[action_state.name]
        self.angle += angular
        self.x += linear * math.cos(math.radians(self.angle))
        self.y += linear * math.sin(math.radians(self.angle))
        self.trail.append((self.x, self.y))
        if len(self.trail) > 500:
            self.trail.pop(0)

    def check_boundary_collision(self, margin=30):
        return (self.x < margin or self.x > WIDTH - margin or
                self.y < margin or self.y > HEIGHT - margin)

    def draw(self, surface, show_trail=True):
        if show_trail and len(self.trail) > 1:
            pygame.draw.lines(surface, CYAN, False, self.trail, 2)
        pygame.draw.circle(surface, BLUE, (int(self.x), int(self.y)), self.radius)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), self.radius, 2)
        end_x = self.x + 15 * math.cos(math.radians(self.angle))
        end_y = self.y + 15 * math.sin(math.radians(self.angle))
        pygame.draw.line(surface, RED, (self.x, self.y), (end_x, end_y), 3)


# ======================== 7. ФУНКЦИЯ ПОДКРЕПЛЕНИЯ ========================
def compute_reward(prev_state_name, collision, boundary_collision):
    if collision or boundary_collision:
        return -1.0
    if "Быстрое" in prev_state_name:
        return 1.0
    return 0.0


# ======================== 8. ГЛАВНАЯ ПРОГРАММА ========================
def main():
    fsm = FSM_Cetlin()
    fsm.set_current_state("Остановка")
    fsm.update_activation()

    robot = Robot(WIDTH//2, HEIGHT//2 + 100, angle=270)
    obstacles = [
        Obstacle(WIDTH//2 - 150, HEIGHT//2, 40),
        Obstacle(WIDTH//2 + 150, HEIGHT//2, 40),
        Obstacle(WIDTH//2, HEIGHT//2 + 100, 35),
        Obstacle(WIDTH//2, HEIGHT//2 - 100, 35),
        Obstacle(WIDTH//2 + 80, HEIGHT//2 - 60, 30),
        Obstacle(WIDTH//2 - 80, HEIGHT//2 + 60, 30),
    ]

    running = True
    paused = False
    show_trail = True
    total_steps = 0
    reward_history = []
    prev_state = fsm.current_state

    while running:
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_p:
                    paused = not paused
                elif event.key == pygame.K_r:
                    # Сброс
                    fsm = FSM_Cetlin()
                    fsm.set_current_state("Остановка")
                    prev_state = fsm.current_state
                    robot = Robot(WIDTH//2, HEIGHT//2 + 100, angle=270)
                    total_steps = 0
                    reward_history.clear()
                elif event.key == pygame.K_t:
                    show_trail = not show_trail

        if paused:
            screen.fill(WHITE)
            for obs in obstacles:
                obs.draw(screen)
            robot.draw(screen, show_trail)
            fsm.draw(screen)
            font = pygame.font.Font(None, 36)
            pause_text = font.render("PAUSED - Press P", True, BLACK)
            screen.blit(pause_text, (WIDTH//2 - 100, 50))
            pygame.display.flip()
            continue

        # --- Шаг обучения ---
        prev_state_name = prev_state.name
        robot.update(fsm.current_state)

        # Проверка столкновений
        collision = any(obs.check_collision(robot.x, robot.y, robot.radius) for obs in obstacles)
        boundary = robot.check_boundary_collision()
        reward = compute_reward(prev_state_name, collision, boundary)
        reward_history.append(reward)

        if collision or boundary:
            # Откат при столкновении
            robot.x -= 10 * math.cos(math.radians(robot.angle))
            robot.y -= 10 * math.sin(math.radians(robot.angle))

        fsm.transition(reward)
        fsm.update_activation()
        prev_state = fsm.current_state

        # Отрисовка
        screen.fill(WHITE)
        for obs in obstacles:
            obs.draw(screen)
        robot.draw(screen, show_trail)
        fsm.draw(screen)

        # Информационная панель
        font = pygame.font.Font(None, 24)
        screen.blit(font.render(f"Шаг: {total_steps}", True, BLACK), (10, 10))
        screen.blit(font.render(f"Состояние: {fsm.current_state.name}", True, BLACK), (10, 35))
        screen.blit(font.render(f"Подкрепление: {reward}", True, GREEN if reward>0 else RED if reward<0 else BLACK), (10, 60))
        avg_reward = np.mean(reward_history[-100:]) if reward_history else 0
        screen.blit(font.render(f"Ср. подкр. (100): {avg_reward:.2f}", True, BLACK), (10, 85))
        screen.blit(font.render("P-пауза  R-сброс  T-траектория  ESC-выход", True, BLACK), (10, HEIGHT-30))

        # Легенда
        leg = pygame.font.Font(None, 18)
        screen.blit(leg.render("Зелёные линии: высокая вероятность", True, GREEN), (WIDTH-250, HEIGHT-80))
        screen.blit(leg.render("Красные линии: низкая вероятность", True, RED), (WIDTH-250, HEIGHT-60))
        screen.blit(leg.render("Красная обводка = текущее состояние", True, RED), (WIDTH-250, HEIGHT-40))

        pygame.display.flip()
        total_steps += 1

    pygame.quit()
    print(f"Симуляция завершена. Шагов: {total_steps}")

if __name__ == "__main__":
    main()