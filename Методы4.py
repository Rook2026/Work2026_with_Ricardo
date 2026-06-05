import pygame
import numpy as np
import random
import math
from collections import defaultdict

#1. НАСТРОЙКА ОКНА
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Семантическая сеть управления роботом")
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

#2. НАБОР СОСТОЯНИЙ РОБОТА
STATES = [
    "ДВИЖЕНИЕ_ВПЕРЕД",
    "ДВИЖЕНИЕ_ВЛЕВО",
    "ДВИЖЕНИЕ_ВПРАВО",
    "ДВИЖЕНИЕ_НАЗАД",
    "СРАБАТЫВАНИЕ_СЕНСОРА",
    "СТОЛКНОВЕНИЕ"
]

#3. КЛАСС УЗЛА СЕМАНТИЧЕСКОЙ СЕТИ
class Node:
    """Узел семантической сети"""
    def __init__(self, name):
        self.name = name                # Название состояния
        self.activation = 0.0           # Степень активации (0..1)
        self.incoming = []              # Список входящих узлов
        self.outgoing = []              # Список исходящих узлов
        self.probabilities = {}         # {узел: вероятность перехода}
        self.position = (0, 0)          # Координаты для отрисовки

    def add_outgoing(self, target_node, prob=0.1):
        """Добавить исходящую дугу с вероятностью"""
        if target_node not in self.outgoing:
            self.outgoing.append(target_node)
            self.probabilities[target_node] = prob
            target_node.incoming.append(self)

    def normalize_probabilities(self):
        """Нормировка вероятностей (сумма = 1)"""
        total = sum(self.probabilities.values())
        if total > 0:
            for node in self.probabilities:
                self.probabilities[node] /= total

    def choose_next_state(self):
        """Вероятностный выбор следующего состояния"""
        if not self.probabilities:
            return None
        nodes = list(self.probabilities.keys())
        probs = list(self.probabilities.values())
        return np.random.choice(nodes, p=probs)

    def __repr__(self):
        return f"Node({self.name}, act={self.activation:.2f})"


#3. КЛАСС СЕМАНТИЧЕСКОЙ СЕТИ
class SemanticMap:
    """Карта семантической сети"""
    def __init__(self):
        self.nodes = {}          # name -> Node
        self.history = []        # История состояний (для обучения)
        self._init_network()
        self._set_positions()

    def _init_network(self):
        """Инициализация узлов и связей (начальная структура)"""
        # Создаём узлы
        for s in STATES:
            self.nodes[s] = Node(s)

        # Задаём связи (вероятности – начальные, будут скорректированы обучением)
        # Движение вперёд -> сенсор или столкновение
        self.nodes["ДВИЖЕНИЕ_ВПЕРЕД"].add_outgoing(self.nodes["СРАБАТЫВАНИЕ_СЕНСОРА"], 0.6)
        self.nodes["ДВИЖЕНИЕ_ВПЕРЕД"].add_outgoing(self.nodes["СТОЛКНОВЕНИЕ"], 0.4)

        # Движение влево/вправо/назад -> сенсор или столкновение
        for act in ["ДВИЖЕНИЕ_ВЛЕВО", "ДВИЖЕНИЕ_ВПРАВО", "ДВИЖЕНИЕ_НАЗАД"]:
            self.nodes[act].add_outgoing(self.nodes["СРАБАТЫВАНИЕ_СЕНСОРА"], 0.7)
            self.nodes[act].add_outgoing(self.nodes["СТОЛКНОВЕНИЕ"], 0.3)

        # Сенсор -> манёвры (избегание)
        sensor = self.nodes["СРАБАТЫВАНИЕ_СЕНСОРА"]
        sensor.add_outgoing(self.nodes["ДВИЖЕНИЕ_ВЛЕВО"], 0.3)
        sensor.add_outgoing(self.nodes["ДВИЖЕНИЕ_ВПРАВО"], 0.3)
        sensor.add_outgoing(self.nodes["ДВИЖЕНИЕ_НАЗАД"], 0.3)
        sensor.add_outgoing(self.nodes["ДВИЖЕНИЕ_ВПЕРЕД"], 0.1)

        # Столкновение -> отъезд назад или в сторону
        crash = self.nodes["СТОЛКНОВЕНИЕ"]
        crash.add_outgoing(self.nodes["ДВИЖЕНИЕ_НАЗАД"], 0.5)
        crash.add_outgoing(self.nodes["ДВИЖЕНИЕ_ВЛЕВО"], 0.5)

        # Нормировка для каждого узла
        for node in self.nodes.values():
            node.normalize_probabilities()

    def _set_positions(self):
        """Расстановка узлов в окне (для наглядности)"""
        center_x, center_y = WIDTH // 2, HEIGHT // 2
        radius = 200
        angles = {
            "ДВИЖЕНИЕ_ВПЕРЕД": 270,
            "ДВИЖЕНИЕ_ВЛЕВО": 180,
            "ДВИЖЕНИЕ_ВПРАВО": 0,
            "ДВИЖЕНИЕ_НАЗАД": 90,
            "СРАБАТЫВАНИЕ_СЕНСОРА": 315,
            "СТОЛКНОВЕНИЕ": 135
        }
        for name, node in self.nodes.items():
            rad = math.radians(angles[name])
            x = center_x + radius * math.cos(rad)
            y = center_y + radius * math.sin(rad)
            node.position = (int(x), int(y))

    # ---------- 4. ФУНКЦИЯ АКТИВАЦИИ (вероятностный подход) ----------
    def update_activation(self, current_state_name, input_activation=1.0):
        """
        Обновляет степени активации узлов.
        Текущий узел получает input_activation, затем активация распространяется
        на соседей с коэффициентом затухания, пропорциональным вероятности перехода.
        """
        # Сброс всех активаций
        for node in self.nodes.values():
            node.activation = 0.0

        curr_node = self.nodes[current_state_name]
        curr_node.activation = input_activation

        # Первый шаг распространения
        for node in self.nodes.values():
            if node.activation > 0:
                for neighbor in node.outgoing:
                    prob = node.probabilities.get(neighbor, 0.0)
                    neighbor.activation += node.activation * prob * 0.5  # затухание

        # Ограничиваем [0, 1]
        for node in self.nodes.values():
            node.activation = min(1.0, node.activation)

    # ---------- 5. АВТОМАТИЧЕСКАЯ НАСТРОЙКА ВЕРОЯТНОСТЕЙ ПО ИСТОРИИ ----------
    def learn_from_history(self, history):
        """
        history: список строк – последовательность состояний робота.
        Функция пересчитывает вероятности переходов на основе частот.
        """
        transitions = defaultdict(lambda: defaultdict(int))
        for i in range(len(history) - 1):
            curr = history[i]
            nxt = history[i + 1]
            transitions[curr][nxt] += 1

        for curr_state, next_dict in transitions.items():
            if curr_state not in self.nodes:
                continue
            node = self.nodes[curr_state]
            total = sum(next_dict.values())
            for next_state, count in next_dict.items():
                if next_state in self.nodes:
                    node.probabilities[self.nodes[next_state]] = count / total
            node.normalize_probabilities()

    def add_to_history(self, state):
        """Добавляет состояние в историю (ограничивая длину)"""
        self.history.append(state)
        if len(self.history) > 200:
            self.history.pop(0)

    # ---------- 8. ФУНКЦИЯ ПРИНЯТИЯ РЕШЕНИЙ БЕЗ СТОЛКНОВЕНИЙ ----------
    def make_decision(self, sensor_active=False):
        """
        Возвращает следующее действие для робота, избегая столкновений.
        Если сенсор активен – выбирается действие из безопасных (влево, вправо, назад).
        Иначе – вероятностный выбор по текущему состоянию.
        """
        current_node = self.nodes[current_robot_state]  # current_robot_state – глобальная переменная

        if sensor_active:
            # Избегаем движения вперёд при сработавшем сенсоре
            safe_actions = ["ДВИЖЕНИЕ_ВЛЕВО", "ДВИЖЕНИЕ_ВПРАВО", "ДВИЖЕНИЕ_НАЗАД"]
            # Веса – текущие вероятности из сети
            probs = []
            for act in safe_actions:
                prob = current_node.probabilities.get(self.nodes[act], 0.2)
                probs.append(prob)
            total = sum(probs)
            if total > 0:
                probs = [p / total for p in probs]
                decision = np.random.choice(safe_actions, p=probs)
            else:
                decision = random.choice(safe_actions)
            return decision

        # Обычный вероятностный переход
        next_node = current_node.choose_next_state()
        if next_node:
            return next_node.name
        return "ДВИЖЕНИЕ_ВПЕРЕД"

    # ---------- 6-7. ВИЗУАЛИЗАЦИЯ (узлы – окружности, переходы – отрезки, цвет активации) ----------
    def draw(self, surface):
        """Отрисовка семантической сети"""
        # Рисуем дуги (переходы)
        for node in self.nodes.values():
            for target in node.outgoing:
                start = node.position
                end = target.position
                prob = node.probabilities.get(target, 0.1)
                # Толщина линии пропорциональна вероятности
                width = max(1, int(prob * 5))
                # Цвет: зелёный (высокая вероятность) – красный (низкая)
                color = (int(255 * (1 - prob)), int(255 * prob), 0)
                pygame.draw.line(surface, color, start, end, width)
                self._draw_arrow(surface, start, end, color)

        # Рисуем узлы (окружности)
        for node in self.nodes.values():
            x, y = node.position
            # Цвет в зависимости от активации
            intensity = int(255 * node.activation)
            if node.activation > 0.7:
                fill_color = GREEN
            elif node.activation > 0.3:
                fill_color = YELLOW
            else:
                fill_color = GRAY
            pygame.draw.circle(surface, BLACK, (x, y), 35, 2)
            pygame.draw.circle(surface, fill_color, (x, y), 33)

            # Название состояния
            font = pygame.font.Font(None, 20)
            text = font.render(node.name.replace("_", " "), True, BLACK)
            text_rect = text.get_rect(center=(x, y))
            surface.blit(text, text_rect)

            # Степень активации (в процентах)
            small_font = pygame.font.Font(None, 16)
            act_text = small_font.render(f"{int(node.activation*100)}%", True, BLUE)
            act_rect = act_text.get_rect(center=(x, y + 45))
            surface.blit(act_text, act_rect)

    def _draw_arrow(self, surface, start, end, color):
        """Рисует стрелку на конце дуги (направление перехода)"""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        angle = math.atan2(dy, dx)
        # Позиция стрелки – на 80% от начала
        t = 0.8
        arrow_x = start[0] + t * dx
        arrow_y = start[1] + t * dy
        arrow_len = 12
        left_angle = angle - math.pi / 6
        right_angle = angle + math.pi / 6
        left_tip = (arrow_x - arrow_len * math.cos(left_angle),
                    arrow_y - arrow_len * math.sin(left_angle))
        right_tip = (arrow_x - arrow_len * math.cos(right_angle),
                     arrow_y - arrow_len * math.sin(right_angle))
        pygame.draw.polygon(surface, color, [(arrow_x, arrow_y), left_tip, right_tip])


#СИМУЛЯЦИЯ РОБОТА
class Robot:
    def __init__(self):
        self.x = WIDTH // 2 - 150
        self.y = HEIGHT // 2
        self.angle = 0        # 0° – вправо, 90° – вниз и т.д.
        self.speed = 3
        self.sensor_active = False

    def move(self, action):
        global current_robot_state
        old_x, old_y = self.x, self.y

        if action == "ДВИЖЕНИЕ_ВПЕРЕД":
            self.x += self.speed * math.cos(math.radians(self.angle))
            self.y += self.speed * math.sin(math.radians(self.angle))
        elif action == "ДВИЖЕНИЕ_НАЗАД":
            self.x -= self.speed * math.cos(math.radians(self.angle))
            self.y -= self.speed * math.sin(math.radians(self.angle))
        elif action == "ДВИЖЕНИЕ_ВЛЕВО":
            self.angle = (self.angle - 20) % 360
        elif action == "ДВИЖЕНИЕ_ВПРАВО":
            self.angle = (self.angle + 20) % 360

        # Проверка столкновения со стенами
        margin = 40
        if (self.x < margin or self.x > WIDTH - margin or
            self.y < margin or self.y > HEIGHT - margin):
            self.x, self.y = old_x, old_y
            current_robot_state = "СТОЛКНОВЕНИЕ"
            return "СТОЛКНОВЕНИЕ"

        # Симуляция сенсора (вероятностное обнаружение препятствия у стен)
        self.sensor_active = self._simulate_sensor()
        if self.sensor_active:
            current_robot_state = "СРАБАТЫВАНИЕ_СЕНСОРА"
            return "СРАБАТЫВАНИЕ_СЕНСОРА"

        current_robot_state = action
        return action

    def _simulate_sensor(self):
        """Сенсор активируется при приближении к границе или случайно"""
        margin_detect = 80
        if (self.x < margin_detect or self.x > WIDTH - margin_detect or
            self.y < margin_detect or self.y > HEIGHT - margin_detect):
            return True
        # Случайное ложное срабатывание с вероятностью 2%
        return random.random() < 0.02

    def draw(self, surface):
        # Рисуем робота (квадрат)
        rect = pygame.Rect(self.x - 15, self.y - 15, 30, 30)
        pygame.draw.rect(surface, BLUE, rect)
        # Линия направления
        end_x = self.x + 20 * math.cos(math.radians(self.angle))
        end_y = self.y + 20 * math.sin(math.radians(self.angle))
        pygame.draw.line(surface, RED, (self.x, self.y), (end_x, end_y), 3)
        # Если сенсор активен – рисуем оранжевый круг
        if self.sensor_active:
            pygame.draw.circle(surface, ORANGE, (int(self.x), int(self.y)), 45, 2)


#ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
current_robot_state = "ДВИЖЕНИЕ_ВПЕРЕД"

#10. ВХОДНЫЕ ИСТОРИЧЕСКИЕ ДАННЫЕ
# Пример массива исторических состояний (для обучения)
training_history = [
    "ДВИЖЕНИЕ_ВПЕРЕД", "СРАБАТЫВАНИЕ_СЕНСОРА", "ДВИЖЕНИЕ_ВЛЕВО", "ДВИЖЕНИЕ_ВПЕРЕД",
    "ДВИЖЕНИЕ_ВПЕРЕД", "СРАБАТЫВАНИЕ_СЕНСОРА", "ДВИЖЕНИЕ_ВПРАВО", "ДВИЖЕНИЕ_ВПЕРЕД",
    "ДВИЖЕНИЕ_ВЛЕВО", "ДВИЖЕНИЕ_ВПЕРЕД", "СТОЛКНОВЕНИЕ", "ДВИЖЕНИЕ_НАЗАД", "ДВИЖЕНИЕ_ВПЕРЕД",
    "ДВИЖЕНИЕ_ВПЕРЕД", "СРАБАТЫВАНИЕ_СЕНСОРА", "ДВИЖЕНИЕ_НАЗАД", "ДВИЖЕНИЕ_ВПРАВО",
    "ДВИЖЕНИЕ_ВПЕРЕД", "ДВИЖЕНИЕ_ВЛЕВО", "ДВИЖЕНИЕ_ВПЕРЕД"
]

#ОСНОВНАЯ ПРОГРАММА
def main():
    global current_robot_state
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Семантическая сеть – управление роботом")
    clock = pygame.time.Clock()

    # Создаём семантическую сеть
    semantic_net = SemanticMap()

    # Обучаем сеть на исторических данных (пункт 5)
    print("Обучение сети на исторических данных...")
    semantic_net.learn_from_history(training_history)
    print("Обучение завершено.")

    robot = Robot()
    running = True
    step = 0
    max_steps = 500

    while running and step < max_steps:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # Принятие решения (пункт 8)
        decision = semantic_net.make_decision(robot.sensor_active)

        # Движение робота
        new_state = robot.move(decision)

        # Обновление активации узлов (пункт 4)
        semantic_net.update_activation(current_robot_state)

        # Добавление в историю (для возможного дообучения)
        semantic_net.add_to_history(new_state)

        # Отрисовка
        screen.fill(WHITE)
        semantic_net.draw(screen)
        robot.draw(screen)

        # Вывод информации на экран
        font_info = pygame.font.Font(None, 24)
        info1 = font_info.render(f"Шаг: {step}", True, BLACK)
        info2 = font_info.render(f"Решение: {decision}", True, BLACK)
        info3 = font_info.render(f"Сенсор: {'АКТИВЕН' if robot.sensor_active else 'выкл'}", True, ORANGE if robot.sensor_active else BLACK)
        screen.blit(info1, (10, 10))
        screen.blit(info2, (10, 35))
        screen.blit(info3, (10, 60))

        # Легенда
        legend_y = HEIGHT - 120
        legend1 = font_info.render("Зелёные линии: высокая вероятность перехода", True, GREEN)
        legend2 = font_info.render("Красные линии: низкая вероятность", True, RED)
        legend3 = font_info.render("Яркость узла = степень активации", True, BLACK)
        screen.blit(legend1, (WIDTH - 300, legend_y))
        screen.blit(legend2, (WIDTH - 300, legend_y + 25))
        screen.blit(legend3, (WIDTH - 300, legend_y + 50))

        pygame.display.flip()
        clock.tick(FPS)
        step += 1

    print("Симуляция завершена.")
    pygame.quit()


if __name__ == "__main__":
    main()