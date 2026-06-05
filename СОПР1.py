import pygame
import math
import random
import numpy as np
from collections import defaultdict
import json
import os

# Инициализация Pygame
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Стабилизация обращенного маятника - Q-обучение")
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
CYAN = (0, 255, 255)

class Robot:
    """Класс мобильного робота с обращенным маятником"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.v = 0.0
        self.angle = math.radians(5)  # Начальный угол 5 градусов
        self.angular_velocity = 0.0
        self.platform_width = 80
        self.platform_height = 20
        self.pole_length = 150
        
        # Физические параметры
        self.mass_cart = 1.0
        self.mass_pole = 0.1
        self.length = self.pole_length / 100
        self.gravity = 9.8
        self.dt = 0.02
        self.force = 0.0
        
        # Ограничения
        self.x_min = 50
        self.x_max = WIDTH - 50
        self.angle_limit = math.radians(45)
        
        # История для отрисовки траектории
        self.trajectory = []
        
    def apply_action(self, action):
        """Применение действия (ускорения) к роботу"""
        if action == 0:
            self.force = -10.0
        elif action == 1:
            self.force = 0.0
        elif action == 2:
            self.force = 10.0
        
    def update(self):
        """Обновление физики робота"""
        sin_angle = math.sin(self.angle)
        cos_angle = math.cos(self.angle)
        
        total_mass = self.mass_cart + self.mass_pole
        
        # Угловое ускорение
        numerator = (self.gravity * sin_angle - 
                    cos_angle * (self.force + self.mass_pole * self.length * 
                                self.angular_velocity**2 * sin_angle) / total_mass)
        denominator = (self.length * (4/3 - self.mass_pole * cos_angle**2 / total_mass))
        
        angular_acceleration = numerator / denominator
        
        # Линейное ускорение тележки
        cart_acceleration = (self.force + self.mass_pole * self.length * 
                            (self.angular_velocity**2 * sin_angle - 
                             angular_acceleration * cos_angle)) / total_mass
        
        # Обновление скоростей
        self.angular_velocity += angular_acceleration * self.dt
        self.v += cart_acceleration * self.dt
        
        # Обновление позиций
        self.angle += self.angular_velocity * self.dt
        self.x += self.v * self.dt
        
        # Ограничения
        self.x = max(self.x_min, min(self.x_max, self.x))
        
        # Добавление в траекторию
        self.trajectory.append((int(self.x), int(self.y)))
        if len(self.trajectory) > 500:
            self.trajectory.pop(0)
        
        # Проверка падения
        return abs(self.angle) > self.angle_limit
    
    def get_state(self, n_angle_bins=10, n_velocity_bins=10):
        """Дискретизация состояний"""
        # Дискретизация угла
        angle_range = (-self.angle_limit, self.angle_limit)
        angle_bin = int((self.angle - angle_range[0]) / 
                       (angle_range[1] - angle_range[0]) * n_angle_bins)
        angle_bin = max(0, min(n_angle_bins - 1, angle_bin))
        
        # Дискретизация скорости
        velocity_range = (-5.0, 5.0)
        velocity_bin = int((self.v - velocity_range[0]) / 
                          (velocity_range[1] - velocity_range[0]) * n_velocity_bins)
        velocity_bin = max(0, min(n_velocity_bins - 1, velocity_bin))
        
        return (angle_bin, velocity_bin)
    
    def reset(self):
        """Сброс робота в начальное состояние"""
        self.x = WIDTH // 2
        self.v = random.uniform(-0.5, 0.5)
        self.angle = random.uniform(-0.1, 0.1)
        self.angular_velocity = random.uniform(-0.5, 0.5)
        self.trajectory.clear()

class QLearning:
    """Класс Q-обучения для стабилизации маятника"""
    def __init__(self, n_angle_bins=10, n_velocity_bins=10, n_actions=3):
        self.n_angle_bins = n_angle_bins
        self.n_velocity_bins = n_velocity_bins
        self.n_actions = n_actions
        
        # Q-таблица
        self.q_table = defaultdict(lambda: defaultdict(float))
        
        # Параметры обучения
        self.alpha = 0.1
        self.gamma = 0.99
        self.epsilon = 0.3
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        
        # История для журнала
        self.history = []
        
        # Статистика
        self.episode_count = 0
        self.total_reward = 0
        
    def get_action(self, state, training=True):
        """Выбор действия на основе ε-жадной стратегии"""
        if training and random.random() < self.epsilon:
            return random.randint(0, self.n_actions - 1)
        else:
            state_actions = self.q_table[state]
            if state_actions:
                return max(state_actions, key=state_actions.get)
            else:
                return random.randint(0, self.n_actions - 1)
    
    def get_reward(self, robot):
        """Расчет функционала подкрепления"""
        angle = abs(robot.angle)
        angle_degrees = math.degrees(angle)
        
        # Базовая награда
        reward = 1.0 - (angle_degrees / math.degrees(robot.angle_limit))
        
        # Бонус за почти вертикальное положение
        if angle_degrees < 2:
            reward += 0.5
        elif angle_degrees < 5:
            reward += 0.2
        
        # Штраф за большое отклонение
        if angle_degrees > 30:
            reward -= 0.5
        
        # Штраф за выход за пределы
        if robot.x <= robot.x_min + 10 or robot.x >= robot.x_max - 10:
            reward -= 0.3
        
        return reward
    
    def update(self, state, action, reward, next_state, done):
        """Обновление Q-таблицы"""
        current_q = self.q_table[state][action]
        
        next_q_values = self.q_table[next_state]
        if next_q_values and not done:
            max_next_q = max(next_q_values.values())
        else:
            max_next_q = 0
        
        new_q = current_q + self.alpha * (reward + self.gamma * max_next_q - current_q)
        self.q_table[state][action] = new_q
        
        # Сохранение в историю
        self.history.append({
            'angle': math.degrees(state[0]),
            'velocity': state[1],
            'action': action,
            'reward': reward
        })
        
        # Обновление epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
    
    def save_q_table(self, filename="q_table.json"):
        """Сохранение Q-таблицы в JSON"""
        serializable = {}
        for state, actions in self.q_table.items():
            # Используем кортеж как ключ
            key = f"{state[0]},{state[1]}"
            serializable[key] = {str(action): value for action, value in actions.items()}
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(serializable, f, indent=2, ensure_ascii=False)
            print(f"Q-таблица сохранена в {filename}")
        except Exception as e:
            print(f"Ошибка сохранения Q-таблицы: {e}")
    
    def load_q_table(self, filename="q_table.json"):
        """Загрузка Q-таблицы из JSON"""
        if not os.path.exists(filename):
            print(f"Файл {filename} не найден. Создана новая Q-таблица.")
            return
        
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            loaded_count = 0
            for key, actions in data.items():
                try:
                    # Парсинг ключа "angle,velocity"
                    parts = key.split(',')
                    if len(parts) == 2:
                        angle_bin = int(parts[0])
                        velocity_bin = int(parts[1])
                        state = (angle_bin, velocity_bin)
                        
                        for action_str, value in actions.items():
                            action = int(action_str)
                            self.q_table[state][action] = float(value)
                            loaded_count += 1
                except (ValueError, IndexError) as e:
                    print(f"Пропущена запись с ключом {key}: {e}")
                    continue
            
            print(f"Загружена Q-таблица: {len(data)} состояний, {loaded_count} записей")
        except json.JSONDecodeError as e:
            print(f"Ошибка чтения JSON: {e}. Создана новая Q-таблица.")
        except Exception as e:
            print(f"Ошибка загрузки Q-таблицы: {e}. Создана новая Q-таблица.")
    
    def save_history(self, filename="history.txt"):
        """Сохранение истории в текстовый файл"""
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write("История перемещений робота\n")
                f.write("Формат: {A, V, U}\n")
                f.write("A - угол балки (градусы), V - скорость (бин), U - действие\n")
                f.write("=" * 60 + "\n\n")
                
                for i, record in enumerate(self.history):
                    f.write(f"[{i}] {{A={record['angle']:.2f}, "
                           f"V={record['velocity']}, "
                           f"U={record['action']}}}\n")
            
            print(f"История сохранена в {filename}")
        except Exception as e:
            print(f"Ошибка сохранения истории: {e}")
    
    def get_q_table_stats(self):
        """Получение статистики Q-таблицы"""
        total_entries = sum(len(actions) for actions in self.q_table.values())
        return {
            'states': len(self.q_table),
            'entries': total_entries,
            'epsilon': self.epsilon
        }

def draw_robot(screen, robot):
    """Отрисовка робота с поворотной балкой"""
    # Отрисовка траектории
    if len(robot.trajectory) > 1:
        pygame.draw.lines(screen, GRAY, False, robot.trajectory, 2)
    
    # Отрисовка платформы
    platform_rect = pygame.Rect(
        robot.x - robot.platform_width // 2,
        robot.y - robot.platform_height // 2,
        robot.platform_width,
        robot.platform_height
    )
    pygame.draw.rect(screen, BLUE, platform_rect)
    pygame.draw.rect(screen, BLACK, platform_rect, 2)
    
    # Отрисовка колес
    wheel_radius = 10
    left_wheel_center = (robot.x - robot.platform_width // 3, robot.y + robot.platform_height // 2)
    right_wheel_center = (robot.x + robot.platform_width // 3, robot.y + robot.platform_height // 2)
    
    pygame.draw.circle(screen, BLACK, left_wheel_center, wheel_radius)
    pygame.draw.circle(screen, BLACK, right_wheel_center, wheel_radius)
    pygame.draw.circle(screen, DARK_GRAY, left_wheel_center, wheel_radius - 3)
    pygame.draw.circle(screen, DARK_GRAY, right_wheel_center, wheel_radius - 3)
    
    # Отрисовка балки
    pole_end_x = robot.x + robot.pole_length * math.sin(robot.angle)
    pole_end_y = robot.y - robot.pole_length * math.cos(robot.angle)
    
    # Цвет балки зависит от угла отклонения
    angle_ratio = abs(robot.angle) / robot.angle_limit
    if angle_ratio < 0.3:
        pole_color = GREEN
    elif angle_ratio < 0.7:
        pole_color = YELLOW
    else:
        pole_color = RED
    
    pygame.draw.line(screen, pole_color, 
                    (robot.x, robot.y - robot.platform_height // 2),
                    (pole_end_x, pole_end_y), 5)
    
    # Отрисовка груза на конце балки
    pygame.draw.circle(screen, RED, (int(pole_end_x), int(pole_end_y)), 8)
    pygame.draw.circle(screen, BLACK, (int(pole_end_x), int(pole_end_y)), 8, 2)
    
    # Отрисовка оси вращения
    pygame.draw.circle(screen, YELLOW, (robot.x, robot.y - robot.platform_height // 2), 5)
    pygame.draw.circle(screen, BLACK, (robot.x, robot.y - robot.platform_height // 2), 5, 1)

def draw_info_panel(screen, x, y, robot, q_learning, episode, total_reward):
    """Отрисовка информационной панели"""
    font_title = pygame.font.Font(None, 28)
    font_text = pygame.font.Font(None, 20)
    
    # Фон панели
    panel_rect = pygame.Rect(x - 10, y - 10, 300, 350)
    pygame.draw.rect(screen, WHITE, panel_rect)
    pygame.draw.rect(screen, BLACK, panel_rect, 2)
    
    # Заголовок
    title = font_title.render("ИНФОРМАЦИЯ", True, BLACK)
    screen.blit(title, (x + 70, y))
    
    y += 35
    
    # Информация о роботе
    info_lines = [
        f"Позиция X: {robot.x:.1f}",
        f"Скорость V: {robot.v:.2f}",
        f"Угол балки: {math.degrees(robot.angle):.1f}°",
        f"Угловая скорость: {robot.angular_velocity:.2f}",
        f"Приложенная сила: {robot.force:.1f}",
        "",
        f"Эпизод: {episode}",
        f"Награда: {total_reward:.2f}",
        f"Epsilon: {q_learning.epsilon:.3f}",
    ]
    
    for line in info_lines:
        text_surface = font_text.render(line, True, BLACK)
        screen.blit(text_surface, (x, y))
        y += 22
    
    # Статистика Q-таблицы
    stats = q_learning.get_q_table_stats()
    stats_lines = [
        f"Состояний в Q-таблице: {stats['states']}",
        f"Записей в Q-таблице: {stats['entries']}",
    ]
    
    for line in stats_lines:
        text_surface = font_text.render(line, True, BLUE)
        screen.blit(text_surface, (x, y))
        y += 22

def draw_legend(screen, x, y):
    """Отрисовка легенды"""
    font_title = pygame.font.Font(None, 28)
    font_text = pygame.font.Font(None, 20)
    
    # Фон легенды
    legend_rect = pygame.Rect(x - 10, y - 10, 300, 200)
    pygame.draw.rect(screen, WHITE, legend_rect)
    pygame.draw.rect(screen, BLACK, legend_rect, 2)
    
    # Заголовок
    title = font_title.render("ЛЕГЕНДА", True, BLACK)
    screen.blit(title, (x + 80, y))
    
    y += 35
    
    # Элементы легенды
    legend_items = [
        ("Платформа", BLUE),
        ("Балка (стабильно)", GREEN),
        ("Балка (внимание)", YELLOW),
        ("Балка (опасно)", RED),
        ("Траектория", GRAY),
        ("Ось вращения", YELLOW),
    ]
    
    for text, color in legend_items:
        pygame.draw.rect(screen, color, (x, y, 20, 20))
        pygame.draw.rect(screen, BLACK, (x, y, 20, 20), 1)
        text_surface = font_text.render(text, True, BLACK)
        screen.blit(text_surface, (x + 30, y + 2))
        y += 25

def draw_q_table_visualization(screen, q_learning, x, y, width, height):
    """Визуализация Q-таблицы"""
    font_title = pygame.font.Font(None, 24)
    font_small = pygame.font.Font(None, 16)
    
    # Фон
    rect = pygame.Rect(x, y, width, height)
    pygame.draw.rect(screen, WHITE, rect)
    pygame.draw.rect(screen, BLACK, rect, 2)
    
    # Заголовок
    title = font_title.render("Q-таблица (фрагмент)", True, BLACK)
    screen.blit(title, (x + 10, y + 5))
    
    # Отображение нескольких записей Q-таблицы
    y_offset = y + 35
    count = 0
    
    # ИСПРАВЛЕНО: используем q_learning.q_table вместо self.q_table
    for state, actions in sorted(q_learning.q_table.items())[:10]:
        if count >= 8:
            break
        
        state_text = f"S({state[0]},{state[1]}): "
        action_names = ["←", "—", "→"]
        for action in range(3):
            value = actions.get(action, 0.0)
            state_text += f"{action_names[action]}={value:.2f} "
        
        text_surface = font_small.render(state_text, True, BLACK)
        screen.blit(text_surface, (x + 10, y_offset))
        y_offset += 20
        count += 1

def main():
    """Основная функция"""
    # Создание робота
    robot = Robot(WIDTH // 2, HEIGHT // 2 + 100)
    
    # Создание Q-обучения
    q_learning = QLearning(n_angle_bins=10, n_velocity_bins=10, n_actions=3)
    
    # Попытка загрузки сохраненной Q-таблицы
    print("Попытка загрузки Q-таблицы...")
    q_learning.load_q_table()
    
    # Переменные обучения
    episode = 0
    total_reward = 0
    steps = 0
    max_steps = 1000
    done = False
    training = True
    paused = False
    
    print("\n=== СТАБИЛИЗАЦИЯ ОБРАЩЕННОГО МАЯТНИКА ===")
    print("Q-обучение с подкреплением")
    print("\nУправление:")
    print("ПРОБЕЛ - пауза/продолжить")
    print("S - сохранить Q-таблицу")
    print("R - сброс робота")
    print("T - переключить обучение/демонстрацию")
    print("ESC - выход\n")
    
    # Главный цикл
    running = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                    print(f"{'Пауза' if paused else 'Продолжение'}")
                elif event.key == pygame.K_s:
                    q_learning.save_q_table()
                    q_learning.save_history()
                elif event.key == pygame.K_r:
                    robot.reset()
                    done = False
                    total_reward = 0
                    steps = 0
                    print("Робот сброшен")
                elif event.key == pygame.K_t:
                    training = not training
                    print(f"Режим: {'Обучение' if training else 'Демонстрация'}")
        
        if not paused:
            if not done:
                # Получение состояния
                state = robot.get_state()
                
                # Выбор действия
                action = q_learning.get_action(state, training)
                
                # Применение действия
                robot.apply_action(action)
                
                # Обновление физики
                done = robot.update()
                
                # Получение награды
                reward = q_learning.get_reward(robot)
                total_reward += reward
                
                # Получение нового состояния
                next_state = robot.get_state()
                
                # Обновление Q-таблицы
                if training:
                    q_learning.update(state, action, reward, next_state, done)
                
                steps += 1
                
                # Автоматический сброс при падении или превышении шагов
                if done or steps >= max_steps:
                    if training:
                        episode += 1
                        q_learning.episode_count = episode
                        
                        # Вывод статистики каждые 10 эпизодов
                        if episode % 10 == 0:
                            stats = q_learning.get_q_table_stats()
                            print(f"Эпизод {episode}, Награда: {total_reward:.2f}, "
                                  f"Epsilon: {q_learning.epsilon:.3f}, "
                                  f"Q-состояний: {stats['states']}")
                    
                    # Сброс для нового эпизода
                    robot.reset()
                    done = False
                    total_reward = 0
                    steps = 0
        
        # Очистка экрана
        screen.fill(WHITE)
        
        # Заголовок
        font_title = pygame.font.Font(None, 36)
        title = font_title.render("Стабилизация обращенного маятника", True, BLACK)
        screen.blit(title, (WIDTH // 2 - title.get_width() // 2, 10))
        
        subtitle = pygame.font.Font(None, 24)
        mode_text = "Обучение" if training else "Демонстрация"
        sub = subtitle.render(f"Q-обучение с подкреплением (Режим: {mode_text})", True, DARK_GRAY)
        screen.blit(sub, (WIDTH // 2 - sub.get_width() // 2, 45))
        
        # Отрисовка робота
        draw_robot(screen, robot)
        
        # Отрисовка земли
        pygame.draw.line(screen, BLACK, (0, robot.y + robot.platform_height // 2 + 15),
                        (WIDTH, robot.y + robot.platform_height // 2 + 15), 3)
        
        # Отрисовка информационной панели
        draw_info_panel(screen, 10, 80, robot, q_learning, episode, total_reward)
        
        # Отрисовка легенды
        draw_legend(screen, 10, 450)
        
        # Отрисовка визуализации Q-таблицы
        draw_q_table_visualization(screen, q_learning, WIDTH - 320, 80, 300, 200)
        
        # Индикатор паузы
        if paused:
            pause_font = pygame.font.Font(None, 72)
            pause_text = pause_font.render("ПАУЗА", True, RED)
            screen.blit(pause_text, (WIDTH // 2 - pause_text.get_width() // 2, HEIGHT // 2))
        
        pygame.display.flip()
        clock.tick(FPS)
    
    # Сохранение при выходе
    print("\nСохранение данных...")
    q_learning.save_q_table()
    q_learning.save_history()
    print("Программа завершена.")
    
    pygame.quit()

if __name__ == "__main__":
    main()
