import pygame
import sys
from collections import deque
import copy

# Инициализация Pygame
pygame.init()

# Параметры окна
WIDTH, HEIGHT = 800, 600
FPS = 30
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Planning")
clock = pygame.time.Clock()

font = pygame.font.Font(None, 24)
big_font = pygame.font.Font(None, 36)


class Robot:
    def __init__(self, x):
        self.x = x
        self.y = HEIGHT // 2

    def draw(self):
        pygame.draw.circle(screen, RED, (self.x, self.y), 20)
        pygame.draw.circle(screen, BLACK, (self.x, self.y), 20, 2)

    def copy(self):
        return Robot(self.x)


class Obj:
    def __init__(self, name, x, y_offset):
        self.name = name
        self.x = x
        self.y = HEIGHT // 2 + y_offset

    def draw(self):
        pygame.draw.rect(screen, BLUE, (self.x - 15, self.y - 15, 30, 30))
        pygame.draw.rect(screen, BLACK, (self.x - 15, self.y - 15, 30, 30), 2)
        label = font.render(self.name, True, WHITE)
        screen.blit(label, (self.x - 8, self.y - 8))

    def copy(self):
        return Obj(self.name, self.x, self.y - HEIGHT // 2)


def get_state_key(robot, objects):
    """Возвращает кортеж состояния"""
    obj_positions = tuple(obj.x for obj in objects)
    return (robot.x, obj_positions)


def find_nearest_object(robot, objects):
    """Находит ближайший к роботу объект"""
    if not objects:
        return None
    return min(objects, key=lambda obj: abs(obj.x - robot.x))


def main():
    # Начальные параметры
    robot = Robot(400)
    objects = [
        Obj("A", 100, -40),
        Obj("B", 200, -20),
        Obj("C", 300, 0),
        Obj("D", 400, 20),
    ]

    # Целевое состояние для отображения
    target_obj_positions = (400, 300, 200, 100)

    # Сообщение о действии
    action_message = ""
    message_timer = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                action_message = ""

                if event.key == pygame.K_RIGHT:
                    robot.x += 50
                    action_message = "Робот переместился вправо"
                    message_timer = 60

                elif event.key == pygame.K_LEFT:
                    robot.x -= 50
                    action_message = "Робот переместился влево"
                    message_timer = 60

                elif event.key == pygame.K_PERIOD:  # Клавиша >
                    nearest = find_nearest_object(robot, objects)
                    if nearest:
                        nearest.x += 50
                        action_message = f"Объект {nearest.name} перемещён вправо"
                        message_timer = 60
                    else:
                        action_message = "Нет объектов"
                        message_timer = 60

                elif event.key == pygame.K_COMMA:  # Клавиша <
                    nearest = find_nearest_object(robot, objects)
                    if nearest:
                        nearest.x -= 50
                        action_message = f"Объект {nearest.name} перемещён влево"
                        message_timer = 60
                    else:
                        action_message = "Нет объектов"
                        message_timer = 60

                elif event.key == pygame.K_r:
                    # Сброс в начальное состояние
                    robot.x = 400
                    objects[0].x = 100
                    objects[1].x = 200
                    objects[2].x = 300
                    objects[3].x = 400
                    action_message = "Сброс в начальное состояние"
                    message_timer = 60

                elif event.key == pygame.K_SPACE:
                    # Проверка достижения цели
                    current_positions = (objects[0].x, objects[1].x, objects[2].x, objects[3].x)
                    if current_positions == target_obj_positions:
                        action_message = "ЦЕЛЬ ДОСТИГНУТА! Объекты в порядке D, C, B, A"
                    else:
                        action_message = f"Цель не достигнута. Нужно: D=400, C=300, B=200, A=100"
                    message_timer = 120

        # Уменьшение таймера сообщения
        if message_timer > 0:
            message_timer -= 1
        else:
            action_message = ""

        # Отрисовка
        screen.fill(WHITE)

        # Рисуем сетку
        for x in range(0, WIDTH, 50):
            pygame.draw.line(screen, GRAY, (x, 0), (x, HEIGHT), 1)

        # Рисуем объекты
        for obj in objects:
            obj.draw()

        # Рисуем робота
        robot.draw()

        # Отображение информации
        y = 10
        info_texts = [
            "УПРАВЛЕНИЕ:",
            "  → / ← - перемещение робота",
            "  > / < - перемещение ближайшего объекта",
            "  R - сброс",
            "  SPACE - проверка цели",
            "",
            f"Robot x: {robot.x}",
            f"A x: {objects[0].x}",
            f"B x: {objects[1].x}",
            f"C x: {objects[2].x}",
            f"D x: {objects[3].x}",
            "",
            f"ЦЕЛЬ: D=400, C=300, B=200, A=100",
        ]

        for text in info_texts:
            rendered = font.render(text, True, BLACK)
            screen.blit(rendered, (10, y))
            y += 22

        # Отображение сообщения о действии
        if action_message:
            msg_rendered = big_font.render(action_message, True, GREEN if "ЦЕЛЬ" in action_message else RED)
            msg_rect = msg_rendered.get_rect(center=(WIDTH // 2, HEIGHT - 50))
            screen.blit(msg_rendered, msg_rect)

        # Подсветка ближайшего объекта
        nearest = find_nearest_object(robot, objects)
        if nearest:
            pygame.draw.circle(screen, GREEN, (nearest.x, nearest.y), 25, 3)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()