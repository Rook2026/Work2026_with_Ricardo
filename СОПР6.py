"""
Практическое занятие №6.
Технология ассоциативной памяти для планирования движений
манипуляционного робота (РТУ МИРЭА)

Запуск:
  python main.py
Управление:
  Клавиша 1  — заполнить ассоциативную память (N=100 записей)
  Клавиша 2  — N=300 записей
  Клавиша 3  — N=1000 записей
  ЛКМ        — переместить концевой элемент в точку курсора
  ESC        — выход
"""

import pygame
from pygame.locals import *
import sys, math
import numpy as np

# ─── Размер окна ──────────────────────────────────────────────────────────────
WIDTH, HEIGHT = 800, 600
FPS = 30


# ─── class Link ───────────────────────────────────────────────────────────────
class Link:
    """Звено манипулятора."""
    def __init__(self, L, alpha=0.0):
        self.L     = L              # длина звена
        self.alpha = alpha          # локальный угол поворота (рад)
        self.P1    = np.zeros(2)    # координаты точки основания
        self.Angle = 0.0            # глобальный угол поворота (рад)
        self.P2    = np.zeros(2)    # координаты конечной точки

    def calc(self):
        """Прямая кинематика одного звена."""
        s, c = math.sin(self.Angle), math.cos(self.Angle)
        self.P2 = self.P1 + self.L * np.array([c, s])

    def draw(self, screen):
        """Отрисовка звена: отрезок + точки сочленений."""
        pygame.draw.line(screen, (0, 0, 0),
                         self.P1.astype(int), self.P2.astype(int), 3)
        pygame.draw.circle(screen, (0, 0, 200),
                           self.P1.astype(int), 5)
        pygame.draw.circle(screen, (0, 0, 200),
                           self.P2.astype(int), 5)


# ─── class RobotManipulator ───────────────────────────────────────────────────
class RobotManipulator:
    """Многозвенный ангулярный манипуляционный робот."""

    def __init__(self, lengths):
        self.lengths = lengths
        self.links   = [Link(lengths[i], 0.3) for i in range(len(lengths))]
        self.endPos  = np.zeros(2)

    def setBasePos(self, pos):
        """Задать позицию основания."""
        self.links[0].P1 = np.array(pos, dtype=float)

    def setAngles(self, angles):
        """Прямая задача кинематики: задать углы и пересчитать."""
        for i in range(len(self.links)):
            self.links[i].alpha = angles[i]
        self.calc()
        return self.endPos

    def calc(self):
        """Полный расчёт кинематики цепи звеньев."""
        for i, lnk in enumerate(self.links):
            lnk.Angle = lnk.alpha
            if i > 0:
                lnk.Angle += self.links[i - 1].Angle
                lnk.P1 = self.links[i - 1].P2
            lnk.calc()
        self.endPos = self.links[-1].P2

    def draw(self, screen):
        """Отрисовка манипулятора."""
        for lnk in self.links:
            lnk.draw(screen)
        self._draw_basement(screen)

    def _draw_basement(self, screen):
        """Отрисовка основания (подставки)."""
        pC = self.links[0].P1.astype(int)
        pL = [pC[0] - 15, pC[1]]
        pR = [pC[0] + 15, pC[1]]
        pygame.draw.line(screen, (0, 0, 0), pL, pR, 3)
        for p in [pL, pC, pR]:
            pygame.draw.line(screen, (0, 0, 0), p, [p[0] - 6, p[1] + 8], 2)


# ─── class Obstacle ───────────────────────────────────────────────────────────
class Obstacle:
    """Круговое препятствие (запретная область)."""

    def __init__(self, x, y, R):
        self.x, self.y, self.R = x, y, R

    def contains(self, x, y) -> bool:
        """Проверка принадлежности точки запретной области."""
        return float(np.linalg.norm(np.array([x, y]) - [self.x, self.y])) < self.R

    def draw(self, screen):
        """Отрисовка препятствия: красная окружность."""
        pygame.draw.circle(screen, (220, 40, 40),
                           (int(self.x), int(self.y)), int(self.R), 2)


# ─── class AssociativeMemory ──────────────────────────────────────────────────
class AssociativeMemory:
    """
    Ассоциативная память табличного типа.
    Хранит записи {α1, α2, x, y} и реализует:
      - ПЗК (findForward): по углам → координаты
      - ОЗК (findInverse): по координатам → углы
    """

    def __init__(self, numAngles: int, numCoords: int):
        self.table     = []
        self.numAngles = numAngles
        self.numCoords = numCoords

    def dist(self, v1, v2) -> float:
        """Евклидово расстояние между двумя векторами."""
        return float(np.linalg.norm(np.subtract(v1, v2)))

    def takeCoords(self, record):
        """Извлечь консеквентные координаты из записи."""
        return record[self.numAngles:]

    def takeAngles(self, record):
        """Извлечь антецедентные углы из записи."""
        return record[:self.numAngles]

    def addRecord(self, angles, coords, eps: float):
        """
        Добавление записи в таблицу с проверкой уникальности:
        запись не добавляется, если в таблице уже есть запись
        с координатами, отличающимися менее чем на eps.
        """
        duplicates = [r for r in self.table
                      if self.dist(coords, self.takeCoords(r)) < eps]
        if len(duplicates) == 0:
            self.table.append([*angles, *coords])

    def findForward(self, angles):
        """ПЗК через ассоциативную память: ближайшая запись по углам."""
        dd = [self.dist(angles, self.takeAngles(r)) for r in self.table]
        return self.takeCoords(self.table[int(np.argmin(dd))])

    def findInverse(self, coords):
        """
        ОЗК через ассоциативную память: ближайшая запись по координатам.
        Если целевая точка попадает в препятствие, ищется ближайшая
        допустимая запись (модифицированная целевая точка).
        """
        dd = [self.dist(coords, self.takeCoords(r)) for r in self.table]
        return self.takeAngles(self.table[int(np.argmin(dd))])

    def findInverseWithObstacle(self, coords, obstacle: Obstacle):
        """
        ОЗК с учётом препятствия (формула 6.4):
        если целевая точка в препятствии — ищем ближайшую допустимую.
        """
        if not obstacle.contains(*coords):
            return self.findInverse(coords)
        # Фильтрация: только записи вне препятствия
        allowed = [r for r in self.table
                   if not obstacle.contains(*self.takeCoords(r))]
        if not allowed:
            return self.findInverse(coords)
        dd = [self.dist(coords, self.takeCoords(r)) for r in allowed]
        return self.takeAngles(allowed[int(np.argmin(dd))])

    def meanError(self, robot: 'RobotManipulator') -> float:
        """
        Средняя ошибка ОЗК: для каждой записи берём координаты,
        запрашиваем ОЗК, устанавливаем углы и меряем расстояние до эталона.
        """
        errors = []
        for rec in self.table:
            coords = self.takeCoords(rec)
            angles = self.findInverse(coords)
            robot.setAngles(angles)
            err = self.dist(robot.endPos, coords)
            errors.append(err)
        return float(np.mean(errors)) if errors else 0.0

    def __len__(self):
        return len(self.table)


# ─── Helpers ─────────────────────────────────────────────────────────────────
def create_manip(base_x=400, base_y=450):
    """Создать двухзвенный манипулятор."""
    r = RobotManipulator([130, 90])
    r.setBasePos([base_x, base_y])
    return r


def fill_memory(am: AssociativeMemory, robot: RobotManipulator,
                obstacle: Obstacle, n: int, avoid: bool = False):
    """Заполнить ассоциативную память N случайными позами."""
    r_tmp = create_manip(robot.links[0].P1[0], robot.links[0].P1[1])
    added = 0
    attempts = 0
    while added < n and attempts < n * 20:
        attempts += 1
        angles = [np.random.uniform(0, 2 * math.pi)
                  for _ in r_tmp.links]
        r_tmp.setAngles(angles)
        if avoid and obstacle.contains(*r_tmp.endPos):
            continue
        am.addRecord(angles, list(r_tmp.endPos), 1.0)
        added += 1
    return added


def draw_hud(screen, font, am, obstacle, error, mode_avoid, base_x, base_y):
    """HUD нижней панели."""
    pygame.draw.rect(screen, (230, 235, 245), (0, HEIGHT - 80, WIDTH, 80))
    pygame.draw.line(screen, (150, 160, 190), (0, HEIGHT - 80), (WIDTH, HEIGHT - 80), 1)

    n_rec  = len(am) if am else 0
    err_s  = f"{error:.2f}" if error > 0 else "—"
    avoid_s = "ДА" if mode_avoid else "нет"
    lines = [
        f"Записей в памяти: {n_rec}   Средняя ошибка ОЗК: {err_s} px   "
        f"Учёт препятствий: {avoid_s}",
        f"Основание: ({int(base_x)}, {int(base_y)})   "
        f"Клавиши: 1=N100  2=N300  3=N1000  O=препятствие вкл/выкл  ESC=выход",
        "ЛКМ — задать целевую точку.  Красная точка — цель,  синие — сочленения.",
    ]
    cols = [(40,50,80),(80,90,120),(100,110,140)]
    for i, (line, col) in enumerate(zip(lines, cols)):
        screen.blit(font.render(line, True, col), (8, HEIGHT - 76 + i * 22))


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Ассоциативная память — Манипулятор")
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont("monospace", 12)
    font_b = pygame.font.SysFont("monospace", 14, bold=True)

    BASE_X, BASE_Y = 400, 450
    robot    = create_manip(BASE_X, BASE_Y)
    obstacle = Obstacle(550, 280, 60)
    am: AssociativeMemory = None
    target   = None
    error    = 0.0
    mode_avoid = False

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT or (
               event.type == KEYDOWN and event.key == K_ESCAPE):
                running = False

            # Заполнение памяти
            if event.type == KEYDOWN:
                n_map = {K_1: 100, K_2: 300, K_3: 1000}
                if event.key in n_map:
                    n = n_map[event.key]
                    am = AssociativeMemory(2, 2)
                    added = fill_memory(am, robot, obstacle, n, avoid=mode_avoid)
                    error = am.meanError(create_manip(BASE_X, BASE_Y))
                    print(f"Записей: {added}/{n}  Средняя ошибка: {error:.2f} px")

                if event.key == K_o:
                    mode_avoid = not mode_avoid
                    # Пересоздать память с новым режимом
                    if am is not None:
                        n_old = len(am)
                        am = AssociativeMemory(2, 2)
                        fill_memory(am, robot, obstacle, n_old, avoid=mode_avoid)
                        error = am.meanError(create_manip(BASE_X, BASE_Y))

            # Клик мышью — задать целевую точку
            if event.type == MOUSEBUTTONUP and am is not None:
                target = list(pygame.mouse.get_pos())
                if mode_avoid:
                    angles = am.findInverseWithObstacle(target, obstacle)
                else:
                    angles = am.findInverse(target)
                robot.setAngles(angles)

        # ── Отрисовка ────────────────────────────────────────────────────────
        screen.fill((245, 248, 255))

        # Сетка
        for gx in range(0, WIDTH, 40):
            pygame.draw.line(screen, (220, 225, 235), (gx, 0), (gx, HEIGHT - 80))
        for gy in range(0, HEIGHT - 80, 40):
            pygame.draw.line(screen, (220, 225, 235), (0, gy), (WIDTH, gy))

        obstacle.draw(screen)
        robot.calc()
        robot.draw(screen)

        # Целевая точка
        if target:
            pygame.draw.circle(screen, (220, 40, 40),
                               (int(target[0]), int(target[1])), 6)
            pygame.draw.circle(screen, (150, 20, 20),
                               (int(target[0]), int(target[1])), 6, 1)

        # Концевой элемент
        ep = robot.endPos.astype(int)
        pygame.draw.circle(screen, (0, 160, 80), tuple(ep), 7)

        # Подсказка при пустой памяти
        if am is None:
            hint = font_b.render(
                "Нажмите 1 / 2 / 3 для заполнения ассоциативной памяти",
                True, (100, 110, 160))
            screen.blit(hint, (WIDTH//2 - hint.get_width()//2, 20))

        # Точки памяти (маленькие)
        if am is not None:
            for rec in am.table:
                cx, cy = int(rec[2]), int(rec[3])
                if 0 <= cx < WIDTH and 0 <= cy < HEIGHT - 80:
                    pygame.draw.circle(screen, (180, 200, 230), (cx, cy), 2)

        draw_hud(screen, font, am, obstacle, error, mode_avoid, BASE_X, BASE_Y)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()