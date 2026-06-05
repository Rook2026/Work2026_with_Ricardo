"""
Самообучение группы роботов при кооперативной транспортировке крупногабаритного объекта
Два робота совместно несут прямоугольную балку и обучаются методом Q-learning
довезти её к целевой точке.
"""

import pygame
import sys
import math
import random
import json
from typing import List, Tuple, Dict

# ─── Constants ────────────────────────────────────────────────────────────────
WINDOW_W, WINDOW_H = 800, 600
FPS          = 30
BAR_LENGTH   = 120      # длина балки (px)
BAR_WIDTH    = 14       # ширина балки (px)
ROBOT_RADIUS = 14
ROBOT_SPEED  = 1.8      # скорость движения

# Начальные условия (вход программы)
INIT_X   = 400.0
INIT_Y   = 300.0
INIT_ANGLE_DEG = 5.0    # начальный угол балки

# Целевая точка
TARGET_X = 650.0
TARGET_Y = 150.0

# Q-learning
ALPHA    = 0.15     # скорость обучения
GAMMA    = 0.90     # дисконт
EPSILON  = 0.25     # вероятность случайного действия
N_ANGLE_BINS = 8    # дискретизация углов

# Цвета
CLR_BG      = (18,  24,  38)
CLR_GRID    = (28,  36,  55)
CLR_BAR     = (180, 160, 80)
CLR_BAR_OUT = (220, 200, 100)
CLR_R1      = (80,  200, 130)
CLR_R2      = (80,  150, 220)
CLR_TARGET  = (220,  80,  80)
CLR_TRAIL   = (60,  100,  80)
CLR_TEXT    = (200, 220, 255)
CLR_PANEL   = (12,  18,  30)


# ─── Helpers ──────────────────────────────────────────────────────────────────
def angle_diff(a: float, b: float) -> float:
    """Минимальная разница углов [-π, π]"""
    d = (a - b) % (2 * math.pi)
    if d > math.pi:
        d -= 2 * math.pi
    return d

def discretize(val: float, lo: float, hi: float, n: int) -> int:
    """Квантизация непрерывного значения в индекс 0..n-1"""
    idx = int((val - lo) / (hi - lo) * n)
    return max(0, min(n - 1, idx))


# ─── class Robot ─────────────────────────────────────────────────────────────
class Robot:
    """
    Мобильный робот, определяемый координатами (x, y, alpha) и скоростью V.
    alpha — угол курса в радианах.
    """
    def __init__(self, x: float, y: float, alpha: float = 0.0, color=(80,200,130)):
        self.x     = x
        self.y     = y
        self.alpha = alpha      # текущий угол курса (рад)
        self.color = color
        self.trail: List[Tuple[float,float]] = []

    def move(self, turn: float):
        """
        Движение: поворот на turn (рад), затем шаг вперёд.
        turn ∈ {-π/4, 0, +π/4} — дискретное действие.
        """
        self.alpha += turn
        self.x += ROBOT_SPEED * math.cos(self.alpha)
        self.y += ROBOT_SPEED * math.sin(self.alpha)
        # Границы окна
        self.x = max(ROBOT_RADIUS, min(WINDOW_W - ROBOT_RADIUS, self.x))
        self.y = max(ROBOT_RADIUS, min(WINDOW_H - ROBOT_RADIUS, self.y))
        self.trail.append((self.x, self.y))
        if len(self.trail) > 800:
            self.trail.pop(0)

    def draw(self, surface: pygame.Surface):
        # След
        if len(self.trail) > 1:
            for i in range(1, len(self.trail)):
                t = i / len(self.trail)
                c = tuple(int(CLR_TRAIL[k] * t) for k in range(3))
                pygame.draw.line(surface, c,
                    (int(self.trail[i-1][0]), int(self.trail[i-1][1])),
                    (int(self.trail[i][0]),   int(self.trail[i][1])), 1)
        # Тело
        cx, cy = int(self.x), int(self.y)
        pygame.draw.circle(surface, self.color, (cx, cy), ROBOT_RADIUS)
        pygame.draw.circle(surface, (255,255,255), (cx, cy), ROBOT_RADIUS, 1)
        # Направление
        ex = cx + (ROBOT_RADIUS-2) * math.cos(self.alpha)
        ey = cy + (ROBOT_RADIUS-2) * math.sin(self.alpha)
        pygame.draw.line(surface, (255,255,200), (cx,cy), (int(ex),int(ey)), 2)


# ─── class Bar ───────────────────────────────────────────────────────────────
class Bar:
    """
    Прямоугольная балка, закреплённая над двумя роботами.
    Центр балки — среднее положение роботов.
    Угол — угол вектора от r1 к r2.
    """
    def __init__(self):
        self.cx    = INIT_X
        self.cy    = INIT_Y
        self.angle = math.radians(INIT_ANGLE_DEG)  # угол балки (рад)
        self.prev_dist = None

    def update(self, r1: Robot, r2: Robot):
        """Пересчёт кинематики балки по положениям роботов."""
        self.cx    = (r1.x + r2.x) / 2
        self.cy    = (r1.y + r2.y) / 2
        self.angle = math.atan2(r2.y - r1.y, r2.x - r1.x)

    def dist_to_target(self) -> float:
        return math.hypot(self.cx - TARGET_X, self.cy - TARGET_Y)

    def angle_to_target(self) -> float:
        """Угол от центра балки до целевой точки."""
        return math.atan2(TARGET_Y - self.cy, TARGET_X - self.cx)

    def draw(self, surface: pygame.Surface):
        # Вычисляем 4 угла прямоугольника
        ca, sa = math.cos(self.angle), math.sin(self.angle)
        hl, hw = BAR_LENGTH / 2, BAR_WIDTH / 2
        corners = [
            (self.cx + ca*hl - sa*hw, self.cy + sa*hl + ca*hw),
            (self.cx + ca*hl + sa*hw, self.cy + sa*hl - ca*hw),
            (self.cx - ca*hl + sa*hw, self.cy - sa*hl - ca*hw),
            (self.cx - ca*hl - sa*hw, self.cy - sa*hl + ca*hw),
        ]
        pts = [(int(x), int(y)) for x,y in corners]
        pygame.draw.polygon(surface, CLR_BAR, pts)
        pygame.draw.polygon(surface, CLR_BAR_OUT, pts, 2)


# ─── State / Action encoding ──────────────────────────────────────────────────
# Состояние: (A0_bin, B0_bin)
#   A0 = угол балки относительно горизонтали  [-π, π]
#   B0 = угол от перпендикуляра к балке на целевую точку [-π, π]
# Действие: (a1_idx, a2_idx), каждый ∈ {0,1,2} = {-π/4, 0, +π/4}

TURNS = [-math.pi/4, 0.0, math.pi/4]   # дискретные действия

def get_state(bar: Bar) -> Tuple[int,int]:
    """
    Возвращает дискретное состояние (A0_bin, B0_bin).
    A0 = угол балки; B0 = угол до цели минус перпендикуляр к балке.
    """
    A0 = bar.angle                                          # угол балки
    perp = bar.angle + math.pi / 2                         # перпендикуляр к балке
    B0 = angle_diff(bar.angle_to_target(), perp)            # отклонение от перпендикуляра

    a0_bin = discretize(A0 + math.pi, 0, 2*math.pi, N_ANGLE_BINS)
    b0_bin = discretize(B0 + math.pi, 0, 2*math.pi, N_ANGLE_BINS)
    return (a0_bin, b0_bin)

def get_state_continuous(bar: Bar) -> Tuple[float,float]:
    A0 = bar.angle
    perp = bar.angle + math.pi / 2
    B0 = angle_diff(bar.angle_to_target(), perp)
    return (A0, B0)

def get_action_angles(r1: Robot, r2: Robot) -> Tuple[float,float]:
    """Непрерывные углы курса обоих роботов."""
    return (r1.alpha, r2.alpha)

def encode_action(a1_idx: int, a2_idx: int) -> int:
    return a1_idx * len(TURNS) + a2_idx

def decode_action(action: int) -> Tuple[int,int]:
    return divmod(action, len(TURNS))

N_ACTIONS = len(TURNS) ** 2   # 9 комбинаций


# ─── Q-table ──────────────────────────────────────────────────────────────────
class QTable:
    """Таблица оценок Q(s, a)."""
    def __init__(self):
        self.table: Dict[Tuple, List[float]] = {}

    def _key(self, state):
        return tuple(state)

    def get(self, state, action: int) -> float:
        return self.table.get(self._key(state), [0.0]*N_ACTIONS)[action]

    def get_all(self, state) -> List[float]:
        return self.table.get(self._key(state), [0.0]*N_ACTIONS)[:]

    def update(self, state, action: int, value: float):
        k = self._key(state)
        if k not in self.table:
            self.table[k] = [0.0] * N_ACTIONS
        self.table[k][action] = value

    def best_action(self, state) -> int:
        return int(max(range(N_ACTIONS), key=lambda a: self.get(state, a)))

    def display_rows(self, n=8) -> List[str]:
        """Возвращает n строк для отображения таблицы на экране."""
        rows = []
        for k, vals in list(self.table.items())[:n]:
            best = int(max(range(N_ACTIONS), key=lambda a: vals[a]))
            a1, a2 = decode_action(best)
            rows.append(f"s{k} a=({a1},{a2}) Q={vals[best]:.2f}")
        return rows


# ─── Reinforcement function ───────────────────────────────────────────────────
def compute_reward(bar: Bar) -> float:
    """
    Функция подкрепления R:
      - Положительная составляющая: обратно пропорциональна производной
        дальности до цели (уменьшение расстояния даёт награду).
      - Отрицательная составляющая: штраф за угловое рассогласование балки
        с направлением на цель.
    """
    dist = bar.dist_to_target()
    angle_to_tgt = bar.angle_to_target()
    angle_err = abs(angle_diff(bar.angle, angle_to_tgt))

    # Награда за приближение к цели
    if bar.prev_dist is not None:
        ddist = bar.prev_dist - dist       # > 0 если приближаемся
        r_dist = ddist * 5.0
    else:
        r_dist = 0.0

    # Штраф за угловое рассогласование (нормировано на π)
    r_angle = -(angle_err / math.pi) * 0.5

    # Бонус за достижение цели
    r_goal = 10.0 if dist < 30 else 0.0

    bar.prev_dist = dist
    return r_dist + r_angle + r_goal


# ─── Action selection (ε-greedy) ─────────────────────────────────────────────
def choose_action(qtable: QTable, state, epsilon: float) -> int:
    if random.random() < epsilon:
        return random.randint(0, N_ACTIONS - 1)
    return qtable.best_action(state)


# ─── Journal (лог) ────────────────────────────────────────────────────────────
class Journal:
    """Журнал истории перемещений в формате {A0, B0, A1, A2}."""
    def __init__(self):
        self.entries: List[Tuple] = []

    def record(self, A0: float, B0: float, A1: float, A2: float):
        self.entries.append((round(A0,4), round(B0,4),
                             round(A1,4), round(A2,4)))

    def save(self, path: str = "robot_coop_log.txt"):
        with open(path, "w") as f:
            f.write("A0,B0,A1,A2\n")
            for e in self.entries:
                f.write(",".join(map(str, e)) + "\n")
        return path


# ─── Drawing helpers ──────────────────────────────────────────────────────────
def draw_target(surface: pygame.Surface, font):
    tx, ty = int(TARGET_X), int(TARGET_Y)
    pygame.draw.circle(surface, CLR_TARGET, (tx, ty), 18, 2)
    pygame.draw.line(surface, CLR_TARGET, (tx-12,ty), (tx+12,ty), 1)
    pygame.draw.line(surface, CLR_TARGET, (tx,ty-12), (tx,ty+12), 1)
    lbl = font.render("Цель", True, CLR_TARGET)
    surface.blit(lbl, (tx+10, ty-8))

def draw_panel(surface: pygame.Surface, font, qtable: QTable,
               bar: Bar, step: int, reward: float):
    """Информационная панель справа."""
    px = 0
    pygame.draw.rect(surface, CLR_PANEL, (px, WINDOW_H-110, WINDOW_W, 110))
    pygame.draw.line(surface, (50,70,110), (px, WINDOW_H-110), (WINDOW_W, WINDOW_H-110), 1)

    A0, B0 = get_state_continuous(bar)
    dist   = bar.dist_to_target()
    info = [
        f"Шаг: {step}   Дист: {dist:.1f}   Награда: {reward:.3f}",
        f"A0 (угол балки): {math.degrees(A0):.1f}°   "
        f"B0 (откл. от перп.): {math.degrees(B0):.1f}°",
        f"Q-состояний: {len(qtable.table)}   ε={EPSILON:.2f}   "
        f"α={ALPHA:.2f}   γ={GAMMA:.2f}",
        "SPACE — пауза   R — сброс   S — сохранить лог   ESC — выход",
    ]
    for i, line in enumerate(info):
        lbl = font.render(line, True, CLR_TEXT if i < 3 else (100,120,160))
        surface.blit(lbl, (8, WINDOW_H - 104 + i*24))

def draw_qtable(surface: pygame.Surface, font_sm, qtable: QTable):
    """Мини-таблица Q в правом верхнем углу."""
    rows = qtable.display_rows(6)
    if not rows:
        return
    x0, y0 = WINDOW_W - 250, 8
    pygame.draw.rect(surface, CLR_PANEL, (x0-4, y0-4, 248, len(rows)*16+10))
    pygame.draw.rect(surface, (40,60,100), (x0-4, y0-4, 248, len(rows)*16+10), 1)
    hdr = font_sm.render("Q-таблица (лучшие состояния)", True, (120,160,220))
    surface.blit(hdr, (x0, y0))
    for i, row in enumerate(rows):
        lbl = font_sm.render(row, True, (160,200,160))
        surface.blit(lbl, (x0, y0 + 16 + i*14))


# ─── Main ─────────────────────────────────────────────────────────────────────
def make_robots():
    """Создать два робота по краям балки согласно начальным условиям."""
    angle = math.radians(INIT_ANGLE_DEG)
    hl    = BAR_LENGTH / 2
    r1 = Robot(INIT_X - hl*math.cos(angle),
               INIT_Y - hl*math.sin(angle), angle, CLR_R1)
    r2 = Robot(INIT_X + hl*math.cos(angle),
               INIT_Y + hl*math.sin(angle), angle + math.pi, CLR_R2)
    return r1, r2

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Кооперативная транспортировка — Q-learning")
    clock = pygame.time.Clock()

    font_sm = pygame.font.SysFont("monospace", 11)
    font_md = pygame.font.SysFont("monospace", 13)

    r1, r2   = make_robots()
    bar      = Bar()
    bar.update(r1, r2)

    qtable  = QTable()
    journal = Journal()

    step   = 0
    paused = False
    last_reward = 0.0
    state  = get_state(bar)
    action = choose_action(qtable, state, EPSILON)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit(); sys.exit()
                if event.key == pygame.K_SPACE:
                    paused = not paused
                if event.key == pygame.K_r:
                    r1, r2 = make_robots()
                    bar = Bar(); bar.update(r1, r2)
                    step = 0; last_reward = 0.0
                    state = get_state(bar)
                    action = choose_action(qtable, state, EPSILON)
                if event.key == pygame.K_s:
                    path = journal.save()
                    print(f"Лог сохранён: {path}")

        if not paused:
            step += 1

            # Декодирование действия
            a1_idx, a2_idx = decode_action(action)
            t1 = TURNS[a1_idx]
            t2 = TURNS[a2_idx]

            # Движение роботов
            r1.move(t1)
            r2.move(t2)

            # Обновление кинематики балки
            bar.update(r1, r2)

            # Непрерывные углы для журнала
            A0, B0 = get_state_continuous(bar)
            A1, A2 = get_action_angles(r1, r2)
            journal.record(A0, B0, A1, A2)

            # Вычисление подкрепления
            reward = compute_reward(bar)
            last_reward = reward

            # Q-learning update
            new_state  = get_state(bar)
            new_action = choose_action(qtable, new_state, EPSILON)
            old_q      = qtable.get(state, action)
            best_next  = max(qtable.get_all(new_state))
            new_q      = old_q + ALPHA * (reward + GAMMA * best_next - old_q)
            qtable.update(state, action, new_q)

            state  = new_state
            action = new_action

            # Сброс при достижении цели
            if bar.dist_to_target() < 25:
                r1, r2 = make_robots()
                bar = Bar(); bar.update(r1, r2)
                state  = get_state(bar)
                action = choose_action(qtable, state, EPSILON * 0.5)

        # ── Отрисовка ────────────────────────────────────────────────────────
        screen.fill(CLR_BG)

        # Сетка
        for gx in range(0, WINDOW_W, 40):
            pygame.draw.line(screen, CLR_GRID, (gx,0),(gx,WINDOW_H-110))
        for gy in range(0, WINDOW_H-110, 40):
            pygame.draw.line(screen, CLR_GRID, (0,gy),(WINDOW_W,gy))

        draw_target(screen, font_md)

        # Линия соединения роботов
        pygame.draw.line(screen, (60,80,80),
                         (int(r1.x),int(r1.y)), (int(r2.x),int(r2.y)), 1)

        bar.draw(screen)
        r1.draw(screen)
        r2.draw(screen)

        # Метки роботов
        screen.blit(font_sm.render("R1",True,CLR_R1),(int(r1.x)+16,int(r1.y)-8))
        screen.blit(font_sm.render("R2",True,CLR_R2),(int(r2.x)+16,int(r2.y)-8))

        draw_qtable(screen, font_sm, qtable)
        draw_panel(screen, font_md, qtable, bar, step, last_reward)

        pygame.display.flip()
        clock.tick(FPS)


if __name__ == "__main__":
    main()