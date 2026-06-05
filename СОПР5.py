"""
Тема 1. Реализация алгоритма прогноза ситуаций функционирования мобильного робота
на примере игры «Квадрат».

Идея:
  - Робот движется по полю 800×600.
  - Цель — собрать все «монеты» (квадраты), избегая врагов.
  - Алгоритм прогноза ситуаций: на каждом шаге оценивается вектор ситуации
    {расстояние до ближайшего врага, расстояние до ближайшей монеты, угол до монеты,
     угол до врага} и выбирается действие по таблице прогноза (наивный байес + счётчики).
  - История ситуаций сохраняется в журнал situations.txt.
"""

import pygame
import sys
import math
import random
from typing import List, Tuple, Dict, Optional

# ─── Constants ────────────────────────────────────────────────────────────────
WINDOW_W, WINDOW_H = 800, 600
FPS           = 30
CELL          = 40          # шаг сетки

ROBOT_R       = 14
COIN_R        = 8
ENEMY_R       = 12
ROBOT_SPEED   = 3.0
ENEMY_SPEED   = 1.4

N_COINS       = 8
N_ENEMIES     = 4

# Дискретизация
DIST_BINS     = 5           # бинов расстояния
ANGLE_BINS    = 8           # бинов угла

# Цвета
CLR_BG        = (16, 22, 34)
CLR_GRID      = (26, 34, 52)
CLR_ROBOT     = (80, 210, 130)
CLR_ENEMY     = (210, 70, 60)
CLR_COIN      = (240, 200, 60)
CLR_TRAIL     = (40, 90, 65)
CLR_TEXT      = (200, 220, 255)
CLR_PANEL     = (10, 15, 26)
CLR_PRED_HI   = (80, 210, 130)
CLR_PRED_LO   = (60, 80, 110)
CLR_DANGER    = (210, 90, 60)
CLR_SAFE      = (60, 180, 120)

# Действия
ACTIONS = {
    0: ( 0, -1, "Вверх"),
    1: ( 0,  1, "Вниз"),
    2: (-1,  0, "Влево"),
    3: ( 1,  0, "Вправо"),
    4: ( 0,  0, "Стоп"),
}

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def angle_diff(a, b):
    d = (a - b) % (2 * math.pi)
    return d - 2*math.pi if d > math.pi else d


# ─── class Robot ─────────────────────────────────────────────────────────────
class Robot:
    def __init__(self, x: float, y: float):
        self.x     = x
        self.y     = y
        self.trail: List[Tuple[float,float]] = []
        self.score = 0
        self.alive = True

    def move(self, action: int):
        dx, dy, _ = ACTIONS[action]
        self.x = clamp(self.x + dx * ROBOT_SPEED, ROBOT_R, WINDOW_W - ROBOT_R)
        self.y = clamp(self.y + dy * ROBOT_SPEED, ROBOT_R, WINDOW_H - 110 - ROBOT_R)
        self.trail.append((self.x, self.y))
        if len(self.trail) > 1000:
            self.trail.pop(0)

    def draw(self, surf: pygame.Surface):
        if len(self.trail) > 1:
            for i in range(1, len(self.trail)):
                t = i / len(self.trail)
                c = (int(CLR_TRAIL[0]*t), int(CLR_TRAIL[1]*t), int(CLR_TRAIL[2]*t))
                pygame.draw.line(surf, c,
                    (int(self.trail[i-1][0]), int(self.trail[i-1][1])),
                    (int(self.trail[i][0]),   int(self.trail[i][1])), 1)
        col = CLR_ROBOT if self.alive else (180, 60, 60)
        pygame.draw.circle(surf, col, (int(self.x), int(self.y)), ROBOT_R)
        pygame.draw.circle(surf, (255,255,255), (int(self.x), int(self.y)), ROBOT_R, 1)
        # глаза
        pygame.draw.circle(surf, (255,255,255), (int(self.x)-4, int(self.y)-4), 3)
        pygame.draw.circle(surf, (255,255,255), (int(self.x)+4, int(self.y)-4), 3)
        pygame.draw.circle(surf, (30,30,30),   (int(self.x)-4, int(self.y)-4), 1)
        pygame.draw.circle(surf, (30,30,30),   (int(self.x)+4, int(self.y)-4), 1)


# ─── class Coin ──────────────────────────────────────────────────────────────
class Coin:
    def __init__(self, x: float, y: float):
        self.x, self.y = x, y
        self.collected = False
        self.pulse = random.uniform(0, math.pi*2)

    def draw(self, surf: pygame.Surface, tick: int):
        if self.collected:
            return
        self.pulse += 0.08
        r = COIN_R + int(2 * math.sin(self.pulse))
        pygame.draw.rect(surf, CLR_COIN,
            (int(self.x) - r, int(self.y) - r, 2*r, 2*r))
        pygame.draw.rect(surf, (255, 230, 100),
            (int(self.x) - r, int(self.y) - r, 2*r, 2*r), 2)

    def collides(self, rx: float, ry: float) -> bool:
        return not self.collected and math.hypot(rx-self.x, ry-self.y) < ROBOT_R + COIN_R


# ─── class Enemy ─────────────────────────────────────────────────────────────
class Enemy:
    def __init__(self, x: float, y: float):
        self.x, self.y = x, y
        self.vx = random.choice([-1,1]) * ENEMY_SPEED
        self.vy = random.choice([-1,1]) * ENEMY_SPEED

    def update(self, robot: Robot):
        # Преследование с небольшим шумом
        dx = robot.x - self.x
        dy = robot.y - self.y
        d  = math.hypot(dx, dy) or 1
        self.vx = self.vx*0.8 + (dx/d)*ENEMY_SPEED*0.2 + random.uniform(-0.3,0.3)
        self.vy = self.vy*0.8 + (dy/d)*ENEMY_SPEED*0.2 + random.uniform(-0.3,0.3)
        spd = math.hypot(self.vx, self.vy) or 1
        self.vx = self.vx/spd * ENEMY_SPEED
        self.vy = self.vy/spd * ENEMY_SPEED
        self.x = clamp(self.x + self.vx, ENEMY_R, WINDOW_W - ENEMY_R)
        self.y = clamp(self.y + self.vy, ENEMY_R, WINDOW_H - 110 - ENEMY_R)

    def collides(self, rx: float, ry: float) -> bool:
        return math.hypot(rx-self.x, ry-self.y) < ROBOT_R + ENEMY_R

    def draw(self, surf: pygame.Surface):
        cx, cy = int(self.x), int(self.y)
        # Тело (квадрат-враг)
        pygame.draw.rect(surf, CLR_ENEMY,
            (cx-ENEMY_R, cy-ENEMY_R, 2*ENEMY_R, 2*ENEMY_R), border_radius=4)
        pygame.draw.rect(surf, (240, 100, 80),
            (cx-ENEMY_R, cy-ENEMY_R, 2*ENEMY_R, 2*ENEMY_R), 2, border_radius=4)
        # крестик
        pygame.draw.line(surf, (255,255,255), (cx-5,cy-5),(cx+5,cy+5), 2)
        pygame.draw.line(surf, (255,255,255), (cx+5,cy-5),(cx-5,cy+5), 2)


# ─── Ситуация (вектор признаков) ─────────────────────────────────────────────
def get_situation(robot: Robot, coins: List[Coin],
                  enemies: List[Enemy]) -> Optional[Dict]:
    """
    Вектор ситуации:
      d_coin   — расстояние до ближайшей монеты (дискр.)
      a_coin   — угол до ближайшей монеты (дискр.)
      d_enemy  — расстояние до ближайшего врага (дискр.)
      a_enemy  — угол до ближайшего врага (дискр.)
      danger   — флаг опасности (враг < 80 px)
    """
    active = [c for c in coins if not c.collected]
    if not active:
        return None

    # Ближайшая монета
    coin = min(active, key=lambda c: math.hypot(c.x-robot.x, c.y-robot.y))
    d_coin = math.hypot(coin.x-robot.x, coin.y-robot.y)
    a_coin = math.atan2(coin.y-robot.y, coin.x-robot.x)

    # Ближайший враг
    enemy = min(enemies, key=lambda e: math.hypot(e.x-robot.x, e.y-robot.y))
    d_enemy = math.hypot(enemy.x-robot.x, enemy.y-robot.y)
    a_enemy = math.atan2(enemy.y-robot.y, enemy.x-robot.x)

    max_d = math.hypot(WINDOW_W, WINDOW_H)
    d_coin_bin  = clamp(int(d_coin  / max_d * DIST_BINS),  0, DIST_BINS-1)
    d_enemy_bin = clamp(int(d_enemy / max_d * DIST_BINS),  0, DIST_BINS-1)
    a_coin_bin  = int((a_coin  + math.pi) / (2*math.pi) * ANGLE_BINS) % ANGLE_BINS
    a_enemy_bin = int((a_enemy + math.pi) / (2*math.pi) * ANGLE_BINS) % ANGLE_BINS

    return {
        "d_coin":   d_coin_bin,
        "a_coin":   a_coin_bin,
        "d_enemy":  d_enemy_bin,
        "a_enemy":  a_enemy_bin,
        "danger":   int(d_enemy < 80),
        # сырые значения для отображения
        "d_coin_raw":  d_coin,
        "d_enemy_raw": d_enemy,
        "a_coin_deg":  math.degrees(a_coin),
        "a_enemy_deg": math.degrees(a_enemy),
        "coin_target": (coin.x, coin.y),
        "enemy_pos":   (enemy.x, enemy.y),
    }


# ─── Таблица прогноза ситуаций ────────────────────────────────────────────────
class SituationPredictor:
    """
    Простая таблица подсчёта: для каждой дискретной ситуации
    накапливает счётчики успешных и неудачных действий.
    Прогноз = действие с наибольшим счётчиком успехов.
    """
    def __init__(self):
        # {sit_key: {action: [wins, losses]}}
        self.table: Dict[tuple, Dict[int, List[int]]] = {}

    def _key(self, sit: Dict) -> tuple:
        return (sit["d_coin"], sit["a_coin"],
                sit["d_enemy"], sit["a_enemy"], sit["danger"])

    def record(self, sit: Dict, action: int, success: bool):
        k = self._key(sit)
        if k not in self.table:
            self.table[k] = {a: [0, 0] for a in ACTIONS}
        self.table[k][action][0 if success else 1] += 1

    def predict(self, sit: Dict) -> int:
        """
        Выбор действия: если ситуация известна — лучшее по win/loss;
        иначе — эвристика (к монете, от врага).
        """
        k = self._key(sit)
        if k in self.table:
            def score(a):
                w, l = self.table[k][a]
                return w - l * 2
            return max(ACTIONS, key=score)

        # Эвристика: если опасно — убегаем от врага, иначе — к монете
        if sit["danger"]:
            a_e = sit["a_enemy"]   # направление врага (бин)
            # бежим в противоположную сторону
            opp = (a_e + ANGLE_BINS // 2) % ANGLE_BINS
            mapping = {0:3, 1:3, 2:1, 3:1, 4:2, 5:2, 6:0, 7:0}
            return mapping.get(opp, 4)
        else:
            a_c = sit["a_coin"]
            mapping = {0:3, 1:3, 2:1, 3:1, 4:2, 5:2, 6:0, 7:0}
            return mapping.get(a_c, 4)

    def known_count(self) -> int:
        return len(self.table)

    def top_rows(self, n=6) -> List[str]:
        rows = []
        for k, acts in list(self.table.items())[:n]:
            best = max(acts, key=lambda a: acts[a][0] - acts[a][1]*2)
            w, l = acts[best]
            rows.append(f"sit{k[:3]}.. a={best} W={w} L={l}")
        return rows


# ─── Журнал ───────────────────────────────────────────────────────────────────
class Journal:
    def __init__(self):
        self.rows: List[str] = []

    def record(self, step: int, sit: Dict, action: int, reward: float):
        d_c = sit["d_coin_raw"]
        d_e = sit["d_enemy_raw"]
        a_c = sit["a_coin_deg"]
        a_e = sit["a_enemy_deg"]
        act_name = ACTIONS[action][2]
        self.rows.append(
            f"{step},{d_c:.1f},{a_c:.1f},{d_e:.1f},{a_e:.1f},{act_name},{reward:.2f}"
        )

    def save(self, path="situations.txt"):
        with open(path, "w", encoding="utf-8") as f:
            f.write("step,d_coin,a_coin_deg,d_enemy,a_enemy_deg,action,reward\n")
            f.writelines(r + "\n" for r in self.rows)


# ─── HUD ─────────────────────────────────────────────────────────────────────
def draw_hud(surf, font_sm, font_md, robot, predictor, sit, step, action, lives):
    y0 = WINDOW_H - 108
    pygame.draw.rect(surf, CLR_PANEL, (0, y0, WINDOW_W, 108))
    pygame.draw.line(surf, (40,60,100), (0, y0), (WINDOW_W, y0), 1)

    if sit:
        danger_col = CLR_DANGER if sit["danger"] else CLR_SAFE
        lines = [
            f"Шаг: {step}   Счёт: {robot.score}   Жизни: {lives}   "
            f"Известных ситуаций: {predictor.known_count()}",
            f"d_монеты: {sit['d_coin_raw']:.1f}  a_монеты: {sit['a_coin_deg']:.1f}°  "
            f"d_врага: {sit['d_enemy_raw']:.1f}  a_врага: {sit['a_enemy_deg']:.1f}°",
            f"Действие: {ACTIONS[action][2]}   "
            f"Опасность: {'ДА' if sit['danger'] else 'нет'}",
            "SPACE — пауза   R — сброс   S — сохранить лог   ESC — выход",
        ]
        cols = [CLR_TEXT, CLR_TEXT, danger_col, (100,120,160)]
    else:
        lines = [f"Шаг: {step}   Счёт: {robot.score}   Жизни: {lives}",
                 "Все монеты собраны! Нажмите R для перезапуска.",
                 "", "SPACE — пауза   R — сброс   S — сохранить лог   ESC — выход"]
        cols = [CLR_TEXT, CLR_COIN, CLR_TEXT, (100,120,160)]

    for i, (line, col) in enumerate(zip(lines, cols)):
        surf.blit(font_md.render(line, True, col), (8, y0 + 4 + i*24))


def draw_predict_panel(surf, font_sm, predictor):
    x0, y0 = WINDOW_W - 240, 8
    rows = predictor.top_rows(6)
    h = 14 * (len(rows)+1) + 16
    pygame.draw.rect(surf, CLR_PANEL, (x0-4, y0-4, 238, h))
    pygame.draw.rect(surf, (40,60,100), (x0-4, y0-4, 238, h), 1)
    surf.blit(font_sm.render("Таблица прогноза ситуаций", True, (120,160,220)), (x0, y0))
    for i, row in enumerate(rows):
        surf.blit(font_sm.render(row, True, (160,200,160)), (x0, y0+16+i*14))


def draw_situation_arrow(surf, robot, sit):
    """Стрелки к монете и от врага."""
    if not sit:
        return
    cx, cy = int(robot.x), int(robot.y)
    # К монете — зелёная
    tx, ty = int(sit["coin_target"][0]), int(sit["coin_target"][1])
    pygame.draw.line(surf, (100, 220, 100), (cx,cy), (tx,ty), 1)
    # От врага — красная пунктирная
    ex, ey = int(sit["enemy_pos"][0]), int(sit["enemy_pos"][1])
    pygame.draw.line(surf, (200, 80, 80), (cx,cy), (ex,ey), 1)


# ─── Game reset ───────────────────────────────────────────────────────────────
def spawn_coins(n=N_COINS) -> List[Coin]:
    coins = []
    for _ in range(n):
        x = random.randint(40, WINDOW_W-40)
        y = random.randint(40, WINDOW_H-150)
        coins.append(Coin(x, y))
    return coins

def spawn_enemies(n=N_ENEMIES) -> List[Enemy]:
    enemies = []
    for _ in range(n):
        # Спавн в углах, подальше от центра
        x = random.choice([random.randint(20,150), random.randint(WINDOW_W-150,WINDOW_W-20)])
        y = random.choice([random.randint(20,150), random.randint(WINDOW_H-250,WINDOW_H-120)])
        enemies.append(Enemy(x, y))
    return enemies

def reset_game():
    robot   = Robot(WINDOW_W//2, WINDOW_H//2 - 50)
    coins   = spawn_coins()
    enemies = spawn_enemies()
    return robot, coins, enemies


# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Игра «Квадрат» — Прогноз ситуаций")
    clock  = pygame.time.Clock()

    font_sm = pygame.font.SysFont("monospace", 11)
    font_md = pygame.font.SysFont("monospace", 13)

    predictor = SituationPredictor()
    journal   = Journal()

    robot, coins, enemies = reset_game()
    lives  = 3
    step   = 0
    paused = False
    action = 0
    last_sit = None
    prev_score = 0

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
                    robot, coins, enemies = reset_game()
                    lives = 3; step = 0
                if event.key == pygame.K_s:
                    journal.save(); print("Лог сохранён: situations.txt")

        if not paused and robot.alive:
            step += 1

            # Получить ситуацию
            sit = get_situation(robot, coins, enemies)
            last_sit = sit

            # Прогноз действия
            if sit:
                action = predictor.predict(sit)
            else:
                action = 4

            # Движение
            robot.move(action)

            # Обновление врагов
            for e in enemies:
                e.update(robot)

            # Сбор монет
            collected = False
            for c in coins:
                if c.collides(robot.x, robot.y):
                    c.collected = True
                    robot.score += 1
                    collected = True

            # Вычисление награды
            reward = 0.0
            if collected:
                reward += 5.0
            if sit:
                if sit["d_enemy_raw"] > 100:
                    reward += 0.1
                elif sit["d_enemy_raw"] < 50:
                    reward -= 1.0

            # Столкновение с врагом
            hit = any(e.collides(robot.x, robot.y) for e in enemies)
            if hit:
                reward -= 3.0
                lives -= 1
                # Запись неудачи
                if sit:
                    predictor.record(sit, action, False)
                    journal.record(step, sit, action, reward)
                if lives <= 0:
                    robot.alive = False
                else:
                    robot.x, robot.y = WINDOW_W//2, WINDOW_H//2 - 50
                    enemies = spawn_enemies()
            else:
                if sit:
                    predictor.record(sit, action, reward > 0)
                    journal.record(step, sit, action, reward)

            # Все монеты собраны — новый раунд
            if all(c.collected for c in coins):
                coins = spawn_coins()
                enemies = spawn_enemies()

        # ── Отрисовка ────────────────────────────────────────────────────────
        screen.fill(CLR_BG)

        # Сетка
        for gx in range(0, WINDOW_W, CELL):
            pygame.draw.line(screen, CLR_GRID, (gx,0),(gx,WINDOW_H-108))
        for gy in range(0, WINDOW_H-108, CELL):
            pygame.draw.line(screen, CLR_GRID, (0,gy),(WINDOW_W,gy))

        # Стрелки ситуации
        draw_situation_arrow(screen, robot, last_sit)

        # Монеты
        for c in coins:
            c.draw(screen, step)

        # Враги
        for e in enemies:
            e.draw(screen)

        # Робот
        robot.draw(screen)

        # Game Over
        if not robot.alive:
            go = font_md.render("GAME OVER — нажмите R", True, CLR_DANGER)
            screen.blit(go, (WINDOW_W//2 - go.get_width()//2, WINDOW_H//2 - 60))

        draw_predict_panel(screen, font_sm, predictor)
        draw_hud(screen, font_sm, font_md, robot, predictor, last_sit, step, action, lives)

        pygame.display.flip()
        clock.tick(FPS)


if __name__ == "__main__":
    main()