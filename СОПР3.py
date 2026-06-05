"""
Адаптивное поведение мобильного робота на базе самообучаемых автоматов Кринского
Отличие от Цетлина: автомат Кринского при получении штрафа возвращается
сразу в начало своей ветки (минимальный номер состояния), а не на один шаг.
При награде продвигается на один шаг вперёд (как у Цетлина).
"""

import pygame
import sys
import math
import random
from typing import List, Optional, Tuple

# ─── Constants ────────────────────────────────────────────────────────────────
WINDOW_W, WINDOW_H = 800, 600
SIM_W,    SIM_H    = 500, 600
FSM_X = SIM_W
FSM_W = WINDOW_W - SIM_W
FSM_H = SIM_H

FPS            = 30
ROBOT_RADIUS   = 12
ROBOT_SPEED    = 2.0
ROBOT_FAST     = 4.0
UPDATE_EVERY   = 12          # frames between automaton updates

# Colours
CLR_BG_SIM   = (20,  25,  35)
CLR_BG_FSM   = (12,  16,  28)
CLR_GRID     = (30,  38,  55)
CLR_OBSTACLE = (180,  60,  40)
CLR_ROBOT    = ( 80, 200, 120)
CLR_TRAIL    = ( 40, 100,  70)
CLR_TEXT     = (200, 220, 255)
CLR_DIVIDER  = ( 60,  80, 120)


def activation_color(a: float) -> Tuple[int, int, int]:
    """0 → blue, 0.5 → cyan, 1 → green"""
    return (min(255, int(20 + a * 20)),
            min(255, int(80 + a * 160)),
            min(255, int(200 - a * 80)))


# ─── State IDs ───────────────────────────────────────────────────────────────
STOP   = 0
FWD    = 1
LEFT   = 2
RIGHT  = 3
BACK   = 4
FFWD   = 5
FLEFT  = 6
FRIGHT = 7
FBACK  = 8

STATE_NAMES = {
    STOP:  "Стоп",
    FWD:   "Вперёд",    LEFT:  "Влево",
    RIGHT: "Вправо",    BACK:  "Назад",
    FFWD:  "Быстро вперёд",  FLEFT:  "Быстро влево",
    FRIGHT:"Быстро вправо",  FBACK:  "Быстро назад",
}

DIRECTIONS = {
    STOP:  (0,  0),
    FWD:   (0, -1),   LEFT:  (-1, 0),
    RIGHT: (1,  0),   BACK:  (0,  1),
    FFWD:  (0, -1),   FLEFT: (-1, 0),
    FRIGHT:(1,  0),   FBACK: (0,  1),
}

SPEEDS = {
    STOP: 0,
    FWD:  ROBOT_SPEED, LEFT:  ROBOT_SPEED,
    RIGHT:ROBOT_SPEED, BACK:  ROBOT_SPEED,
    FFWD: ROBOT_FAST,  FLEFT: ROBOT_FAST,
    FRIGHT:ROBOT_FAST, FBACK: ROBOT_FAST,
}

# Branches: (slow_state, fast_state)
BRANCHES = [
    (FWD,  FFWD),
    (LEFT, FLEFT),
    (RIGHT,FRIGHT),
    (BACK, FBACK),
]


# ─── class State ─────────────────────────────────────────────────────────────
class State:
    def __init__(self, sid: int, x: float, y: float):
        self.sid        = sid
        self.name       = STATE_NAMES[sid]
        self.x          = x
        self.y          = y
        self.activation = 0.0
        self.is_active  = False
        self.out_arcs:  List['State'] = []
        self.in_arcs:   List['State'] = []

    def add_arc(self, target: 'State'):
        if target not in self.out_arcs:
            self.out_arcs.append(target)
        if self not in target.in_arcs:
            target.in_arcs.append(self)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, ox: int):
        cx, cy, r = int(self.x) + ox, int(self.y), 20
        col = activation_color(self.activation)
        pygame.draw.circle(surface, col, (cx, cy), r)
        if self.is_active:
            pygame.draw.circle(surface, (255, 255, 100), (cx, cy), r + 3, 2)
        else:
            pygame.draw.circle(surface, CLR_DIVIDER, (cx, cy), r, 1)
        lbl = font.render(self.name[:8], True, (230, 240, 255))
        surface.blit(lbl, (cx - lbl.get_width() // 2, cy + r + 2))


# ─── class FSM (Krinsky) ─────────────────────────────────────────────────────
class FSM:
    """
    Автомат Кринского:
      - При НАГРАДЕ:  переход на один шаг вперёд внутри ветки (slow→fast, fast→fast).
      - При ШТРАФЕ:   немедленный возврат в начало ветки (т.е. в slow-состояние),
                      а из slow — сразу в STOP.
    Это отличает Кринского от Цетлина: штраф — прыжок к началу,
    а не откат на один шаг.
    """

    def __init__(self):
        self.states:  dict[int, State] = {}
        self.current: State = None
        self._build()

    def _add(self, sid: int, x: float, y: float):
        self.states[sid] = State(sid, x, y)

    def _build(self):
        cx = FSM_W // 2
        self._add(STOP, cx, 40)

        positions = {
            FWD:   (cx - 75, 130),  FFWD:  (cx - 75, 220),
            LEFT:  (cx - 75, 330),  FLEFT: (cx - 75, 420),
            RIGHT: (cx + 75, 130),  FRIGHT:(cx + 75, 220),
            BACK:  (cx + 75, 330),  FBACK: (cx + 75, 420),
        }
        for sid, (x, y) in positions.items():
            self._add(sid, x, y)

        # STOP → все медленные состояния
        for slow, _ in BRANCHES:
            self.states[STOP].add_arc(self.states[slow])

        # Внутри каждой ветки
        for slow, fast in BRANCHES:
            self.states[slow].add_arc(self.states[fast])    # награда
            self.states[slow].add_arc(self.states[STOP])    # штраф из slow
            self.states[fast].add_arc(self.states[slow])    # штраф из fast → начало ветки
            self.states[fast].add_arc(self.states[STOP])    # тяжёлый штраф

        self.current = self.states[STOP]
        self.current.is_active = True

    # ── Krinsky update rule ──────────────────────────────────────────────────
    def update(self, reward: bool):
        cur = self.current
        cur.is_active = False

        if cur.sid == STOP:
            # Выбираем случайную ветку, входим в slow-состояние
            slow, _ = random.choice(BRANCHES)
            nxt = self.states[slow]
        else:
            branch = next(((s, f) for s, f in BRANCHES if cur.sid in (s, f)), None)
            if branch is None:
                nxt = self.states[STOP]
            else:
                slow, fast = branch
                if reward:
                    # Кринский: награда — шаг вперёд (slow→fast, fast→fast)
                    nxt = self.states[fast]
                else:
                    # Кринский: штраф — ПРЫЖОК В НАЧАЛО ВЕТКИ (или в STOP из slow)
                    # Это ключевое отличие от Цетлина (который делает шаг назад)
                    if cur.sid == fast:
                        nxt = self.states[slow]   # fast → slow (начало ветки)
                    else:
                        nxt = self.states[STOP]   # slow → STOP

        self.current = nxt
        nxt.is_active = True

        for s in self.states.values():
            s.activation *= 0.97
        nxt.activation = min(1.0, nxt.activation + 0.35)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, ox: int):
        # Дуги
        drawn = set()
        for s in self.states.values():
            for t in s.out_arcs:
                key = (min(s.sid, t.sid), max(s.sid, t.sid))
                if key in drawn:
                    continue
                drawn.add(key)
                sx, sy = int(s.x) + ox, int(s.y)
                tx, ty = int(t.x) + ox, int(t.y)
                pygame.draw.line(surface, CLR_DIVIDER, (sx, sy), (tx, ty), 1)
                ang = math.atan2(ty - sy, tx - sx)
                mx, my = (sx + tx) // 2, (sy + ty) // 2
                al, aa = 7, 0.45
                p1 = (mx + al * math.cos(ang + math.pi - aa),
                      my + al * math.sin(ang + math.pi - aa))
                p2 = (mx + al * math.cos(ang + math.pi + aa),
                      my + al * math.sin(ang + math.pi + aa))
                pygame.draw.polygon(surface, CLR_DIVIDER, [(mx, my), p1, p2])

        # Узлы
        for s in self.states.values():
            s.draw(surface, font, ox)

        # Подписи веток
        bfont = pygame.font.SysFont("monospace", 10)
        for txt, bx, by in [
            ("Вперёд", FSM_W // 2 - 75, 90),
            ("Влево",  FSM_W // 2 - 75, 300),
            ("Вправо", FSM_W // 2 + 75, 90),
            ("Назад",  FSM_W // 2 + 75, 300),
        ]:
            lbl = bfont.render(txt, True, (100, 120, 160))
            surface.blit(lbl, (bx - lbl.get_width() // 2 + ox, by))


# ─── class Obstacle ──────────────────────────────────────────────────────────
class Obstacle:
    def __init__(self, x: int, y: int, w: int, h: int):
        self.rect = pygame.Rect(x, y, w, h)

    def draw(self, surface: pygame.Surface):
        pygame.draw.rect(surface, CLR_OBSTACLE, self.rect, border_radius=4)
        pygame.draw.rect(surface, (220, 90, 60), self.rect, 2, border_radius=4)

    def collides_circle(self, cx: float, cy: float, r: float) -> bool:
        nx = max(self.rect.left,   min(cx, self.rect.right))
        ny = max(self.rect.top,    min(cy, self.rect.bottom))
        return (cx - nx) ** 2 + (cy - ny) ** 2 < r * r


# ─── class Robot ─────────────────────────────────────────────────────────────
class Robot:
    def __init__(self, x: float, y: float):
        self.x         = x
        self.y         = y
        self.trail:    List[Tuple[float, float]] = []
        self.colliding = False
        self.angle     = 0.0

    def move(self, state_id: int, obstacles: List[Obstacle]):
        dx, dy = DIRECTIONS[state_id]
        sp     = SPEEDS[state_id]
        nx     = max(ROBOT_RADIUS, min(SIM_W - ROBOT_RADIUS, self.x + dx * sp))
        ny     = max(ROBOT_RADIUS, min(SIM_H - ROBOT_RADIUS, self.y + dy * sp))

        if any(o.collides_circle(nx, ny, ROBOT_RADIUS + 2) for o in obstacles):
            self.colliding = True
        else:
            self.colliding = False
            self.x, self.y = nx, ny
            if dx != 0 or dy != 0:
                self.angle = math.degrees(math.atan2(dy, dx)) + 90

        self.trail.append((self.x, self.y))
        if len(self.trail) > 1200:
            self.trail.pop(0)

    def draw(self, surface: pygame.Surface):
        if len(self.trail) > 1:
            for i in range(1, len(self.trail)):
                a = i / len(self.trail)
                c = (int(CLR_TRAIL[0] * a), int(CLR_TRAIL[1] * a), int(CLR_TRAIL[2] * a))
                pygame.draw.line(surface, c,
                                 (int(self.trail[i - 1][0]), int(self.trail[i - 1][1])),
                                 (int(self.trail[i][0]),     int(self.trail[i][1])), 1)
        col = (220, 60, 60) if self.colliding else CLR_ROBOT
        pygame.draw.circle(surface, col,   (int(self.x), int(self.y)), ROBOT_RADIUS)
        pygame.draw.circle(surface, (255, 255, 255), (int(self.x), int(self.y)), ROBOT_RADIUS, 1)
        rad = math.radians(self.angle)
        ex  = self.x + (ROBOT_RADIUS - 2) * math.sin(rad)
        ey  = self.y - (ROBOT_RADIUS - 2) * math.cos(rad)
        pygame.draw.line(surface, (255, 255, 200),
                         (int(self.x), int(self.y)), (int(ex), int(ey)), 2)


# ─── Reinforcement signal ────────────────────────────────────────────────────
def compute_reward(robot: Robot, state_id: int) -> Optional[bool]:
    if robot.colliding:
        return False
    if state_id in (FFWD, FLEFT, FRIGHT, FBACK):
        return True
    if state_id == STOP:
        return False
    return None   # медленное движение → нейтральный (штраф)


# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Робот Кринского — Адаптивное управление")
    clock  = pygame.time.Clock()

    font_sm = pygame.font.SysFont("monospace", 11)
    font_md = pygame.font.SysFont("monospace", 13)

    # ── Входные данные: препятствия ───────────────────────────────────────
    obstacles = [
        Obstacle( 80,  80, 100,  30),
        Obstacle(280,  50,  30, 120),
        Obstacle( 50, 220,  80,  30),
        Obstacle(200, 200, 120,  25),
        Obstacle(360, 150,  25, 100),
        Obstacle( 80, 380,  60, 100),
        Obstacle(220, 350, 140,  30),
        Obstacle(380, 350,  30, 120),
        Obstacle(150, 480, 200,  30),
        Obstacle( 60, 510, 100,  25),
        Obstacle(360, 480, 100,  40),
    ]

    # ── Входные данные: начальное положение робота ─────────────────────────
    robot  = Robot(40.0, 540.0)
    fsm    = FSM()
    step   = 0
    paused = False

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
                    robot = Robot(40.0, 540.0)
                    fsm   = FSM()
                    step  = 0

        if not paused:
            step += 1
            cur_sid = fsm.current.sid
            robot.move(cur_sid, obstacles)
            if step % UPDATE_EVERY == 0:
                rew = compute_reward(robot, cur_sid)
                if rew is None:
                    rew = False
                fsm.update(rew)

        # ── Симуляционная панель ──────────────────────────────────────────
        screen.fill(CLR_BG_SIM, (0, 0, SIM_W, SIM_H))
        for gx in range(0, SIM_W, 40):
            pygame.draw.line(screen, CLR_GRID, (gx, 0), (gx, SIM_H))
        for gy in range(0, SIM_H, 40):
            pygame.draw.line(screen, CLR_GRID, (0, gy), (SIM_W, gy))
        for obs in obstacles:
            obs.draw(screen)
        robot.draw(screen)

        screen.blit(font_md.render(f"Состояние: {STATE_NAMES[fsm.current.sid]}", True, CLR_TEXT), (8, 8))
        screen.blit(font_sm.render(f"Шаг: {step}  [SPACE]-пауза  [R]-сброс", True, (120, 140, 180)), (8, 26))

        # ── Разделитель ───────────────────────────────────────────────────
        pygame.draw.line(screen, CLR_DIVIDER, (SIM_W, 0), (SIM_W, SIM_H), 2)

        # ── FSM панель ────────────────────────────────────────────────────
        screen.fill(CLR_BG_FSM, (SIM_W, 0, FSM_W, FSM_H))
        title = font_md.render("Автомат Кринского", True, (160, 190, 240))
        screen.blit(title, (SIM_W + (FSM_W - title.get_width()) // 2, 8))
        fsm.draw(screen, font_sm, SIM_W)

        # Легенда активации
        for i, (sid, name) in enumerate(STATE_NAMES.items()):
            s = fsm.states[sid]
            pygame.draw.rect(screen, activation_color(s.activation),
                             (SIM_W + 4, FSM_H - 145 + i * 15, 10, 10))
            screen.blit(font_sm.render(name, True, CLR_TEXT if s.is_active else (100, 120, 150)),
                        (SIM_W + 18, FSM_H - 146 + i * 15))

        pygame.display.flip()
        clock.tick(FPS)


if __name__ == "__main__":
    main()