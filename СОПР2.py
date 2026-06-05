"""
Адаптивное поведение мобильного робота на базе самообучаемых автоматов Цетлина
Practical assignment: Tsetlin automaton-based adaptive robot navigation
"""

import pygame
import sys
import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ─── Constants ───────────────────────────────────────────────────────────────
WINDOW_W, WINDOW_H = 800, 600
SIM_W, SIM_H = 500, 600          # left: simulation area
FSM_X = SIM_W                    # right: FSM visualisation area
FSM_W = WINDOW_W - SIM_W
FSM_H = SIM_H  

FPS = 30
ROBOT_RADIUS = 12
ROBOT_SPEED = 2.0
ROBOT_FAST  = 4.0

# Colours
CLR_BG_SIM   = (20,  25,  35)
CLR_BG_FSM   = (12,  16,  28)
CLR_GRID     = (30,  38,  55)
CLR_OBSTACLE = (180,  60,  40)
CLR_ROBOT    = ( 80, 200, 120)
CLR_TRAIL    = ( 40, 100,  70)
CLR_TEXT     = (200, 220, 255)
CLR_DIVIDER  = ( 60,  80, 120)

# State activation colour gradient: inactive → active
def activation_color(activation: float) -> Tuple[int,int,int]:
    """Blue (0) → Cyan (0.5) → Green (1.0)"""
    r = int(20  + activation * 20)
    g = int(80  + activation * 160)
    b = int(200 - activation * 80)
    return (min(255,r), min(255,g), min(255,b))

# ─── State names ─────────────────────────────────────────────────────────────
STOP   = 0
FWD    = 1
LEFT   = 2
RIGHT  = 3
BACK   = 4
FFWD   = 5   # fast
FLEFT  = 6
FRIGHT = 7
FBACK  = 8

STATE_NAMES = {
    STOP:"Стоп", FWD:"Вперёд", LEFT:"Влево", RIGHT:"Вправо", BACK:"Назад",
    FFWD:"Быстро вперёд", FLEFT:"Быстро влево",
    FRIGHT:"Быстро вправо", FBACK:"Быстро назад",
}

# Direction vectors (dx, dy)  – Y grows downward
DIRECTIONS = {
    STOP:  ( 0,  0),
    FWD:   ( 0, -1),
    LEFT:  (-1,  0),
    RIGHT: ( 1,  0),
    BACK:  ( 0,  1),
    FFWD:  ( 0, -1),
    FLEFT: (-1,  0),
    FRIGHT:( 1,  0),
    FBACK: ( 0,  1),
}

SPEEDS = {
    STOP:0, FWD:ROBOT_SPEED, LEFT:ROBOT_SPEED,
    RIGHT:ROBOT_SPEED, BACK:ROBOT_SPEED,
    FFWD:ROBOT_FAST, FLEFT:ROBOT_FAST,
    FRIGHT:ROBOT_FAST, FBACK:ROBOT_FAST,
}

# ─── Tsetlin Automaton branch layout ─────────────────────────────────────────
# 4 branches: Forward, Left, Right, Backward
# Each branch: [slow state, fast state] — slow is "penalised step",
#              fast is "rewarded step"
BRANCHES = [
    (FWD,  FFWD),    # branch 0 – forward
    (LEFT, FLEFT),   # branch 1 – left
    (RIGHT,FRIGHT),  # branch 2 – right
    (BACK, FBACK),   # branch 3 – back
]

# ─── Class: State ─────────────────────────────────────────────────────────────
class State:
    def __init__(self, sid: int, x: float, y: float):
        self.sid = sid
        self.name = STATE_NAMES[sid]
        self.x = x                  # FSM canvas position
        self.y = y
        self.activation = 0.0       # 0..1
        self.is_active = False
        self.out_arcs: List['State'] = []   # outgoing arcs
        self.in_arcs:  List['State'] = []   # incoming arcs

    def add_arc(self, target: 'State'):
        if target not in self.out_arcs:
            self.out_arcs.append(target)
        if self not in target.in_arcs:
            target.in_arcs.append(self)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, ox: int):
        cx = int(self.x) + ox
        cy = int(self.y)
        r  = 20

        # Fill based on activation
        col = activation_color(self.activation)
        pygame.draw.circle(surface, col, (cx, cy), r)

        # Active ring
        if self.is_active:
            pygame.draw.circle(surface, (255, 255, 100), (cx, cy), r+3, 2)
        else:
            pygame.draw.circle(surface, CLR_DIVIDER, (cx, cy), r, 1)

        # Label
        short = self.name[:8]
        lbl = font.render(short, True, (230, 240, 255))
        surface.blit(lbl, (cx - lbl.get_width()//2, cy + r + 2))

# ─── Class: FSM ───────────────────────────────────────────────────────────────
class FSM:
    """Tsetlin automaton finite-state machine with 4 branches."""

    def __init__(self):
        self.states: dict[int, State] = {}
        self.current: State = None
        self._build()

    # --- layout helpers -------------------------------------------------------
    def _build(self):
        # Stop node at top centre of FSM area
        W, H = FSM_W, FSM_H
        cx = W // 2
        self._add(STOP, cx, 40)

        # 4 branches arranged vertically in two columns
        branch_x = [cx - 70, cx - 70, cx + 70, cx + 70]
        # reorder: left column = FWD,BACK; right = LEFT,RIGHT
        positions = {
            FWD:   (cx - 75, 130),
            FFWD:  (cx - 75, 220),
            LEFT:  (cx - 75, 330),
            FLEFT: (cx - 75, 420),
            RIGHT: (cx + 75, 130),
            FRIGHT:(cx + 75, 220),
            BACK:  (cx + 75, 330),
            FBACK: (cx + 75, 420),
        }
        for sid, (x, y) in positions.items():
            self._add(sid, x, y)

        # Tsetlin transitions:
        # STOP → all slow states (random choice at start)
        for slow, fast in BRANCHES:
            self.states[STOP].add_arc(self.states[slow])

        # Within each branch: reward→advance, penalty→retreat
        # slow ←reward→ fast  (reward: slow→fast, penalty: stay slow)
        # fast ←penalty→ slow (reward: stay fast, penalty: fast→slow)
        for slow, fast in BRANCHES:
            self.states[slow].add_arc(self.states[fast])   # reward
            self.states[slow].add_arc(self.states[STOP])   # heavy penalty
            self.states[fast].add_arc(self.states[slow])   # penalty
            self.states[fast].add_arc(self.states[STOP])   # very heavy penalty

        # Set initial state
        self.current = self.states[STOP]
        self.current.is_active = True

    def _add(self, sid: int, x: float, y: float):
        self.states[sid] = State(sid, x, y)

    # --- Tsetlin update -------------------------------------------------------
    def update(self, reward: bool):
        """
        Tsetlin rule:
          reward → move toward fast state in branch (or stay if already fast)
          penalty → move toward STOP (one step back)
        """
        cur = self.current
        cur.is_active = False

        if cur.sid == STOP:
            # Choose a random branch to explore
            slow = random.choice([s for s,_ in BRANCHES])
            nxt = self.states[slow]
        else:
            # Find which branch
            branch = None
            for s, f in BRANCHES:
                if cur.sid in (s, f):
                    branch = (s, f)
                    break

            if branch is None:
                nxt = self.states[STOP]
            else:
                slow, fast = branch
                if reward:
                    # advance in branch: slow→fast, fast→fast (stay)
                    nxt = self.states[fast]
                else:
                    # retreat: fast→slow, slow→STOP
                    if cur.sid == fast:
                        nxt = self.states[slow]
                    else:
                        nxt = self.states[STOP]

        self.current = nxt
        nxt.is_active = True

        # Decay all activations, boost current
        for s in self.states.values():
            s.activation *= 0.97
        nxt.activation = min(1.0, nxt.activation + 0.35)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, ox: int):
        # Draw arcs first
        drawn = set()
        for s in self.states.values():
            for t in s.out_arcs:
                key = (min(s.sid,t.sid), max(s.sid,t.sid))
                if key in drawn:
                    continue
                drawn.add(key)
                sx, sy = int(s.x)+ox, int(s.y)
                tx, ty = int(t.x)+ox, int(t.y)
                pygame.draw.line(surface, CLR_DIVIDER, (sx,sy), (tx,ty), 1)
                # Arrow head
                ang = math.atan2(ty-sy, tx-sx)
                mx, my = (sx+tx)//2, (sy+ty)//2
                al = 7
                aa = 0.45
                p1 = (mx + al*math.cos(ang+math.pi-aa),
                      my + al*math.sin(ang+math.pi-aa))
                p2 = (mx + al*math.cos(ang+math.pi+aa),
                      my + al*math.sin(ang+math.pi+aa))
                pygame.draw.polygon(surface, CLR_DIVIDER, [(mx,my),p1,p2])

        # Draw states
        for s in self.states.values():
            s.draw(surface, font, ox)

        # Branch labels
        bfont = pygame.font.SysFont("monospace", 10)
        labels = [("Вперёд", int(FSM_W//2 - 75), 90),
                  ("Влево",  int(FSM_W//2 - 75), 300),
                  ("Вправо",int(FSM_W//2 + 75), 90),
                  ("Назад",  int(FSM_W//2 + 75), 300)]
        for txt, bx, by in labels:
            lbl = bfont.render(txt, True, (100,120,160))
            surface.blit(lbl, (bx - lbl.get_width()//2 + ox, by))

# ─── Class: Obstacle ──────────────────────────────────────────────────────────
class Obstacle:
    def __init__(self, x: int, y: int, w: int, h: int):
        self.rect = pygame.Rect(x, y, w, h)

    def draw(self, surface: pygame.Surface):
        pygame.draw.rect(surface, CLR_OBSTACLE, self.rect, border_radius=4)
        pygame.draw.rect(surface, (220, 90, 60), self.rect, 2, border_radius=4)

    def collides_circle(self, cx: float, cy: float, r: float) -> bool:
        # clamp centre to rect, measure distance
        nx = max(self.rect.left, min(cx, self.rect.right))
        ny = max(self.rect.top,  min(cy, self.rect.bottom))
        return (cx-nx)**2 + (cy-ny)**2 < r*r

# ─── Class: Robot ─────────────────────────────────────────────────────────────
class Robot:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.trail: List[Tuple[float,float]] = []
        self.colliding = False
        self.angle = 0.0   # visual heading in degrees

    def move(self, state_id: int, obstacles: List[Obstacle]):
        dx, dy = DIRECTIONS[state_id]
        sp = SPEEDS[state_id]
        nx = self.x + dx * sp
        ny = self.y + dy * sp

        # Boundary
        nx = max(ROBOT_RADIUS, min(SIM_W - ROBOT_RADIUS, nx))
        ny = max(ROBOT_RADIUS, min(SIM_H - ROBOT_RADIUS, ny))

        # Obstacle check
        hit = any(o.collides_circle(nx, ny, ROBOT_RADIUS + 2) for o in obstacles)
        if hit:
            self.colliding = True
        else:
            self.colliding = False
            self.x, self.y = nx, ny
            if dx != 0 or dy != 0:
                self.angle = math.degrees(math.atan2(dy, dx)) + 90

        # Trail
        self.trail.append((self.x, self.y))
        if len(self.trail) > 1200:
            self.trail.pop(0)

    def draw(self, surface: pygame.Surface):
        # Trail
        if len(self.trail) > 1:
            for i in range(1, len(self.trail)):
                alpha = i / len(self.trail)
                c = (int(CLR_TRAIL[0]*alpha), int(CLR_TRAIL[1]*alpha), int(CLR_TRAIL[2]*alpha))
                pygame.draw.line(surface, c,
                                 (int(self.trail[i-1][0]), int(self.trail[i-1][1])),
                                 (int(self.trail[i][0]),   int(self.trail[i][1])), 1)

        # Body
        col = (220, 60, 60) if self.colliding else CLR_ROBOT
        pygame.draw.circle(surface, col, (int(self.x), int(self.y)), ROBOT_RADIUS)
        pygame.draw.circle(surface, (255,255,255), (int(self.x), int(self.y)), ROBOT_RADIUS, 1)

        # Direction indicator
        rad = math.radians(self.angle)
        ex = self.x + (ROBOT_RADIUS - 2) * math.sin(rad)
        ey = self.y - (ROBOT_RADIUS - 2) * math.cos(rad)
        pygame.draw.line(surface, (255,255,200),
                         (int(self.x), int(self.y)), (int(ex), int(ey)), 2)

# ─── Reinforcement signal ────────────────────────────────────────────────────
def compute_reward(robot: Robot, state_id: int) -> Optional[bool]:
    """
    Returns True (reward) if robot moves fast without collision,
    False (penalty) if collision or stopped,
    None if slow movement (neutral – small penalty).
    """
    if robot.colliding:
        return False
    if state_id in (FFWD, FLEFT, FRIGHT, FBACK):
        return True
    if state_id == STOP:
        return False
    return None   # slow movement → slight penalty

# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Робот Цетлина — Адаптивное управление")
    clock = pygame.time.Clock()

    font_sm = pygame.font.SysFont("monospace", 11)
    font_md = pygame.font.SysFont("monospace", 13)
    font_lg = pygame.font.SysFont("monospace", 15, bold=True)

    # ── Obstacles (input) ──────────────────────────────────────────────────
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

    # ── Robot (input) ──────────────────────────────────────────────────────
    robot = Robot(40.0, 540.0)

    # ── FSM ───────────────────────────────────────────────────────────────
    fsm = FSM()

    step = 0
    update_interval = 12   # frames between automaton updates
    paused = False

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_SPACE:
                    paused = not paused
                if event.key == pygame.K_r:
                    robot = Robot(40.0, 540.0)
                    fsm = FSM()
                    step = 0

        if not paused:
            step += 1

            # Move robot according to current FSM state
            cur_sid = fsm.current.sid
            robot.move(cur_sid, obstacles)

            # Every N frames: compute reward and update automaton
            if step % update_interval == 0:
                rew = compute_reward(robot, cur_sid)
                if rew is None:
                    rew = False   # slow movement treated as penalty
                fsm.update(rew)

        # ── Draw simulation area ──────────────────────────────────────────
        screen.fill(CLR_BG_SIM, (0, 0, SIM_W, SIM_H))

        # Grid
        for gx in range(0, SIM_W, 40):
            pygame.draw.line(screen, CLR_GRID, (gx, 0), (gx, SIM_H))
        for gy in range(0, SIM_H, 40):
            pygame.draw.line(screen, CLR_GRID, (0, gy), (SIM_W, gy))

        for obs in obstacles:
            obs.draw(screen)
        robot.draw(screen)

        # HUD
        state_lbl = font_md.render(f"Состояние: {STATE_NAMES[fsm.current.sid]}", True, CLR_TEXT)
        screen.blit(state_lbl, (8, 8))
        step_lbl = font_sm.render(f"Шаг: {step}  [SPACE]-пауза  [R]-сброс", True, (120,140,180))
        screen.blit(step_lbl, (8, 26))

        # ── Divider ───────────────────────────────────────────────────────
        pygame.draw.line(screen, CLR_DIVIDER, (SIM_W, 0), (SIM_W, SIM_H), 2)

        # ── Draw FSM area ─────────────────────────────────────────────────
        screen.fill(CLR_BG_FSM, (SIM_W, 0, FSM_W, FSM_H))
        fsm_title = font_lg.render("Автомат Цетлина", True, (160, 190, 240))
        screen.blit(fsm_title, (SIM_W + (FSM_W - fsm_title.get_width())//2, 8))
        fsm.draw(screen, font_sm, SIM_W)

        # Activation legend
        for i, (sid, name) in enumerate(STATE_NAMES.items()):
            s = fsm.states[sid]
            c = activation_color(s.activation)
            pygame.draw.rect(screen, c, (SIM_W + 4, FSM_H - 145 + i*15, 10, 10))
            lbl = font_sm.render(name, True, CLR_TEXT if s.is_active else (100,120,150))
            screen.blit(lbl, (SIM_W + 18, FSM_H - 146 + i*15))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()