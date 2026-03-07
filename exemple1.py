import pygame
import random
import math
import heapq
import numpy as np

# CONFIGURATION
WIDTH = 800
HEIGHT = 600

CELL = 20

ROBOT_RADIUS = 12
OBJ_RADIUS = 8

DROP_POINT = np.array([750, 550])

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Autonomous Transport Robot - Maze")
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 18)

# MAZE GENERATION

def generate_maze():

    grid_w = WIDTH // CELL
    grid_h = HEIGHT // CELL

    grid = [[0 for _ in range(grid_h)] for _ in range(grid_w)]

    for x in range(grid_w):
        for y in range(grid_h):
            if random.random() < 0.25:
                grid[x][y] = 1

    # corridor garanti
    for x in range(1, grid_w-1):
        grid[x][1] = 0

    for y in range(1, grid_h-1):
        grid[grid_w-2][y] = 0

    return grid


# A* PATH PLANNING
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])


def astar(grid, start, goal):

    w = len(grid)
    h = len(grid[0])

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g = {start:0}

    while open_set:

        current = heapq.heappop(open_set)[1]

        if current == goal:

            path = []

            while current in came_from:
                path.append(current)
                current = came_from[current]

            path.reverse()
            return path

        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:

            nx = current[0] + dx
            ny = current[1] + dy

            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue

            if grid[nx][ny] == 1:
                continue

            neighbor = (nx,ny)

            new_g = g[current] + 1

            if neighbor not in g or new_g < g[neighbor]:

                g[neighbor] = new_g
                f = new_g + heuristic(neighbor, goal)

                heapq.heappush(open_set, (f, neighbor))
                came_from[neighbor] = current

    return []

# OBJECT CLASS
class Obj:

    def __init__(self, x, y):
        self.pos = np.array([x,y], dtype=float)
        self.collected = False

    def draw(self, screen):

        if not self.collected:
            pygame.draw.circle(screen,(0,200,0),
                               self.pos.astype(int),OBJ_RADIUS)

# ROBOT CLASS
class Robot:

    def __init__(self,x,y):

        self.pos = np.array([x,y],dtype=float)

        self.speed = 2

        self.path = []
        self.path_pixels = []

        self.attached = None

        self.angle = 0

    def set_path(self,path):

        self.path = path
        self.path_pixels = []

        for cell in path:

            px = cell[0]*CELL + CELL//2
            py = cell[1]*CELL + CELL//2

            self.path_pixels.append(np.array([px,py]))

    def update(self):

        if not self.path_pixels:
            return

        target = self.path_pixels[0]

        direction = target - self.pos
        dist = np.linalg.norm(direction)

        if dist < 3:
            self.path_pixels.pop(0)
            return

        direction = direction/dist

        self.pos += direction*self.speed

        self.angle = math.atan2(direction[1],direction[0])

        if self.attached:
            self.attached.pos = self.pos.copy()

    def draw(self,screen):

        # corps
        pygame.draw.circle(screen,(0,0,255),
                           self.pos.astype(int),
                           ROBOT_RADIUS)

        # triangle direction
        size = ROBOT_RADIUS

        p1 = self.pos + np.array([
            math.cos(self.angle)*size,
            math.sin(self.angle)*size
        ])

        p2 = self.pos + np.array([
            math.cos(self.angle+2.5)*size*0.7,
            math.sin(self.angle+2.5)*size*0.7
        ])

        p3 = self.pos + np.array([
            math.cos(self.angle-2.5)*size*0.7,
            math.sin(self.angle-2.5)*size*0.7
        ])

        pygame.draw.polygon(screen,(255,255,0),
                            [p1.astype(int),
                             p2.astype(int),
                             p3.astype(int)])

        # roues
        offset = ROBOT_RADIUS

        left = self.pos + np.array([
            math.cos(self.angle+math.pi/2)*offset,
            math.sin(self.angle+math.pi/2)*offset
        ])

        right = self.pos + np.array([
            math.cos(self.angle-math.pi/2)*offset,
            math.sin(self.angle-math.pi/2)*offset
        ])

        pygame.draw.rect(screen,(30,30,30),
                         (left[0]-4,left[1]-8,8,16))

        pygame.draw.rect(screen,(30,30,30),
                         (right[0]-4,right[1]-8,8,16))

        # chemin A*
        for p in self.path_pixels:

            pygame.draw.circle(screen,(255,255,0),
                               p.astype(int),3)


# FSM
class FSM:

    def __init__(self):
        self.state = "Ready"

# CONTROL SYSTEM
class ControlSystem:

    def __init__(self,robot,objects,grid,fsm):

        self.robot = robot
        self.objects = objects
        self.grid = grid
        self.fsm = fsm

        self.target = None
        self.delivered = 0

    def cell(self,pos):

        return (int(pos[0]//CELL),
                int(pos[1]//CELL))

    def update(self):

        if self.fsm.state == "Ready":

            remaining = [o for o in self.objects if not o.collected]

            if not remaining:
                return

            remaining.sort(key=lambda o:
                           np.linalg.norm(o.pos-self.robot.pos))

            self.target = remaining[0]

            start = self.cell(self.robot.pos)
            goal = self.cell(self.target.pos)

            path = astar(self.grid,start,goal)

            if path:

                self.robot.set_path(path)
                self.fsm.state = "Approaching"

        elif self.fsm.state == "Approaching":

            self.robot.update()

            if np.linalg.norm(self.robot.pos-self.target.pos) < 15:

                self.robot.attached = self.target
                self.target.collected = True

                start = self.cell(self.robot.pos)
                goal = self.cell(DROP_POINT)

                path = astar(self.grid,start,goal)

                self.robot.set_path(path)

                self.fsm.state = "Moving"

        elif self.fsm.state == "Moving":

            self.robot.update()

            if np.linalg.norm(self.robot.pos-DROP_POINT) < 15:

                self.robot.attached = None
                self.delivered += 1

                self.fsm.state = "Finished"

        elif self.fsm.state == "Finished":

            remaining = any(not o.collected for o in self.objects)

            if remaining:
                self.fsm.state = "Ready"

# OBJECT GENERATION
def generate_objects(n,grid):

    objs = []

    w = len(grid)
    h = len(grid[0])

    while len(objs) < n:

        x = random.randint(0,w-1)
        y = random.randint(0,h-1)

        if grid[x][y] == 0:

            px = x*CELL + CELL//2
            py = y*CELL + CELL//2

            objs.append(Obj(px,py))

    return objs

# DRAW MAZE
def draw_maze(grid):

    for x in range(len(grid)):
        for y in range(len(grid[0])):

            if grid[x][y] == 1:

                pygame.draw.rect(screen,(80,80,80),
                                 (x*CELL,y*CELL,CELL,CELL))

# INITIALIZATION
grid = generate_maze()

robot = Robot(40,40)

objects = generate_objects(8,grid)

fsm = FSM()

control = ControlSystem(robot,objects,grid,fsm)

# MAIN LOOP
running = True

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    control.update()

    screen.fill((30,30,30))

    draw_maze(grid)

    pygame.draw.circle(screen,(255,0,0),
                       DROP_POINT.astype(int),12)

    for obj in objects:
        obj.draw(screen)

    robot.draw(screen)

    state_text = font.render("State: "+fsm.state,
                             True,(255,255,255))

    screen.blit(state_text,(10,10))

    count_text = font.render(
        f"Delivered: {control.delivered}/{len(objects)}",
        True,(255,255,255))

    screen.blit(count_text,(10,30))

    pygame.display.flip()

    clock.tick(60)

pygame.quit()