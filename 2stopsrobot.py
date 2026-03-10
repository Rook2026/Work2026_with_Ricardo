import pygame
import random
import math
import time

# ================== CONFIG ==================
WIDTH, HEIGHT = 800, 600

ROBOT_RADIUS = 20
OBSTACLE_RADIUS = 30
NUM_OBSTACLES = 10

GOAL1_RADIUS = 30
GOAL2_RADIUS = 30

WAIT_TIME = 3  # secondes

MAX_LIDAR_DIST = 120
NUM_RAYS = 24

MAX_SPEED = 2.0
SAFE_DISTANCE = 60

# ================== INIT ==================
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Advanced Mobile Robot Simulation")
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 16)

# ================== UTILS ==================
def distance(x1,y1,x2,y2):
    return math.hypot(x2-x1,y2-y1)

# ================== CLASSES ==================

class Obstacle:
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.r=OBSTACLE_RADIUS

    def draw(self,screen):
        pygame.draw.circle(screen,(200,60,60),(int(self.x),int(self.y)),self.r)

class Lidar:

    def __init__(self,robot):
        self.robot=robot
        self.rays=[]
        self.values=[]

    def scan(self,obstacles):

        self.rays=[]
        self.values=[]

        for i in range(NUM_RAYS):

            angle = 2*math.pi*i/NUM_RAYS

            dist = MAX_LIDAR_DIST

            for d in range(0,MAX_LIDAR_DIST,4):

                x = self.robot.x + d*math.cos(angle)
                y = self.robot.y + d*math.sin(angle)

                for obs in obstacles:
                    if distance(x,y,obs.x,obs.y) <= obs.r:
                        dist = d
                        break

                if dist != MAX_LIDAR_DIST:
                    break

            self.rays.append((angle,dist))
            self.values.append(dist)

    def draw(self,screen):

        for angle,dist in self.rays:

            x2 = self.robot.x + dist*math.cos(angle)
            y2 = self.robot.y + dist*math.sin(angle)

            pygame.draw.line(screen,(80,80,80),
                             (self.robot.x,self.robot.y),
                             (x2,y2),1)

class Robot:

    def __init__(self,x,y,alpha):

        self.x=x
        self.y=y
        self.alpha=alpha
        self.r=ROBOT_RADIUS

        self.path=[]

        self.lidar=Lidar(self)

    def draw(self,screen):

        pygame.draw.circle(screen,(80,160,255),
                           (int(self.x),int(self.y)),self.r)

        # direction line
        x2 = self.x + self.r*math.cos(self.alpha)
        y2 = self.y + self.r*math.sin(self.alpha)

        pygame.draw.line(screen,(255,255,255),
                         (self.x,self.y),(x2,y2),3)

        # trajectory
        if len(self.path)>2:
            pygame.draw.lines(screen,(255,255,0),False,self.path,2)

    def move(self,vx,vy,obstacles):

        newx = self.x + vx
        newy = self.y + vy

        collision=False

        for obs in obstacles:
            if distance(newx,newy,obs.x,obs.y) <= obs.r + self.r:
                collision=True
                break

        if not collision:
            self.x=newx
            self.y=newy

        if math.hypot(vx,vy)>0.001:
            self.alpha = math.atan2(vy,vx)

        self.path.append((self.x,self.y))

class ControlSystem:

    def __init__(self,robot,goal1,goal2):

        self.robot=robot

        self.goal1=goal1
        self.goal2=goal2

        self.current_goal=goal1

        self.waiting=False
        self.wait_start=None
        self.stage=1

    def compute(self,obstacles):

        rx,ry=self.robot.x,self.robot.y

        gx,gy=self.current_goal

        dist_goal = distance(rx,ry,gx,gy)

        # ---------- ARRIVAL ----------
        if dist_goal < 10:

            if self.stage == 1:

                if not self.waiting:
                    self.waiting=True
                    self.wait_start=time.time()

                if time.time()-self.wait_start < WAIT_TIME:
                    return 0,0

                else:
                    self.waiting=False
                    self.stage=2
                    self.current_goal=self.goal2

            else:
                return 0,0

        # ---------- POTENTIAL FIELD ----------

        # attraction
        fx = gx - rx
        fy = gy - ry

        d = math.hypot(fx,fy)

        fx/=d
        fy/=d

        # repulsion
        for obs in obstacles:

            dx = rx-obs.x
            dy = ry-obs.y

            d = math.hypot(dx,dy)

            if d < SAFE_DISTANCE:

                rep = 1/(d+0.1)

                fx += rep*dx
                fy += rep*dy

        # normalize
        mag = math.hypot(fx,fy)

        if mag>0:
            fx/=mag
            fy/=mag

        vx = fx*MAX_SPEED
        vy = fy*MAX_SPEED

        return vx,vy


# ================== ENVIRONMENT ==================

def generate_position(exclude):

    while True:

        x=random.randint(100,700)
        y=random.randint(100,500)

        valid=True

        for e in exclude:
            if distance(x,y,e[0],e[1]) < 100:
                valid=False
                break

        if valid:
            return (x,y)

# start
start=(100,100)

# obstacles
obstacles=[]
coords=[start]

for i in range(NUM_OBSTACLES):

    pos=generate_position(coords)
    coords.append(pos)
    obstacles.append(Obstacle(pos[0],pos[1]))

goal1=generate_position(coords)
coords.append(goal1)

goal2=generate_position(coords)

# ================== CREATE ROBOT ==================

robot = Robot(start[0],start[1],0)

control = ControlSystem(robot,goal1,goal2)

# ================== MAIN LOOP ==================

running=True

while running:

    clock.tick(60)

    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False

    robot.lidar.scan(obstacles)

    vx,vy = control.compute(obstacles)

    robot.move(vx,vy,obstacles)

    # ================== DRAW ==================

    screen.fill((25,25,25))

    # obstacles
    for obs in obstacles:
        obs.draw(screen)

    # goals
    pygame.draw.circle(screen,(60,220,60),
                       (int(goal1[0]),int(goal1[1])),GOAL1_RADIUS)

    pygame.draw.circle(screen,(60,160,255),
                       (int(goal2[0]),int(goal2[1])),GOAL2_RADIUS)

    # lidar
    robot.lidar.draw(screen)

    # robot
    robot.draw(screen)

    txt=font.render("Station № : "+str(control.stage),True,(255,255,255))
    screen.blit(txt,(10,10))

    pygame.display.flip()

pygame.quit()