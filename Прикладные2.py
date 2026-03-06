import pygame
import random
import math
import numpy as np

# CONFIG
WIDTH = 800
HEIGHT = 600

ROBOT_RADIUS = 20
OBJ_RADIUS = 10

DROP_POINT = np.array([500, 500])

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Transport Robot FSM")
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 18)

# OBJECT CLASS
class Obj:
    def __init__(self, x, y):
        self.pos = np.array([x, y], dtype=float)
        self.radius = OBJ_RADIUS
        self.collected = False

    def draw(self, screen):
        if not self.collected:
            pygame.draw.circle(screen, (0, 200, 0), self.pos.astype(int), self.radius)

# ROBOT CLASS
class Robot:
    def __init__(self, x, y, alpha):
        self.pos = np.array([x, y], dtype=float)
        self.alpha = alpha
        self.radius = ROBOT_RADIUS

        self.speed = 2
        self.rot_speed = 0.05

        self.attachedObj = None

        self.path = []

    def move(self, target):

        error = target - self.pos

        dist = np.linalg.norm(error)

        angle_to_target = math.atan2(error[1], error[0])

        angle_error = angle_to_target - self.alpha

        # Normaliser angle dans [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        self.alpha += angle_error * self.rot_speed

        vx = self.speed * math.cos(self.alpha)
        vy = self.speed * math.sin(self.alpha)

        self.pos += np.array([vx, vy])

        self.path.append(self.pos.copy())

        return dist

    def draw(self, screen):

        pygame.draw.circle(screen, (0, 0, 255), self.pos.astype(int), self.radius)

        head = self.pos + np.array([
            self.radius * math.cos(self.alpha),
            self.radius * math.sin(self.alpha)
        ])

        pygame.draw.line(screen, (255, 255, 0), self.pos.astype(int), head.astype(int), 3)

        # tracer la trajectoire
        for i in range(1, len(self.path)):
            pygame.draw.line(screen, (200, 200, 200),
                             self.path[i - 1].astype(int),
                             self.path[i].astype(int), 2)

        if self.attachedObj:
            self.attachedObj.pos = self.pos.copy()



# FSM CLASS
class FSM:

    def __init__(self):
        self.state = "Ready"

    def set(self, state):
        self.state = state


# CONTROL SYSTEM (corrigé)
class ControlSystem:
    def __init__(self, robot, objects, fsm):
        self.robot = robot
        self.objects = objects
        self.fsm = fsm
        self.currentObj = None
        self.delivered_count = 0
        self.state_timer = 0  # nouveau : compteur de frames pour afficher Ready/Finished

    def update(self):
        #READY 
        if self.fsm.state == "Ready":
            self.state_timer += 1
            # attendre quelques frames pour afficher Ready
            if self.state_timer > 30:  # environ 0.5 sec à 60 fps
                remaining_objects = [obj for obj in self.objects if not obj.collected]
                if remaining_objects:
                    remaining_objects.sort(key=lambda o: np.linalg.norm(o.pos - self.robot.pos))
                    self.currentObj = remaining_objects[0]
                    self.fsm.set("Approaching")
                    self.state_timer = 0  # réinitialiser compteur

        #APPROACHING 
        elif self.fsm.state == "Approaching":
            target = self.currentObj.pos
            dist = self.robot.move(target)
            if dist < 25:
                self.robot.attachedObj = self.currentObj
                self.currentObj.collected = True
                self.fsm.set("Moving")

        #MOVING 
        elif self.fsm.state == "Moving":
            dist = self.robot.move(DROP_POINT)
            if dist < 30:
                self.robot.attachedObj = None
                self.fsm.set("Finished")
                self.delivered_count += 1
                self.currentObj = None
                self.state_timer = 0  # démarrer timer pour afficher Finished

        #FINISHED 
        elif self.fsm.state == "Finished":
            self.state_timer += 1
            if self.state_timer > 30:  # afficher Finished ~0.5 sec
                remaining = any(not obj.collected for obj in self.objects)
                if remaining:
                    self.fsm.set("Ready")
                self.state_timer = 0

# INITIALIZATION
robot = Robot(100, 100, 0)

objects = []

for i in range(10):

    x = random.randint(50, WIDTH - 50)
    y = random.randint(50, HEIGHT - 50)

    objects.append(Obj(x, y))

fsm = FSM()

control = ControlSystem(robot, objects, fsm)

# MAIN LOOP
running = True

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    control.update()

    screen.fill((30, 30, 30))

    # point de dépôt
    pygame.draw.circle(screen, (255, 0, 0), DROP_POINT.astype(int), 15)

    # dessiner objets
    for obj in objects:
        obj.draw(screen)

    # dessiner robot
    robot.draw(screen)

    # afficher état
    text_state = font.render("State: " + fsm.state, True, (255, 255, 255))
    screen.blit(text_state, (10, 10))

    # afficher compteur d'objets déposés
    text_count = font.render(f"Number of object deposited: {control.delivered_count}/{len(objects)}", True, (255, 255, 255))
    screen.blit(text_count, (10, 30))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()