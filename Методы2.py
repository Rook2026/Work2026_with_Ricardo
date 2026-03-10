import pygame
import sys


# Classe Objet

class Obj:
    def __init__(self, name, x):
        self.name = name
        self.x = x



# Classe Robot

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.speed = 2

    def move_to(self, target_x):
        if self.x < target_x:
            self.x += self.speed
        elif self.x > target_x:
            self.x -= self.speed



# Classe Event

class Event:
    def __init__(self, name):
        self.name = name



# Classe Graph

class Graph:
    def __init__(self):
        self.edges = []

    def add_edge(self, e1, e2):
        self.edges.append((e1, e2))

    def print_graph(self):
        print("\nGRAPH EVENTS")
        for e in self.edges:
            print(e[0], "->", e[1])



# PLANIFICATION

def create_plan(objects):

    names = [o.name for o in objects]
    target = list(reversed(names))

    plan = []

    for i in range(len(names)):
        plan.append(names[len(names)-1-i])

    print("\nPLAN ACTIONS")
    for i,a in enumerate(plan):
        print(i+1,"Move",a)

    return plan



# PYGAME INIT

pygame.init()

WIDTH = 800
HEIGHT = 600

screen = pygame.display.set_mode((WIDTH,HEIGHT))
pygame.display.set_caption("Robot Planning")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None,24)


# ROBOT

robot = Robot(100,300)


# OBJETS

objects = [

Obj("A",200),
Obj("B",320),
Obj("C",440),
Obj("D",560)

]


# GRAPH

graph = Graph()

graph.add_edge("Start","RobotMove")
graph.add_edge("RobotMove","ObjectMove")
graph.add_edge("ObjectMove","Goal")

graph.print_graph()


# PLAN

plan = create_plan(objects)

step = 0
target = None


# MAIN LOOP

running = True

while running:

    screen.fill((240,240,240))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # suivre le plan automatiquement
    if step < len(plan):

        target_obj = next(o for o in objects if o.name == plan[step])

        robot.move_to(target_obj.x)

        if abs(robot.x - target_obj.x) < 5:

            target_obj.x = 700 - target_obj.x
            step += 1


    # ===== dessin robot =====
    pygame.draw.circle(screen,(200,0,0),(int(robot.x),int(robot.y)),20)

    # roues
    pygame.draw.circle(screen,(0,0,0),(int(robot.x-15),robot.y+20),8)
    pygame.draw.circle(screen,(0,0,0),(int(robot.x+15),robot.y+20),8)

    # ===== objets =====
    for obj in objects:

        pygame.draw.circle(screen,(0,0,255),(int(obj.x),400),22)

        text = font.render(obj.name,True,(255,255,255))
        screen.blit(text,(obj.x-6,392))


    pygame.display.update()
    clock.tick(60)

pygame.quit()
sys.exit()