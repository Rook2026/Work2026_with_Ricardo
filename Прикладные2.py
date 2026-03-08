"""
Autonomous Transport Robot Simulation

Simulation d'un robot mobile capable de :
- détecter les objets
- choisir l'objet le plus proche
- le transporter vers une zone de dépôt

Architecture :
Robot -> mouvement
Obj -> objets à transporter
FSM -> machine à états
ControlSystem -> logique de décision

"""

import pygame
import random
import math
import numpy as np


# CONFIGURATION

WIDTH: int = 800
HEIGHT: int = 600
FPS: int = 60

ROBOT_RADIUS: int = 20
OBJ_RADIUS: int = 10

ROBOT_SPEED: float = 2
ROBOT_ROT_SPEED: float = 0.05

NUM_OBJECTS: int = 10

DROP_POINT = np.array([500, 500])



# INITIALISATION PYGAME

pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Autonomous Transport Robot")

clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 18)


# CLASSE OBJET

class Obj:
    """
    Représente un objet à transporter
    """

    def __init__(self, x: float, y: float):

        self.pos = np.array([x, y], dtype=float)
        self.radius = OBJ_RADIUS
        self.collected = False

    def draw(self, screen):

        if not self.collected:

            pygame.draw.circle(
                screen,
                (0, 200, 0),
                self.pos.astype(int),
                self.radius
            )


# CLASSE ROBOT

class Robot:
    """
    Robot mobile avec orientation et déplacement
    """

    def __init__(self, x: float, y: float, alpha: float):

        self.pos = np.array([x, y], dtype=float)
        self.alpha = alpha

        self.radius = ROBOT_RADIUS

        self.speed = ROBOT_SPEED
        self.rot_speed = ROBOT_ROT_SPEED

        self.attached_obj = None

        self.path = []

    def move(self, target: np.ndarray) -> float:
        """
        Déplacement vers une cible
        """

        error = target - self.pos

        dist = np.linalg.norm(error)

        angle_target = math.atan2(error[1], error[0])

        angle_error = angle_target - self.alpha

        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        self.alpha += angle_error * self.rot_speed

        vx = self.speed * math.cos(self.alpha)
        vy = self.speed * math.sin(self.alpha)

        self.pos += np.array([vx, vy])

        self.path.append(self.pos.copy())

        return dist


    def draw(self, screen):

        # Corps du robot
        pygame.draw.circle(
            screen,
            (0, 0, 255),
            self.pos.astype(int),
            self.radius
        )

        # Triangle d'orientation
        size = self.radius
        angle = self.alpha

        points = [
            self.pos + np.array([math.cos(angle)*size, math.sin(angle)*size]),
            self.pos + np.array([math.cos(angle+2.5)*size*0.6, math.sin(angle+2.5)*size*0.6]),
            self.pos + np.array([math.cos(angle-2.5)*size*0.6, math.sin(angle-2.5)*size*0.6])
        ]

        pygame.draw.polygon(
            screen,
            (255, 255, 0),
            [p.astype(int) for p in points]
        )

        # Trajectoire du robot
        for i in range(1, len(self.path)):

            pygame.draw.line(
                screen,
                (200, 200, 200),
                self.path[i-1].astype(int),
                self.path[i].astype(int),
                2
            )

        if self.attached_obj:
            self.attached_obj.pos = self.pos.copy()

# MACHINE A ETATS (FSM)

class FSM:

    READY = "Ready"
    APPROACHING = "Approaching"
    MOVING = "Moving"
    FINISHED = "Finished"

    def __init__(self):

        self.state = FSM.READY

    def set(self, state: str):

        self.state = state



# SYSTEME DE CONTROLE

class ControlSystem:

    def __init__(self, robot: Robot, objects: list, fsm: FSM):

        self.robot = robot
        self.objects = objects
        self.fsm = fsm

        self.current_obj = None
        self.delivered_count = 0

    def get_closest_object(self):

        remaining = [obj for obj in self.objects if not obj.collected]

        if not remaining:
            return None

        remaining.sort(
            key=lambda o: np.linalg.norm(o.pos - self.robot.pos)
        )

        return remaining[0]


    def update(self):

        if self.fsm.state == FSM.READY:

            if self.current_obj is None:

                self.current_obj = self.get_closest_object()

                if self.current_obj:
                    self.fsm.set(FSM.APPROACHING)

        elif self.fsm.state == FSM.APPROACHING:

            target = self.current_obj.pos

            dist = self.robot.move(target)

            if dist < 25:

                self.robot.attached_obj = self.current_obj
                self.current_obj.collected = True

                self.fsm.set(FSM.MOVING)

        elif self.fsm.state == FSM.MOVING:

            dist = self.robot.move(DROP_POINT)

            if dist < 30:

                self.robot.attached_obj = None

                self.delivered_count += 1
                self.current_obj = None

                self.fsm.set(FSM.FINISHED)

        elif self.fsm.state == FSM.FINISHED:

            if any(not obj.collected for obj in self.objects):

                self.fsm.set(FSM.READY)


# CREATION DES OBJETS

def create_objects(n: int):

    objects = []

    for _ in range(n):

        x = random.randint(50, WIDTH-50)
        y = random.randint(50, HEIGHT-50)

        objects.append(Obj(x, y))

    return objects


# PROGRAMME PRINCIPAL

def main():

    robot = Robot(100, 100, 0)

    objects = create_objects(NUM_OBJECTS)

    fsm = FSM()

    control = ControlSystem(robot, objects, fsm)

    running = True

    while running:

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

        control.update()

        screen.fill((30, 30, 30))

        # zone de dépôt
        pygame.draw.circle(
            screen,
            (255, 0, 0),
            DROP_POINT.astype(int),
            15
        )

        # objets
        for obj in objects:
            obj.draw(screen)

        # robot
        robot.draw(screen)

        # affichage état
        text_state = font.render(
            f"State : {fsm.state}",
            True,
            (255, 255, 255)
        )

        screen.blit(text_state, (10, 10))

        # compteur
        text_count = font.render(
            f"Objects delivered : {control.delivered_count}/{len(objects)}",
            True,
            (255, 255, 255)
        )

        screen.blit(text_count, (10, 30))

        pygame.display.flip()

        clock.tick(FPS)

    pygame.quit()

# EXECUTION

if __name__ == "__main__":
    main()