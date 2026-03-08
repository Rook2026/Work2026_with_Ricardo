import pygame
import random
import math

# CONFIGURATION
WIDTH, HEIGHT = 800, 600
START = (100, 100)
GOAL = (500, 500)

ROBOT_RADIUS = 20
OBSTACLE_RADIUS = 30
NUM_OBSTACLES = 10

LIDAR_RANGE = 120
LIDAR_ANGLES = [-40, -20, 0, 20, 40]

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Expert Robot Navigation")
clock = pygame.time.Clock()


# OBSTACLE CLASS

class Obstacle:
    def __init__(self, x, y, r=OBSTACLE_RADIUS):
        self.x = x
        self.y = y
        self.r = r

    def draw(self, screen):
        pygame.draw.circle(screen, (200, 0, 0), (int(self.x), int(self.y)), self.r)


# LIDAR CLASS

class Lidar:

    def __init__(self, robot, max_range=LIDAR_RANGE):
        self.robot = robot
        self.max_range = max_range

    def scan(self, obstacles):

        distances = []

        for a in LIDAR_ANGLES:

            angle = self.robot.alpha + math.radians(a)
            dist = self.max_range

            for d in range(0, self.max_range, 3):

                x = self.robot.x + d * math.cos(angle)
                y = self.robot.y + d * math.sin(angle)

                for obs in obstacles:

                    if math.hypot(x - obs.x, y - obs.y) <= obs.r:
                        dist = d
                        break

                if dist != self.max_range:
                    break

            distances.append(dist)

        return distances

    def draw(self, screen, distances):

        for i, a in enumerate(LIDAR_ANGLES):

            angle = self.robot.alpha + math.radians(a)

            x = self.robot.x + distances[i] * math.cos(angle)
            y = self.robot.y + distances[i] * math.sin(angle)

            pygame.draw.line(screen, (0, 255, 0),
                             (self.robot.x, self.robot.y),
                             (x, y), 1)
            
# ROBOT CLASS

class Robot:

    def __init__(self, x, y, alpha=0):

        self.x = x
        self.y = y
        self.alpha = alpha

        self.v = 2

        self.trajectory = []

        self.lidar = Lidar(self)

    def move(self):

        self.x += self.v * math.cos(self.alpha)
        self.y += self.v * math.sin(self.alpha)

        self.trajectory.append((self.x, self.y))

    def reached_goal(self):

        dist = math.hypot(self.x - GOAL[0], self.y - GOAL[1])

        if dist < 10:
            return True

        return False

    def draw(self, screen):

        # robot body
        pygame.draw.circle(screen, (0, 0, 255),
                           (int(self.x), int(self.y)),
                           ROBOT_RADIUS)

        # orientation line (alpha)
        dir_x = self.x + ROBOT_RADIUS * math.cos(self.alpha)
        dir_y = self.y + ROBOT_RADIUS * math.sin(self.alpha)

        pygame.draw.line(screen, (255, 255, 255),
                         (self.x, self.y),
                         (dir_x, dir_y), 3)

        # directional triangle
        p1 = (self.x + ROBOT_RADIUS * math.cos(self.alpha),
              self.y + ROBOT_RADIUS * math.sin(self.alpha))

        p2 = (self.x + ROBOT_RADIUS * math.cos(self.alpha + 2.5),
              self.y + ROBOT_RADIUS * math.sin(self.alpha + 2.5))

        p3 = (self.x + ROBOT_RADIUS * math.cos(self.alpha - 2.5),
              self.y + ROBOT_RADIUS * math.sin(self.alpha - 2.5))

        pygame.draw.polygon(screen, (0, 150, 255), [p1, p2, p3])

        # trajectory
        if len(self.trajectory) > 1:
            pygame.draw.lines(screen, (255, 255, 0),
                              False, self.trajectory, 2)

# CONTROL SYSTEM

class ControlSystem:

    def __init__(self, robot):
        self.robot = robot

    def update(self, lidar_data):

        goal_dist = math.hypot(GOAL[0] - self.robot.x,
                               GOAL[1] - self.robot.y)

        if goal_dist < 10:
            self.robot.v = 0
            return

        left = lidar_data[0]
        front_left = lidar_data[1]
        front = lidar_data[2]
        front_right = lidar_data[3]
        right = lidar_data[4]

        # obstacle in front
        if front < 40:
            self.robot.alpha += 0.12
            self.robot.v = 1

        # obstacle left
        elif front_left < 40 or left < 40:
            self.robot.alpha += 0.07
            self.robot.v = 1.5

        # obstacle right
        elif front_right < 40 or right < 40:
            self.robot.alpha -= 0.07
            self.robot.v = 1.5

        # free path → go to goal
        else:

            goal_angle = math.atan2(
                GOAL[1] - self.robot.y,
                GOAL[0] - self.robot.x
            )

            self.robot.alpha += (goal_angle - self.robot.alpha) * 0.05
            self.robot.v = 2


# CREATE OBSTACLES

obstacles = []

for i in range(NUM_OBSTACLES):

    x = random.randint(80, 720)
    y = random.randint(80, 520)

    obstacles.append(Obstacle(x, y))


# INITIALIZE ROBOT

robot = Robot(START[0], START[1], 0)
control = ControlSystem(robot)


# MAIN LOOP

running = True

while running:

    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # lidar scan
    lidar_data = robot.lidar.scan(obstacles)

    # control decision
    control.update(lidar_data)

    # robot move
    if not robot.reached_goal():
        robot.move()

    else:
        robot.v = 0

    # DRAW
    screen.fill((30, 30, 30))

    # goal
    pygame.draw.circle(screen, (0, 255, 255), GOAL, 10)

    # obstacles
    for obs in obstacles:
        obs.draw(screen)

    # robot
    robot.draw(screen)

    # lidar
    robot.lidar.draw(screen, lidar_data)

    pygame.display.flip()

pygame.quit()