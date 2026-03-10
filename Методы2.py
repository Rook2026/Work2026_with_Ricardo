from operator import le
import sys, pygame, numpy as np, math

pygame.font.init()
def draw_text(screen, s, x, y, sz=20, color=(0,0,0)): #отрисовка текста
    screen.blit(pygame.font.SysFont('Comic Sans MS', sz).render(s, True, (0,0,0)), (x,y))
def lim_ang(ang, arc=3.141592653589793): # ограничение угла в пределах +/-pi
    ang=ang%(2*arc); return ang + (2*arc if ang<-arc else -2*arc if ang>arc else 0)
def rot(v, ang): return np.dot([[-v[1], v[0]], v],[math.sin(ang), math.cos(ang)]) # поворот вектора на угол
# проверяем, находится ли точка внутри многоугольника
#TODO: вспомнить теорию этой функции
def pt_inside_ngon(point, vertices): #ngon_contains_pt
    (x, y), c = point, 0
    for i in range(len(vertices)):
        (x1, y1), (x2, y2) = vertices[i-1], vertices[i]
        if min(y1,y2) <= y < max(y1, y2):
            ratio = (y - y1) / (y2 - y1)
            c ^= (x - x1 < ratio*(x2 - x1))
    return c

def get_vec_ang(vec): return math.atan2(vec[1], vec[0])

class Cube:
    def __init__(self, x, y, color):
        self.x, self.y=x,y
        self.sz=30
        self.color=color
        self.highlighted=False
    def get_pos(self): return [self.x, self.y]
    def draw(self, screen):
        p1=[self.x-self.sz/2, self.y-self.sz/2]
        w = 5 if self.highlighted else 2
        pygame.draw.rect(screen, self.color, [*p1, self.sz, self.sz], w)
        pygame.draw.circle(screen, self.color, self.get_pos(), 3, 2)

class Task: #конечный автомат как единица знания робота о мире
    def __init__(self, cube):
        self.cube=cube
        self.state="none" #running, detecting, finished
    def run(self, robot): #проявление свойства "активности" знаний
        if self.state=="none":
            self.state = "running"
            print(self.state)
        elif self.state=="running":
            vec=np.subtract(self.cube.get_pos(), robot.get_pos())
            if np.linalg.norm(vec)<50:
                self.state = "detecting" #завершаем движение выдачей сигнала на распознавание
                print(self.state)
                return self.state
            ang=get_vec_ang(vec)
            dang=lim_ang(ang-robot.a)
            robot.vrot=0.5*dang
            robot.vlin=30
        elif self.state=="detecting":
            res=robot.check_object_visible(self.cube)
            if res: 
                self.state="finished"
                robot.vlin=robot.vrot=0
                print(self.state)
                return self.state #завершаем распознавание выдачей сигнала готовоности
        elif self.state=="finished":
            pass
        return self.state

class Robot:
    def __init__(self, x, y):
        self.radius, self.color=20, (0,0,0)
        self.x, self.y, self.a, self.vlin, self.vrot=x,y,0, 0,0
        self.fov=[[0,0], [60,-30], [60,30]] #поле зрения робота
    def get_pos(self): return [self.x, self.y]
    def get_real_fov(self):
        return [np.array(rot(p, self.a))+self.get_pos() for p in self.fov]
    def draw(self, screen):
        p1=np.array(self.get_pos())
        pygame.draw.circle(screen, self.color, p1, self.radius, 2)
        s,c=math.sin(self.a), math.cos(self.a)
        pygame.draw.line(screen, self.color, p1, p1+[self.radius*c, self.radius*s],2)
        pygame.draw.lines(screen, (0,255,0), True, self.get_real_fov(), 2)
    def sim(self, dt):
        s,c=math.sin(self.a), math.cos(self.a)
        self.x, self.y=self.x+c*self.vlin*dt, self.y+s*self.vlin*dt
        self.a=lim_ang(self.a+self.vrot*dt)
    def check_object_visible(self, obj):
        return pt_inside_ngon(obj.get_pos(), self.get_real_fov())

if __name__=="__main__":
    sz, timer, fps = (800, 600), pygame.time.Clock(), 20
    screen, dt = pygame.display.set_mode(sz), 1 / fps
    robot = Robot(200, 450)

    cubes = [
        Cube(200,200, (0,0,255)),
        Cube(400,250, (200,0,200)),
        Cube(600,400, (220,180,0))
    ]

    tasks = [Task(cube) for cube in cubes]
    active_task=tasks[0]

    while True:
        for ev in pygame.event.get():
            if ev.type==pygame.QUIT: sys.exit(0)
            if ev.type == pygame.KEYDOWN:
                robot.vlin = 50 if ev.key == pygame.K_w else -50 if ev.key == pygame.K_s else robot.vlin
                robot.vrot = -1 if ev.key == pygame.K_a else 1 if ev.key == pygame.K_d else robot.vrot

        if active_task: 
            active_task.run(robot)
            if active_task.state=="finished":
                print("strategic level: taking new task")
                remaining_tasks=[t for t in tasks if t.state!="finished"]
                if len(remaining_tasks):
                    active_task=remaining_tasks[0]
                else:
                    active_task=None

        robot.sim(dt)

        for cube in cubes:
            cube.highlighted=robot.check_object_visible(cube)

        screen.fill((255, 255, 255))
        robot.draw(screen)
        for cube in cubes: cube.draw(screen)

        draw_text(screen, f"Test = {1}", 5, 5)
        pygame.display.flip(), timer.tick(fps)

#template file by S. Diane, RTU MIREA, 2024-2025