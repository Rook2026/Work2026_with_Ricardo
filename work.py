import pygame # Bibliotheque de creation du jeu en python
import math # Bibliotheque utilisé pour les calculs mathematique
import numpy as np # Bibliotheque utilise pour faire les calcules numerique
import random # bibliotheque des nombres aleatoires
import heapq # bibliotheque pour A* (l'algorithme du chemin)

LONG = 1000 # longueur de l'interface du jeu
LARG = 1000 # largeur de l'interface du jeu

CELL_LABY = 20 # taille de la grille du labyrinthe en longueur et largeur (1000/2)

R_ROBOT = 12 # rayon du robot
R_OBJ = 8 # rayon des objets placer de facon aleatoire dans le labyrinthe

P_DEPOT = np.array([500, 500])

pygame.init() # pour initialiser tous les modules de pygame

interface = pygame.display.set_mode((LONG, LARG)) #CREATION DE LA FENETRE DU JEU AVEC LES DIMENSIONS DE LONGUEUR ET LA LARGUEUR

#interface = pygame.display.set_mode(size=(0, 0), flags=0, depth=0, display=0, vsync=0)

pygame.display.set_caption("Mon apprentissage") # titre de ma fenetre

clock = pygame.time.clock() # creation de l'horloge qui permet de controler la vitesse du jeu (image par seconde)

font = pygame.font.SysFont("Times New Roman", 18) # definir la police et la taille des textes a l'affichage

#CREATION DU LABYRINTHE

def generate_maze():

    grid_w = LARG // CELL_LABY
    grid_h = LONG // CELL_LABY

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










































    pygame.display.flip()

    clock.tick(60)

pygame.quit()





















































