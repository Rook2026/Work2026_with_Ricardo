import pygame
import math
import sys
import copy

# Инициализация Pygame
pygame.init()

# Константы окна
WIDTH, HEIGHT = 800, 600
FPS = 60
BACKGROUND_COLOR = (255, 255, 255)
POINT_COLOR = (0, 0, 255)
POINT_RADIUS = 6
LINE_COLOR = (0, 200, 0)
FINAL_PATH_COLOR = (255, 0, 0)
TEXT_COLOR = (0, 0, 0)

# ========== 1. Класс целевой точки (Pt) ==========
class Pt:
    def __init__(self, x, y, index=None):
        self.x = x
        self.y = y
        self.index = index  # порядковый номер точки

    def get_pos(self):
        return (self.x, self.y)

    def distance_to(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)

    def __repr__(self):
        return f"Pt({self.x}, {self.y})"

# ========== 2. Класс маршрута (Route) ==========
class Route:
    def __init__(self, points=None):
        self.points = points if points is not None else []

    def add_point(self, point):
        self.points.append(point)

    def get_length(self):
        """Вычисляет длину маршрута"""
        if len(self.points) < 2:
            return 0
        length = 0
        for i in range(len(self.points) - 1):
            length += self.points[i].distance_to(self.points[i + 1])
        # Замыкаем маршрут (возврат в начало)
        length += self.points[-1].distance_to(self.points[0])
        return length

    def __repr__(self):
        return f"Route({[p.index for p in self.points]})"

# ========== 3. Класс узла графа декомпозиций (Node) ==========
class Node:
    def __init__(self, matrix, upper_bound, fixed_edges=None, excluded_edges=None):
        """
        matrix: матрица расстояний для подзадачи
        upper_bound: верхняя оценка длины маршрута
        fixed_edges: список рёбер, которые обязательно входят в маршрут [(i, j), ...]
        excluded_edges: список рёбер, которые запрещены [(i, j), ...]
        """
        self.matrix = matrix  # копия матрицы расстояний
        self.upper_bound = upper_bound
        self.fixed_edges = fixed_edges if fixed_edges is not None else []
        self.excluded_edges = excluded_edges if excluded_edges is not None else []
        self.children = []
        self.best_route = None

    def add_child(self, child):
        self.children.append(child)

# ========== 4. Функция расчёта длины маршрута ==========
def calculate_route_length(route_points, dist_matrix):
    """Расчёт длины маршрута по матрице расстояний"""
    if len(route_points) < 2:
        return 0
    length = 0
    for i in range(len(route_points) - 1):
        length += dist_matrix[route_points[i].index][route_points[i + 1].index]
    length += dist_matrix[route_points[-1].index][route_points[0].index]
    return length

# ========== 5. Функция расчёта верхней оценки (жадный алгоритм) ==========
def calculate_upper_bound(matrix, points):
    """
    Вычисляет верхнюю оценку длины маршрута с помощью жадного алгоритма (ближайший сосед)
    """
    n = len(matrix)
    if n <= 1:
        return 0

    visited = [False] * n
    path = [0]
    visited[0] = True
    total = 0

    for _ in range(n - 1):
        current = path[-1]
        # Находим ближайшего непосещённого соседа
        best_dist = float('inf')
        best_idx = -1
        for j in range(n):
            if not visited[j] and matrix[current][j] < best_dist:
                best_dist = matrix[current][j]
                best_idx = j
        if best_idx != -1:
            path.append(best_idx)
            visited[best_idx] = True
            total += best_dist

    # Возвращаемся в начало
    total += matrix[path[-1]][path[0]]
    return total

# ========== 6. Функция выбора наилучшей дуги для декомпозиции ==========
def select_best_edge(matrix):
    """
    Выбирает дугу (ребро) для декомпозиции.
    Используется эвристика: выбираем ребро с максимальной стоимостью,
    так как его включение/исключение сильно разбивает пространство решений.
    """
    n = len(matrix)
    best_i, best_j = -1, -1
    max_dist = -1

    for i in range(n):
        for j in range(n):
            if i != j and matrix[i][j] != float('inf') and matrix[i][j] > max_dist:
                # Избегаем симметричных дублей
                if best_i == -1 or matrix[i][j] > max_dist:
                    max_dist = matrix[i][j]
                    best_i, best_j = i, j
    return (best_i, best_j)

# ========== 7. Функция получения подматрицы с фиксированной дугой ==========
def get_submatrix_with_edge(matrix, fixed_edge):
    """
    Возвращает новую матрицу, где зафиксировано включение ребра fixed_edge.
    При включении ребра (i, j) удаляются i-я строка и j-й столбец,
    а также запрещаются рёбра, ведущие в i или из j.
    """
    n = len(matrix)
    i, j = fixed_edge

    # Создаём новую матрицу без строки i и столбца j
    new_size = n - 1
    new_matrix = [[float('inf')] * new_size for _ in range(new_size)]

    # Переиндексация
    row_map = [r for r in range(n) if r != i]
    col_map = [c for c in range(n) if c != j]

    for new_r, old_r in enumerate(row_map):
        for new_c, old_c in enumerate(col_map):
            new_matrix[new_r][new_c] = matrix[old_r][old_c]

    return new_matrix

# ========== 8. Функция редукции матрицы (нижняя оценка) ==========
def reduce_matrix(matrix):
    """
    Приводит матрицу: вычитает минимумы из строк и столбцов.
    Возвращает приведённую матрицу и сумму констант приведения (нижнюю оценку)
    """
    n = len(matrix)
    if n == 0:
        return matrix, 0

    reduced = [row[:] for row in matrix]
    lower_bound = 0

    # Приведение по строкам
    for i in range(n):
        min_val = min(reduced[i])
        if min_val > 0 and min_val != float('inf'):
            lower_bound += min_val
            for j in range(n):
                if reduced[i][j] != float('inf'):
                    reduced[i][j] -= min_val

    # Приведение по столбцам
    for j in range(n):
        min_val = float('inf')
        for i in range(n):
            if reduced[i][j] < min_val:
                min_val = reduced[i][j]
        if min_val > 0 and min_val != float('inf'):
            lower_bound += min_val
            for i in range(n):
                if reduced[i][j] != float('inf'):
                    reduced[i][j] -= min_val

    return reduced, lower_bound

# ========== 9. Рекурсивное формирование узлов декомпозиции ==========
def branch_and_bound(root_node, points, original_matrix, best_solution):
    """
    Рекурсивный алгоритм ветвей и границ для решения TSP
    """
    n = len(root_node.matrix)

    # Если матрица 2x2, вычисляем финальный маршрут
    if n <= 2:
        # Восстанавливаем полный маршрут
        route = []
        used_edges = root_node.fixed_edges.copy()

        # Строим маршрут из фиксированных рёбер
        if len(used_edges) == len(points):
            # Восстанавливаем порядок
            adj = {}
            for u, v in used_edges:
                adj[u] = v

            start = used_edges[0][0]
            current = start
            for _ in range(len(points)):
                route.append(current)
                current = adj.get(current)
                if current is None:
                    break

        # Вычисляем длину маршрута
        route_length = 0
        for k in range(len(route) - 1):
            route_length += original_matrix[route[k]][route[k + 1]]
        if len(route) > 1:
            route_length += original_matrix[route[-1]][route[0]]

        if route_length < best_solution[0]:
            best_solution[0] = route_length
            best_solution[1] = route
        return

    # Выбираем наилучшее ребро для декомпозиции
    edge = select_best_edge(root_node.matrix)
    if edge[0] == -1:
        return

    i, j = edge

    # --- Ветвь 1: Включаем ребро ---
    # Проверяем, не создаёт ли ребро подцикл
    creates_cycle = False
    # Простая проверка: если ребро замыкает цикл, не включая все вершины
    temp_edges = root_node.fixed_edges + [(i, j)]
    # Проверка на преждевременное замыкание
    adj_count = {}
    for u, v in temp_edges:
        adj_count[u] = adj_count.get(u, 0) + 1
        adj_count[v] = adj_count.get(v, 0) + 1
    # Если какая-то вершина имеет степень 2, это нормально для гамильтонова цикла
    # Более сложная проверка потребовала бы поиска циклов

    if not creates_cycle:
        # Создаём новую матрицу с включённым ребром
        new_matrix = get_submatrix_with_edge(root_node.matrix, (i, j))
        new_fixed = root_node.fixed_edges + [(i, j)]

        # Приводим матрицу и вычисляем нижнюю оценку
        reduced_matrix, reduction_cost = reduce_matrix(new_matrix)

        # Нижняя оценка = родительская оценка + стоимость ребра + константы приведения
        edge_cost = root_node.matrix[i][j]
        lower_bound = (root_node.upper_bound - edge_cost) + reduction_cost

        child_node = Node(new_matrix, lower_bound, new_fixed, root_node.excluded_edges)

        if lower_bound < best_solution[0]:
            root_node.add_child(child_node)
            branch_and_bound(child_node, points, original_matrix, best_solution)

    # --- Ветвь 2: Исключаем ребро ---
    new_matrix_excl = [row[:] for row in root_node.matrix]
    new_matrix_excl[i][j] = float('inf')
    new_matrix_excl[j][i] = float('inf')  # симметрично

    # Приводим матрицу
    reduced_matrix_excl, reduction_cost_excl = reduce_matrix(new_matrix_excl)

    lower_bound_excl = root_node.upper_bound + reduction_cost_excl

    excl_node = Node(reduced_matrix_excl, lower_bound_excl, root_node.fixed_edges,
                     root_node.excluded_edges + [(i, j)])

    if lower_bound_excl < best_solution[0]:
        root_node.add_child(excl_node)
        branch_and_bound(excl_node, points, original_matrix, best_solution)

# ========== 10. Построение полного маршрута из индексов ==========
def build_route_from_indices(indices, points):
    """Преобразует список индексов в список объектов Pt"""
    route = Route()
    for idx in indices:
        route.add_point(points[idx])
    return route

# ========== 11. Главная функция ==========
def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("TSP - Branch and Bound Method")
    clock = pygame.time.Clock()

    # Входные координаты точек (10 точек)
    coords = [
        (100, 100), (150, 200), (250, 200), (250, 100),
        (150, 150), (350, 100), (150, 300), (200, 150),
        (250, 300), (100, 200)
    ]

    # Создаём объекты точек
    points = []
    for i, (x, y) in enumerate(coords):
        points.append(Pt(x, y, i))

    # Построение матрицы расстояний
    n = len(points)
    dist_matrix = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                dist_matrix[i][j] = points[i].distance_to(points[j])
            else:
                dist_matrix[i][j] = float('inf')

    # Вычисляем верхнюю оценку (жадный алгоритм)
    upper_bound = calculate_upper_bound(dist_matrix, points)
    print(f"Начальная верхняя оценка: {upper_bound:.2f}")

    # Приводим исходную матрицу
    reduced_matrix, lower_bound = reduce_matrix(dist_matrix)
    print(f"Нижняя оценка: {lower_bound:.2f}")

    # Создаём корневой узел
    root_node = Node(reduced_matrix, lower_bound)

    # Лучшее решение [длина, маршрут]
    best_solution = [float('inf'), []]

    # Запускаем алгоритм ветвей и границ
    print("Запуск алгоритма ветвей и границ...")
    branch_and_bound(root_node, points, dist_matrix, best_solution)

    # Получаем лучший маршрут
    best_route_indices = best_solution[1]
    if not best_route_indices:
        # Если не нашли, используем жадный алгоритм
        best_route_indices = [0]
        visited = [False] * n
        visited[0] = True
        current = 0
        for _ in range(n - 1):
            best = -1
            best_dist = float('inf')
            for j in range(n):
                if not visited[j] and dist_matrix[current][j] < best_dist:
                    best_dist = dist_matrix[current][j]
                    best = j
            if best != -1:
                best_route_indices.append(best)
                visited[best] = True
                current = best
        best_route_indices.append(0)  # возврат в начало

    best_route = build_route_from_indices(best_route_indices, points)
    final_length = best_route.get_length()
    print(f"Лучший маршрут: {[p.index for p in best_route.points]}")
    print(f"Длина маршрута: {final_length:.2f}")

    # Визуализация
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(BACKGROUND_COLOR)

        # Рисуем все точки
        for pt in points:
            pygame.draw.circle(screen, POINT_COLOR, pt.get_pos(), POINT_RADIUS)
            # Подписываем номер точки
            font = pygame.font.Font(None, 20)
            text = font.render(str(pt.index), True, TEXT_COLOR)
            screen.blit(text, (pt.x + 8, pt.y - 8))

        # Рисуем лучший маршрут
        if len(best_route.points) > 1:
            for i in range(len(best_route.points) - 1):
                pygame.draw.line(screen, FINAL_PATH_COLOR,
                                 best_route.points[i].get_pos(),
                                 best_route.points[i + 1].get_pos(), 3)
            # Замыкающая линия (возврат в начало)
            pygame.draw.line(screen, FINAL_PATH_COLOR,
                             best_route.points[-1].get_pos(),
                             best_route.points[0].get_pos(), 3)

        # Отображаем информацию
        font = pygame.font.Font(None, 24)
        info_text = f"TSP - Branch and Bound | Length: {final_length:.2f}"
        text_surface = font.render(info_text, True, TEXT_COLOR)
        screen.blit(text_surface, (10, 10))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()