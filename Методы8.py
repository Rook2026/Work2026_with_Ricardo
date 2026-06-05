import pygame
import sys
import math
from enum import Enum

# Инициализация Pygame
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Крестики-нолики - Минимаксный алгоритм")
clock = pygame.time.Clock()
FPS = 60

# Цвета
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
DARK_GRAY = (100, 100, 100)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
LIGHT_BLUE = (173, 216, 230)
LIGHT_RED = (255, 182, 193)

class Player(Enum):
    """Игроки"""
    X = 1  # Крестик (человек)
    O = 2  # Нолик (компьютер)
    EMPTY = 0  # Пустая ячейка

class Cell:
    """Класс ячейки игрового поля"""
    def __init__(self, x, y):
        self.x = x  # Координата X в сетке
        self.y = y  # Координата Y в сетке
        self.player = Player.EMPTY  # Состояние ячейки
        self.is_winning = False  # Является ли частью выигрышной комбинации

    def set_player(self, player):
        """Установка игрока в ячейку"""
        if self.player == Player.EMPTY:
            self.player = player
            return True
        return False

    def clear(self):
        """Очистка ячейки"""
        self.player = Player.EMPTY
        self.is_winning = False

    def __str__(self):
        if self.player == Player.X:
            return "X"
        elif self.player == Player.O:
            return "O"
        return " "

class Table:
    """Класс игрового поля"""
    def __init__(self, size=3):
        self.size = size  # Размер поля
        self.cells = []  # Массив ячеек
        self.create_table()
        
    def create_table(self):
        """Создание игрового поля"""
        self.cells = []
        for y in range(self.size):
            row = []
            for x in range(self.size):
                row.append(Cell(x, y))
            self.cells.append(row)
    
    def get_cell(self, x, y):
        """Получение ячейки по координатам"""
        if 0 <= x < self.size and 0 <= y < self.size:
            return self.cells[y][x]
        return None
    
    def make_move(self, x, y, player):
        """Выполнение хода"""
        cell = self.get_cell(x, y)
        if cell and cell.player == Player.EMPTY:
            cell.set_player(player)
            return True
        return False
    
    def get_state(self):
        """Получение состояния поля в виде строки"""
        state = []
        for y in range(self.size):
            for x in range(self.size):
                cell = self.get_cell(x, y)
                if cell.player == Player.X:
                    state.append('X')
                elif cell.player == Player.O:
                    state.append('O')
                else:
                    state.append('-')
        return ''.join(state)
    
    def get_empty_cells(self):
        """Получение списка пустых ячеек"""
        empty = []
        for y in range(self.size):
            for x in range(self.size):
                cell = self.get_cell(x, y)
                if cell.player == Player.EMPTY:
                    empty.append((x, y))
        return empty
    
    def check_winner(self):
        """Проверка победителя"""
        # Проверка строк
        for y in range(self.size):
            if (self.cells[y][0].player != Player.EMPTY and
                self.cells[y][0].player == self.cells[y][1].player == self.cells[y][2].player):
                # Отмечаем выигрышные ячейки
                for x in range(self.size):
                    self.cells[y][x].is_winning = True
                return self.cells[y][0].player
        
        # Проверка столбцов
        for x in range(self.size):
            if (self.cells[0][x].player != Player.EMPTY and
                self.cells[0][x].player == self.cells[1][x].player == self.cells[2][x].player):
                for y in range(self.size):
                    self.cells[y][x].is_winning = True
                return self.cells[0][x].player
        
        # Проверка главной диагонали
        if (self.cells[0][0].player != Player.EMPTY and
            self.cells[0][0].player == self.cells[1][1].player == self.cells[2][2].player):
            for i in range(self.size):
                self.cells[i][i].is_winning = True
            return self.cells[0][0].player
        
        # Проверка побочной диагонали
        if (self.cells[0][2].player != Player.EMPTY and
            self.cells[0][2].player == self.cells[1][1].player == self.cells[2][0].player):
            for i in range(self.size):
                self.cells[i][self.size - 1 - i].is_winning = True
            return self.cells[0][2].player
        
        return None
    
    def is_full(self):
        """Проверка заполненности поля"""
        return len(self.get_empty_cells()) == 0
    
    def is_game_over(self):
        """Проверка окончания игры"""
        return self.check_winner() is not None or self.is_full()
    
    def clear_winning_marks(self):
        """Очистка отметок выигрышных ячеек"""
        for y in range(self.size):
            for x in range(self.size):
                self.cells[y][x].is_winning = False
    
    def clone(self):
        """Создание копии поля для симуляции"""
        new_table = Table(self.size)
        for y in range(self.size):
            for x in range(self.size):
                new_table.cells[y][x].player = self.cells[y][x].player
        return new_table
    
    def reset(self):
        """Сброс поля"""
        self.clear_winning_marks()
        for y in range(self.size):
            for x in range(self.size):
                self.cells[y][x].clear()

class MinimaxAI:
    """Класс минимаксного алгоритма"""
    def __init__(self, player=Player.O):
        self.player = player  # Игрок, за которого играет AI
        self.opponent = Player.X if player == Player.O else Player.O
        self.nodes_evaluated = 0  # Счетчик оцененных узлов
    
    def get_score(self, table, depth):
        """
        Функция оценки цены игры
        Возвращает оценку позиции с точки зрения AI
        """
        winner = table.check_winner()
        
        if winner == self.player:
            # Победа AI - высокая оценка (уменьшается с глубиной для приоритета быстрых побед)
            return 10 - depth
        elif winner == self.opponent:
            # Поражение AI - низкая оценка
            return depth - 10
        elif table.is_full():
            # Ничья
            return 0
        
        return None  # Игра не закончена
    
    def minimax(self, table, depth, is_maximizing, alpha=-math.inf, beta=math.inf):
        """
        Минимаксный алгоритм с альфа-бета отсечением
        is_maximizing: True - ход AI (максимизация), False - ход противника (минимизация)
        """
        self.nodes_evaluated += 1
        
        score = self.get_score(table, depth)
        if score is not None:
            return score
        
        if is_maximizing:
            # Ход AI - максимизация оценки
            max_eval = -math.inf
            empty_cells = table.get_empty_cells()
            
            # Сортировка ходов для улучшения альфа-бета отсечения
            # Предпочитаем центр и углы
            empty_cells.sort(key=lambda pos: self._move_priority(pos), reverse=True)
            
            for x, y in empty_cells:
                new_table = table.clone()
                new_table.make_move(x, y, self.player)
                
                eval = self.minimax(new_table, depth + 1, False, alpha, beta)
                max_eval = max(max_eval, eval)
                alpha = max(alpha, eval)
                
                if beta <= alpha:
                    break  # Альфа-бета отсечение
            
            return max_eval
        else:
            # Ход противника - минимизация оценки
            min_eval = math.inf
            empty_cells = table.get_empty_cells()
            
            empty_cells.sort(key=lambda pos: self._move_priority(pos), reverse=True)
            
            for x, y in empty_cells:
                new_table = table.clone()
                new_table.make_move(x, y, self.opponent)
                
                eval = self.minimax(new_table, depth + 1, True, alpha, beta)
                min_eval = min(min_eval, eval)
                beta = min(beta, eval)
                
                if beta <= alpha:
                    break  # Альфа-бета отсечение
            
            return min_eval
    
    def _move_priority(self, pos):
        """Приоритет хода для сортировки"""
        x, y = pos
        # Центр имеет наивысший приоритет
        if x == 1 and y == 1:
            return 4
        # Углы
        elif (x == 0 or x == 2) and (y == 0 or y == 2):
            return 3
        # Боковые клетки
        else:
            return 1
    
    def get_best_action(self, table):
        """
        Функция определения наилучшего хода
        Возвращает координаты (x, y) лучшего хода
        """
        self.nodes_evaluated = 0
        best_score = -math.inf
        best_move = None
        empty_cells = table.get_empty_cells()
        
        # Сортировка ходов по приоритету
        empty_cells.sort(key=lambda pos: self._move_priority(pos), reverse=True)
        
        for x, y in empty_cells:
            new_table = table.clone()
            new_table.make_move(x, y, self.player)
            
            # Вызываем минимакс для оценки этого хода
            score = self.minimax(new_table, 0, False, -math.inf, math.inf)
            
            if score > best_score:
                best_score = score
                best_move = (x, y)
        
        print(f"Оценено узлов: {self.nodes_evaluated}")
        print(f"Лучшая оценка: {best_score}")
        
        return best_move

def draw_table(screen, table, offset_x, offset_y, cell_size):
    """Отрисовка игрового поля"""
    # Отрисовка сетки
    for y in range(table.size):
        for x in range(table.size):
            rect = pygame.Rect(
                offset_x + x * cell_size,
                offset_y + y * cell_size,
                cell_size,
                cell_size
            )
            
            cell = table.get_cell(x, y)
            
            # Определение цвета ячейки
            if cell.is_winning:
                color = GREEN
            elif cell.player == Player.EMPTY:
                color = WHITE
            else:
                color = LIGHT_BLUE if cell.player == Player.X else LIGHT_RED
            
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, BLACK, rect, 2)
            
            # Отрисовка символа
            if cell.player == Player.X:
                draw_x(screen, rect.center, cell_size // 3)
            elif cell.player == Player.O:
                draw_o(screen, rect.center, cell_size // 3)

def draw_x(screen, center, size):
    """Отрисовка крестика"""
    x, y = center
    pygame.draw.line(screen, BLUE, (x - size, y - size), (x + size, y + size), 5)
    pygame.draw.line(screen, BLUE, (x + size, y - size), (x - size, y + size), 5)

def draw_o(screen, center, size):
    """Отрисовка нолика"""
    pygame.draw.circle(screen, RED, center, size, 5)

def draw_info_panel(screen, x, y, game_state, current_player, ai_nodes):
    """Отрисовка информационной панели"""
    font_title = pygame.font.Font(None, 36)
    font_text = pygame.font.Font(None, 24)
    
    # Фон панели
    panel_rect = pygame.Rect(x - 10, y - 10, 250, 300)
    pygame.draw.rect(screen, WHITE, panel_rect)
    pygame.draw.rect(screen, BLACK, panel_rect, 2)
    
    # Заголовок
    title = font_title.render("ИНФОРМАЦИЯ", True, BLACK)
    screen.blit(title, (x + 40, y))
    
    y += 40
    
    # Состояние игры
    if game_state == "playing":
        state_text = "Игра идет"
        state_color = BLACK
    elif game_state == "X_wins":
        state_text = "Победили КРЕСТИКИ!"
        state_color = BLUE
    elif game_state == "O_wins":
        state_text = "Победили НОЛИКИ!"
        state_color = RED
    elif game_state == "draw":
        state_text = "НИЧЬЯ!"
        state_color = DARK_GRAY
    else:
        state_text = ""
        state_color = BLACK
    
    text_surface = font_text.render(state_text, True, state_color)
    screen.blit(text_surface, (x, y))
    y += 30
    
    # Текущий игрок
    if current_player == Player.X:
        player_text = "Ход: КРЕСТИКИ (Вы)"
        player_color = BLUE
    else:
        player_text = "Ход: НОЛИКИ (AI)"
        player_color = RED
    
    text_surface = font_text.render(player_text, True, player_color)
    screen.blit(text_surface, (x, y))
    y += 30
    
    # Статистика AI
    text_surface = font_text.render(f"Оценено узлов: {ai_nodes}", True, BLACK)
    screen.blit(text_surface, (x, y))
    y += 30
    
    # Подсказки
    y += 10
    text_surface = font_text.render("Управление:", True, BLACK)
    screen.blit(text_surface, (x, y))
    y += 25
    
    controls = [
        "Клик мышью - ваш ход",
        "R - новая игра",
        "ESC - выход"
    ]
    
    for control in controls:
        text_surface = pygame.font.Font(None, 20).render(control, True, DARK_GRAY)
        screen.blit(text_surface, (x, y))
        y += 20

def draw_legend(screen, x, y):
    """Отрисовка легенды"""
    font_title = pygame.font.Font(None, 28)
    font_text = pygame.font.Font(None, 22)
    
    # Фон легенды
    legend_rect = pygame.Rect(x - 10, y - 10, 250, 180)
    pygame.draw.rect(screen, WHITE, legend_rect)
    pygame.draw.rect(screen, BLACK, legend_rect, 2)
    
    # Заголовок
    title = font_title.render("ЛЕГЕНДА", True, BLACK)
    screen.blit(title, (x + 60, y))
    
    y += 40
    
    # Элементы легенды
    legend_items = [
        ("Крестик (X)", BLUE),
        ("Нолик (O)", RED),
        ("Победа", GREEN)
    ]
    
    for text, color in legend_items:
        if color == GREEN:
            pygame.draw.rect(screen, color, (x, y, 25, 25))
        else:
            pygame.draw.rect(screen, WHITE, (x, y, 25, 25))
            pygame.draw.rect(screen, BLACK, (x, y, 25, 25), 2)
            if color == BLUE:
                draw_x_mini(screen, (x + 12, y + 12), 8)
            else:
                draw_o_mini(screen, (x + 12, y + 12), 8)
        
        text_surface = font_text.render(text, True, BLACK)
        screen.blit(text_surface, (x + 35, y + 2))
        y += 35

def draw_x_mini(screen, center, size):
    """Отрисовка маленького крестика для легенды"""
    x, y = center
    pygame.draw.line(screen, BLUE, (x - size, y - size), (x + size, y + size), 3)
    pygame.draw.line(screen, BLUE, (x + size, y - size), (x - size, y + size), 3)

def draw_o_mini(screen, center, size):
    """Отрисовка маленького нолика для легенды"""
    pygame.draw.circle(screen, RED, center, size, 3)

def get_cell_from_mouse(mouse_pos, offset_x, offset_y, cell_size, table_size):
    """Определение ячейки по координатам мыши"""
    x = (mouse_pos[0] - offset_x) // cell_size
    y = (mouse_pos[1] - offset_y) // cell_size
    
    if 0 <= x < table_size and 0 <= y < table_size:
        return x, y
    return None

def main():
    """Основная функция"""
    # Параметры игрового поля
    TABLE_SIZE = 3
    CELL_SIZE = 150
    
    # Расчет отступов для центрирования
    offset_x = (WIDTH - TABLE_SIZE * CELL_SIZE) // 2 + 100  # Смещаем вправо для панели
    offset_y = (HEIGHT - TABLE_SIZE * CELL_SIZE) // 2
    
    # Создание игрового поля
    table = Table(TABLE_SIZE)
    
    # Создание AI
    ai = MinimaxAI(Player.O)
    
    # Игровые переменные
    current_player = Player.X  # Первыми ходят крестики
    game_state = "playing"  # Состояние игры
    ai_nodes = 0  # Количество оцененных узлов
    game_over = False
    
    # Главный цикл
    running = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    # Новая игра
                    table.reset()
                    current_player = Player.X
                    game_state = "playing"
                    game_over = False
                    ai_nodes = 0
                    print("\n=== НОВАЯ ИГРА ===")
            
            elif event.type == pygame.MOUSEBUTTONDOWN and not game_over:
                # Ход игрока (крестики)
                if current_player == Player.X:
                    cell_pos = get_cell_from_mouse(
                        pygame.mouse.get_pos(), offset_x, offset_y, CELL_SIZE, TABLE_SIZE
                    )
                    
                    if cell_pos:
                        x, y = cell_pos
                        if table.make_move(x, y, Player.X):
                            print(f"Игрок ходит: ({x}, {y})")
                            
                            # Проверка окончания игры
                            winner = table.check_winner()
                            if winner == Player.X:
                                game_state = "X_wins"
                                game_over = True
                                print("Победили КРЕСТИКИ!")
                            elif table.is_full():
                                game_state = "draw"
                                game_over = True
                                print("НИЧЬЯ!")
                            else:
                                # Передача хода AI
                                current_player = Player.O
        
        # Ход AI (нолики)
        if current_player == Player.O and not game_over:
            print("\nAI думает...")
            
            # Небольшая задержка для визуализации
            pygame.time.wait(500)
            
            best_move = ai.get_best_action(table)
            
            if best_move:
                x, y = best_move
                table.make_move(x, y, Player.O)
                ai_nodes = ai.nodes_evaluated
                print(f"AI ходит: ({x}, {y})")
                
                # Проверка окончания игры
                winner = table.check_winner()
                if winner == Player.O:
                    game_state = "O_wins"
                    game_over = True
                    print("Победили НОЛИКИ!")
                elif table.is_full():
                    game_state = "draw"
                    game_over = True
                    print("НИЧЬЯ!")
                else:
                    # Передача хода игроку
                    current_player = Player.X
        
        # Очистка экрана
        screen.fill(WHITE)
        
        # Заголовок
        font_title = pygame.font.Font(None, 48)
        title = font_title.render("КРЕСТИКИ-НОЛИКИ", True, BLACK)
        screen.blit(title, (WIDTH // 2 - title.get_width() // 2, 10))
        
        subtitle = pygame.font.Font(None, 28)
        sub = subtitle.render("Алгоритм Минимакс с α-β отсечением", True, DARK_GRAY)
        screen.blit(sub, (WIDTH // 2 - sub.get_width() // 2, 55))
        
        # Отрисовка игрового поля
        draw_table(screen, table, offset_x, offset_y, CELL_SIZE)
        
        # Отрисовка информационной панели
        draw_info_panel(screen, 20, 100, game_state, current_player, ai_nodes)
        
        # Отрисовка легенды
        draw_legend(screen, 20, 400)
        
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    print("=== КРЕСТИКИ-НОЛИКИ ===")
    print("Алгоритм Минимакс с альфа-бета отсечением")
    print("\nВы играете за КРЕСТИКИ (X)")
    print("AI играет за НОЛИКИ (O)")
    print("\nКликните мышью по ячейке для хода")
    print("R - новая игра")
    print("ESC - выход")
    main()
