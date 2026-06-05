import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from collections import defaultdict
import random
import heapq
import matplotlib.pyplot as plt
import math

# ============================================================
# 1. НЕЧЁТКАЯ МОДЕЛЬ (та же, что и раньше, но надёжнее)
# ============================================================
soil = ctrl.Antecedent(np.arange(0, 1.05, 0.05), 'soil')
slope = ctrl.Antecedent(np.arange(-30, 31, 1), 'slope')
moisture = ctrl.Antecedent(np.arange(0, 1.05, 0.05), 'moisture')
cost = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'cost')

soil['solid'] = fuzz.trimf(soil.universe, [0, 0, 0.5])
soil['loose'] = fuzz.trimf(soil.universe, [0, 0.5, 1])
soil['muddy'] = fuzz.trimf(soil.universe, [0.5, 1, 1])

slope['negative_steep'] = fuzz.trapmf(slope.universe, [-30, -30, -15, -5])
slope['flat'] = fuzz.trimf(slope.universe, [-5, 0, 5])
slope['gentle_up'] = fuzz.trimf(slope.universe, [5, 10, 20])
slope['steep_up'] = fuzz.trapmf(slope.universe, [15, 25, 30, 30])

moisture['dry'] = fuzz.trimf(moisture.universe, [0, 0, 0.5])
moisture['damp'] = fuzz.trimf(moisture.universe, [0, 0.5, 1])
moisture['wet'] = fuzz.trimf(moisture.universe, [0.5, 1, 1])

cost['low'] = fuzz.trimf(cost.universe, [0, 0, 0.4])
cost['medium'] = fuzz.trimf(cost.universe, [0.2, 0.5, 0.8])
cost['high'] = fuzz.trimf(cost.universe, [0.6, 0.9, 1])
cost['extreme'] = fuzz.trimf(cost.universe, [0.9, 1, 1])

# Генерация правил (3x4x3 = 36)
soil_terms = ['solid', 'loose', 'muddy']
slope_terms = ['negative_steep', 'flat', 'gentle_up', 'steep_up']
moisture_terms = ['dry', 'damp', 'wet']

soil_weight = {'solid': 0.2, 'loose': 0.6, 'muddy': 0.9}
slope_weight = {'negative_steep': 0.1, 'flat': 0.3, 'gentle_up': 0.6, 'steep_up': 0.9}
moisture_weight = {'dry': 0.2, 'damp': 0.5, 'wet': 0.8}

rules = []
for s in soil_terms:
    for sl in slope_terms:
        for m in moisture_terms:
            avg = (soil_weight[s] + slope_weight[sl] + moisture_weight[m]) / 3.0
            if avg < 0.35: ct = 'low'
            elif avg < 0.65: ct = 'medium'
            elif avg < 0.85: ct = 'high'
            else: ct = 'extreme'
            rule = ctrl.Rule(soil[s] & slope[sl] & moisture[m], cost[ct])
            rules.append(rule)

cost_ctrl = ctrl.ControlSystem(rules)
fuzzy_sim = ctrl.ControlSystemSimulation(cost_ctrl)

def fuzzy_cost(soil_val, slope_val, moisture_val):
    try:
        fuzzy_sim.input['soil'] = np.clip(soil_val, 0, 1)
        fuzzy_sim.input['slope'] = np.clip(slope_val, -30, 30)
        fuzzy_sim.input['moisture'] = np.clip(moisture_val, 0, 1)
        fuzzy_sim.compute()
        return fuzzy_sim.output['cost']
    except:
        return 0.5  # fallback

# ============================================================
# 2. СРЕДА С НЕОДНОРОДНОЙ КАРТОЙ (сложный рельеф)
# ============================================================
class TerrainGrid:
    def __init__(self, width=15, height=15):
        self.width = width
        self.height = height
        np.random.seed(42)
        # Создадим "плохие" зоны: например, болото в центре, крутые склоны справа
        self.soil_map = np.ones((height, width)) * 0.2  # в основном твёрдый
        self.slope_map = np.zeros((height, width))
        self.moisture_map = np.zeros((height, width))
        
        # Зона болота (влажный рыхлый грунт) – прямоугольник
        for i in range(5, 10):
            for j in range(5, 10):
                self.soil_map[i, j] = 0.8
                self.moisture_map[i, j] = 0.7
        # Крутой подъём на правой стороне
        for i in range(height):
            for j in range(12, width):
                self.slope_map[i, j] = 20
        # Хорошая дорога внизу
        for i in range(height-3, height):
            for j in range(width):
                self.soil_map[i, j] = 0.1
                self.slope_map[i, j] = 0
                self.moisture_map[i, j] = 0.1
        
        self.start = (0, 0)
        self.goal = (height-1, width-1)
    
    def get_cell_params(self, x, y):
        return (self.soil_map[x, y], self.slope_map[x, y], self.moisture_map[x, y])

class RobotSimulator:
    def __init__(self, terrain):
        self.terrain = terrain
    
    def step(self, from_cell, to_cell):
        """Возвращает реальную энергию (ток) и пробуксовку при переходе в to_cell."""
        soil_val, slope_val, moisture_val = self.terrain.get_cell_params(to_cell[0], to_cell[1])
        base = fuzzy_cost(soil_val, slope_val, moisture_val)
        # Добавляем шум и нелинейность: энергия зависит от квадрата сложности
        real_energy = base ** 1.5 + np.random.normal(0, 0.05)
        real_energy = np.clip(real_energy, 0.1, 1.0)
        slip = base * (1 + max(0, slope_val)/30) * (1 + soil_val)
        slip = np.clip(slip, 0, 0.9)
        stuck = (real_energy > 0.9 and slip > 0.6)
        return real_energy, slip, stuck

# ============================================================
# 3. ПЛАНИРОВЩИК A*
# ============================================================
def a_star_path(start, goal, cost_map, width, height):
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    neighbors = [(0,1),(1,0),(0,-1),(-1,0)]
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        for dx, dy in neighbors:
            nx, ny = current[0]+dx, current[1]+dy
            if 0 <= nx < height and 0 <= ny < width:
                tentative_g = g_score[current] + cost_map[nx][ny]
                if (nx,ny) not in g_score or tentative_g < g_score[(nx,ny)]:
                    came_from[(nx,ny)] = current
                    g_score[(nx,ny)] = tentative_g
                    f_score[(nx,ny)] = tentative_g + heuristic((nx,ny), goal)
                    heapq.heappush(open_set, (f_score[(nx,ny)], (nx,ny)))
    return None

# ============================================================
# 4. АГЕНТ Q-LEARNING
# ============================================================
class RLCorrector:
    def __init__(self, actions=[-0.3, -0.15, 0, 0.15, 0.3], lr=0.3, gamma=0.9, epsilon=0.3):
        self.actions = actions
        self.lr = lr
        self.gamma = gamma
        self.epsilon = epsilon
        self.q_table = defaultdict(lambda: {a: 0.0 for a in actions})
    
    def discretize(self, soil_val, slope_val, moisture_val):
        s_bin = 0 if soil_val < 0.3 else (1 if soil_val < 0.7 else 2)
        if slope_val < -15: slope_bin = 0
        elif slope_val < -5: slope_bin = 1
        elif slope_val < 5: slope_bin = 2
        elif slope_val < 15: slope_bin = 3
        else: slope_bin = 4
        m_bin = 0 if moisture_val < 0.3 else (1 if moisture_val < 0.7 else 2)
        return (s_bin, slope_bin, m_bin)
    
    def get_action(self, state):
        if random.random() < self.epsilon:
            return random.choice(self.actions)
        else:
            return max(self.q_table[state], key=self.q_table[state].get)
    
    def update(self, state, action, reward, next_state):
        best_next = max(self.q_table[next_state].values())
        old_q = self.q_table[state][action]
        self.q_table[state][action] = old_q + self.lr * (reward + self.gamma * best_next - old_q)
    
    def correct_cost(self, fuzzy_base, state):
        if state in self.q_table:
            action = max(self.q_table[state], key=self.q_table[state].get)
        else:
            action = 0.0
        return np.clip(fuzzy_base + action, 0.0, 1.0), action

# ============================================================
# 5. ОСНОВНОЙ ЦИКЛ ОБУЧЕНИЯ (с записью метрик)
# ============================================================
def run_episode(terrain, robot, learner, ep_num, total_episodes):
    # Строим карту стоимостей с учётом RL
    cost_map = np.zeros((terrain.height, terrain.width))
    state_map = {}
    for i in range(terrain.height):
        for j in range(terrain.width):
            s_val, sl_val, m_val = terrain.get_cell_params(i, j)
            base = fuzzy_cost(s_val, sl_val, m_val)
            state = learner.discretize(s_val, sl_val, m_val)
            state_map[(i,j)] = state
            corrected, _ = learner.correct_cost(base, state)
            cost_map[i, j] = corrected
    
    # Планируем путь
    path = a_star_path(terrain.start, terrain.goal, cost_map, terrain.width, terrain.height)
    if path is None:
        return 100.0, 0, True, []  # нет пути, большой штраф
    
    total_energy = 0.0
    total_slip = 0.0
    stuck = False
    # Проходим по пути, обновляем Q-таблицу для каждой клетки
    for idx in range(1, len(path)):
        cell = path[idx]
        s_val, sl_val, m_val = terrain.get_cell_params(cell[0], cell[1])
        base = fuzzy_cost(s_val, sl_val, m_val)
        state = learner.discretize(s_val, sl_val, m_val)
        corrected, action = learner.correct_cost(base, state)
        # Симуляция прохода
        real_energy, slip, stuck_here = robot.step(path[idx-1], cell)
        total_energy += real_energy
        total_slip += slip
        if stuck_here:
            stuck = True
            break
        # Награда за шаг: отрицательная, пропорциональная энергии и пробуксовке
        reward = - (0.6 * real_energy + 0.4 * slip)
        # Следующее состояние (если есть)
        if idx+1 < len(path):
            next_cell = path[idx+1]
            ns_val, nsl_val, nm_val = terrain.get_cell_params(next_cell[0], next_cell[1])
            next_state = learner.discretize(ns_val, nsl_val, nm_val)
        else:
            next_state = state
        learner.update(state, action, reward, next_state)
    
    # Дополнительная награда за достижение цели
    if not stuck and path[-1] == terrain.goal:
        reward_goal = 5.0
        # Применим награду и к последнему состоянию
        last_cell = path[-1]
        s_val, sl_val, m_val = terrain.get_cell_params(last_cell[0], last_cell[1])
        last_state = learner.discretize(s_val, sl_val, m_val)
        # Небольшой бонус
        learner.update(last_state, action, reward_goal, last_state)
        total_energy -= 2.0  # фиктивное снижение энергии для подсчёта (несущественно)
    
    return total_energy, total_slip, stuck, path

def train(num_episodes=150):
    terrain = TerrainGrid(width=15, height=15)
    robot = RobotSimulator(terrain)
    learner = RLCorrector(epsilon=0.4)
    
    energies = []
    slips = []
    stuck_flags = []
    path_lengths = []
    
    for ep in range(num_episodes):
        # Уменьшаем epsilon со временем
        learner.epsilon = max(0.05, 0.4 * (1 - ep/num_episodes))
        energy, slip, stuck, path = run_episode(terrain, robot, learner, ep, num_episodes)
        energies.append(energy)
        slips.append(slip)
        stuck_flags.append(1 if stuck else 0)
        path_lengths.append(len(path) if path else 0)
        
        if (ep+1) % 15 == 0:
            avg_energy = np.mean(energies[-15:])
            print(f"Эпизод {ep+1}: ср.энергия={avg_energy:.2f}, застреваний={stuck_flags[-15:].count(1)}/15, epsilon={learner.epsilon:.2f}")
    
    # Визуализация результатов
    plt.figure(figsize=(15,5))
    plt.subplot(1,3,1)
    plt.plot(energies)
    plt.title('Суммарная энергия за эпизод')
    plt.xlabel('Эпизод')
    plt.ylabel('Энергия')
    plt.grid(True)
    
    plt.subplot(1,3,2)
    # Скользящее среднее застреваний
    stuck_rates = np.convolve(stuck_flags, np.ones(10)/10, mode='valid')
    plt.plot(stuck_rates)
    plt.title('Доля застреваний (окно 10)')
    plt.xlabel('Эпизод')
    plt.ylabel('Доля')
    plt.grid(True)
    
    plt.subplot(1,3,3)
    plt.plot(path_lengths)
    plt.title('Длина пути (число шагов)')
    plt.xlabel('Эпизод')
    plt.ylabel('Шаги')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('learning_curves.png')
    plt.show()
    
    # Дополнительно: показать тепловую карту финальной стоимости
    final_cost_map = np.zeros((terrain.height, terrain.width))
    for i in range(terrain.height):
        for j in range(terrain.width):
            s_val, sl_val, m_val = terrain.get_cell_params(i, j)
            base = fuzzy_cost(s_val, sl_val, m_val)
            state = learner.discretize(s_val, sl_val, m_val)
            final_cost_map[i, j], _ = learner.correct_cost(base, state)
    
    plt.figure(figsize=(8,8))
    plt.imshow(final_cost_map, cmap='hot', interpolation='nearest')
    plt.colorbar(label='Стоимость прохода')
    plt.title('Финальная карта стоимостей (с обучением)')
    plt.savefig('final_costmap.png')
    plt.show()
    return learner

if __name__ == "__main__":
    train(150)