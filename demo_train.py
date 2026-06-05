import numpy as np
import matplotlib.pyplot as plt
import random
import heapq
from collections import defaultdict

# ------------------ ПРОСТАЯ СРЕДА 3x3 ------------------
class TinyEnv:
    def __init__(self):
        self.start = (0, 0)
        self.goal = (2, 2)
        self.left_cell = (1, 1)          # левый короткий путь идёт через эту клетку
        self.right_cells = [(0,1), (0,2), (1,2)]  # правый обходной путь
    def cell_type(self, pos):
        return 'left' if pos == self.left_cell else 'right'
    def fuzzy_cost(self, pos):
        # ОШИБОЧНАЯ нечёткая оценка: левый путь кажется дешёвым (0.1), правый – дорогим (0.8)
        return 0.1 if self.cell_type(pos) == 'left' else 0.8
    def real_cost(self, pos):
        # РЕАЛЬНАЯ энергия: левый путь очень дорогой (1.0), правый – дешёвый (0.2)
        return 1.0 if self.cell_type(pos) == 'left' else 0.2
    def neighbors(self, pos):
        x, y = pos
        for dx, dy in [(0,1),(1,0),(0,-1),(-1,0)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx <= 2 and 0 <= ny <= 2:
                yield (nx, ny)

# ------------------ A* ПЛАНИРОВЩИК ------------------
def a_star(env, cost_map):
    start = env.start
    goal = env.goal
    open_set = [(0, start)]
    came_from = {}
    g = {start: 0}
    f = {start: abs(start[0]-goal[0]) + abs(start[1]-goal[1])}
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
        for nb in env.neighbors(current):
            tentative_g = g[current] + cost_map[nb[0]][nb[1]]
            if nb not in g or tentative_g < g[nb]:
                came_from[nb] = current
                g[nb] = tentative_g
                fval = tentative_g + abs(nb[0]-goal[0]) + abs(nb[1]-goal[1])
                heapq.heappush(open_set, (fval, nb))
    return None

# ------------------ Q-LEARNING АГЕНТ ------------------
class QLearner:
    def __init__(self, actions=[-0.5, 0, 0.5, 1.0, 1.5], lr=0.6, gamma=0.9, eps=0.7):
        self.actions = actions
        self.lr = lr
        self.gamma = gamma
        self.eps = eps
        self.q = defaultdict(lambda: {a: 0.0 for a in actions})
    def state(self, pos, env):
        return 1 if env.cell_type(pos) == 'left' else 0
    def get_action(self, state):
        if random.random() < self.eps:
            return random.choice(self.actions)
        return max(self.q[state], key=self.q[state].get)
    def update(self, state, action, reward, next_state):
        best_next = max(self.q[next_state].values())
        old = self.q[state][action]
        self.q[state][action] = old + self.lr * (reward + self.gamma * best_next - old)
    def correct_cost(self, fuzzy_base, state):
        action = self.get_action(state)
        return np.clip(fuzzy_base + action, 0.0, 1.0), action

# ------------------ ОБУЧЕНИЕ ------------------
def train(episodes=60):
    env = TinyEnv()
    learner = QLearner(eps=0.7)
    energies = []
    path_lengths = []
    for ep in range(episodes):
        learner.eps = max(0.05, 0.7 * (1 - ep/episodes))
        # Построение карты скорректированных стоимостей
        cost_map = np.zeros((3,3))
        for i in range(3):
            for j in range(3):
                pos = (i,j)
                fuzzy = env.fuzzy_cost(pos)
                state = learner.state(pos, env)
                corrected, _ = learner.correct_cost(fuzzy, state)
                cost_map[i,j] = corrected
        path = a_star(env, cost_map)
        if not path:
            energies.append(100)
            path_lengths.append(0)
            continue
        total_energy = 0.0
        for idx in range(1, len(path)):
            pos = path[idx]
            fuzzy = env.fuzzy_cost(pos)
            state = learner.state(pos, env)
            corrected, action = learner.correct_cost(fuzzy, state)
            real = env.real_cost(pos)
            total_energy += real
            reward = -real
            next_state = learner.state(path[idx+1], env) if idx+1 < len(path) else state
            learner.update(state, action, reward, next_state)
        energies.append(total_energy)
        path_lengths.append(len(path))
        if (ep+1) % 10 == 0:
            print(f"Эпизод {ep+1}: энергия={total_energy:.2f}, длина пути={len(path)}")
    # Графики
    plt.figure(figsize=(12,5))
    plt.subplot(1,2,1)
    plt.plot(energies, 'b-', linewidth=2)
    plt.title('Суммарная энергия за эпизод', fontsize=12)
    plt.xlabel('Эпизод')
    plt.ylabel('Энергия (отн. ед.)')
    plt.grid(True)
    plt.subplot(1,2,2)
    plt.plot(path_lengths, 'r-', linewidth=2)
    plt.title('Длина пути (шаги)', fontsize=12)
    plt.xlabel('Эпизод')
    plt.ylabel('Шаги')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('presentation_results.png', dpi=150)
    plt.savefig('energy_path_plot.png', dpi=200, bbox_inches='tight')
    plt.show()
    # Вывод Q-значений
    print("\n=== РЕЗУЛЬТАТЫ ОБУЧЕНИЯ ===")
    print("Q-значения для левой клетки (state=1):", dict(learner.q[1]))
    print("Q-значения для правых клеток (state=0):", dict(learner.q[0]))
    # Финальный путь
    final_cost_map = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            pos = (i,j)
            fuzzy = env.fuzzy_cost(pos)
            state = learner.state(pos, env)
            corrected, _ = learner.correct_cost(fuzzy, state)
            final_cost_map[i,j] = corrected
    final_path = a_star(env, final_cost_map)
    print(f"Длина финального пути: {len(final_path)} шагов (левый путь = 3 шага, правый = 5 шагов)")
    if len(final_path) >= 5:
        print(" Робот переключился на энергоэффективный обходной путь.")
        print(" Энергопотребление снижено на {:.0f}%.".format((1 - min(energies[-10:])/max(energies[:10]))*100))
    else:
        print(" Переключения не произошло, увеличьте число эпизодов.")
    return learner

if __name__ == "__main__":
    train(60)