import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from collections import defaultdict
import random

# === 1. Нечёткая модель ===
soil = ctrl.Antecedent(np.arange(0, 1.05, 0.05), 'soil')
slope = ctrl.Antecedent(np.arange(-30, 31, 1), 'slope')
moisture = ctrl.Antecedent(np.arange(0, 1.05, 0.05), 'moisture')
cost = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'cost')

# Функции принадлежности
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

# === Генерация всех 36 правил ===
soil_terms = ['solid', 'loose', 'muddy']
slope_terms = ['negative_steep', 'flat', 'gentle_up', 'steep_up']
moisture_terms = ['dry', 'damp', 'wet']

# Эвристические веса
soil_weight = {'solid': 0.2, 'loose': 0.6, 'muddy': 0.9}
slope_weight = {'negative_steep': 0.1, 'flat': 0.3, 'gentle_up': 0.6, 'steep_up': 0.9}
moisture_weight = {'dry': 0.2, 'damp': 0.5, 'wet': 0.8}

rules = []
for s in soil_terms:
    for sl in slope_terms:
        for m in moisture_terms:
            # усреднённая стоимость
            avg = (soil_weight[s] + slope_weight[sl] + moisture_weight[m]) / 3.0
            if avg < 0.35:
                cost_term = 'low'
            elif avg < 0.65:
                cost_term = 'medium'
            elif avg < 0.85:
                cost_term = 'high'
            else:
                cost_term = 'extreme'
            rule = ctrl.Rule(soil[s] & slope[sl] & moisture[m], cost[cost_term])
            rules.append(rule)

cost_ctrl = ctrl.ControlSystem(rules)
fuzzy_sim = ctrl.ControlSystemSimulation(cost_ctrl)

# === 2. Агент Q-learning ===
class RLCorrector:
    def __init__(self, actions=[-0.2, -0.1, 0, 0.1, 0.2], lr=0.3, gamma=0.95, epsilon=0.2):
        self.actions = actions
        self.lr = lr
        self.gamma = gamma
        self.epsilon = epsilon
        self.q_table = defaultdict(lambda: {a: 0.0 for a in actions})

    def discretize(self, soil_val, slope_val, moisture_val):
        # soil_bin (0..2)
        s_bin = 0 if soil_val < 0.3 else (1 if soil_val < 0.7 else 2)
        # slope_bin (0..3)
        if slope_val < -15: slope_bin = 0
        elif slope_val < -5: slope_bin = 1
        elif slope_val < 5: slope_bin = 2
        elif slope_val < 15: slope_bin = 3
        else: slope_bin = 4  # 4 зоны, хотя у нас 4 терма, но для безопасности
        # moisture_bin (0..2)
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
        result = np.clip(fuzzy_base + action, 0.0, 1.0)
        return result, action

# === Пример использования ===
learner = RLCorrector()
soil_val = 0.7
slope_val = 12
moisture_val = 0.4
state = learner.discretize(soil_val, slope_val, moisture_val)

fuzzy_sim.input['soil'] = soil_val
fuzzy_sim.input['slope'] = slope_val
fuzzy_sim.input['moisture'] = moisture_val
fuzzy_sim.compute()

print("Output dictionary:", fuzzy_sim.output)  # Теперь должен быть ключ 'cost'
fuzzy_cost = fuzzy_sim.output['cost']
final_cost, used_action = learner.correct_cost(fuzzy_cost, state)

print(f"Базовая стоимость: {fuzzy_cost:.3f}")
print(f"Итоговая стоимость: {final_cost:.3f}")
print(f"Выбранное действие: {used_action}")