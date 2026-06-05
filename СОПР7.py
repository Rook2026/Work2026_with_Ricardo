"""
Практическое занятие №7.
Применение многослойной нейронной сети для аппроксимации нелинейных функций.
РТУ МИРЭА

Запуск:  python main_pr7.py
Результат: папка results/ с графиками для каждой конфигурации сети.
"""

import math
import os
import torch
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ─── Параметры ────────────────────────────────────────────────────────────────
EPOCHS      = 50000
LR          = 0.01
PRINT_EVERY = 5000
RESULTS_DIR = "results"
os.makedirs(RESULTS_DIR, exist_ok=True)

# Варианты нейронов в скрытом слое
HIDDEN_VARIANTS = [2, 3, 5, 10]

# Варианты числа обучающих примеров (дополнительное задание)
TRAIN_N_VARIANTS = [10, 100, 1000]


# ─── Генерация данных ─────────────────────────────────────────────────────────
def make_data(n: int = 1000):
    """
    Генерация обучающих и проверочных данных.
    Аппроксимируемая функция: y = sin(x + π/2) = cos(x), x ∈ [0, 6π]
    """
    x_train = torch.rand(n) * 6 * math.pi
    y_train = torch.tensor(np.sin(x_train.numpy() + math.pi / 2),
                           dtype=torch.float32)

    x_test = torch.rand(1000) * 6 * math.pi
    x_test, _ = x_test.sort()          # сортировка для корректной визуализации
    y_test = torch.tensor(np.sin(x_test.numpy() + math.pi / 2),
                          dtype=torch.float32)

    return (x_train.unsqueeze(1), y_train.unsqueeze(1),
            x_test.unsqueeze(1),  y_test.unsqueeze(1))


# ─── class Network (1 скрытый слой) ──────────────────────────────────────────
class Network(torch.nn.Module):
    """
    Трёхслойная нейронная сеть прямого распространения (МНСПР):
      вход (1) → скрытый слой (hidden_neurons, Sigmoid)
               → скрытый слой (hidden_neurons, Sigmoid)
               → выход (1, линейный)
    Соответствует структуре из методички (пп. 7–8).
    """

    def __init__(self, hidden_neurons: int):
        super(Network, self).__init__()
        # 1-й скрытый слой
        self.fc1  = torch.nn.Linear(1, hidden_neurons)
        self.act1 = torch.nn.Sigmoid()
        # 2-й скрытый слой
        self.fc2  = torch.nn.Linear(hidden_neurons, hidden_neurons)
        self.act2 = torch.nn.Sigmoid()
        # Выходной слой
        self.fc3  = torch.nn.Linear(hidden_neurons, 1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.act1(self.fc1(x))
        x = self.act2(self.fc2(x))
        return self.fc3(x)


# ─── class Network1Hidden (1 скрытый слой) ───────────────────────────────────
class Network1Hidden(torch.nn.Module):
    """
    Двухслойная МНСПР (1 скрытый слой):
      вход (1) → скрытый слой (hidden_neurons, Sigmoid) → выход (1)
    Используется для сравнения с Network (2 скрытых слоя).
    """

    def __init__(self, hidden_neurons: int):
        super(Network1Hidden, self).__init__()
        self.fc1  = torch.nn.Linear(1, hidden_neurons)
        self.act1 = torch.nn.Sigmoid()
        self.fc2  = torch.nn.Linear(hidden_neurons, 1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.fc2(self.act1(self.fc1(x)))


# ─── Функция потерь (СКО) ────────────────────────────────────────────────────
def loss(pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    """
    Функция потерь — среднеквадратичное отклонение (формула 7.5):
    E = (1/M) * Σ(y - y*)²
    """
    return ((pred - target) ** 2).mean()


# ─── Функция прогноза и визуализации ─────────────────────────────────────────
def predict(net: torch.nn.Module,
            x: torch.Tensor, y: torch.Tensor,
            title: str = "", save_path: str = None):
    """
    Прогноз сети на тестовой выборке с визуализацией:
    синие точки — эталон (Ground truth), красные — прогноз сети.
    """
    net.eval()
    with torch.no_grad():
        y_pred = net.forward(x)
    rmse = float((((y_pred - y) ** 2).mean()) ** 0.5)

    plt.clf()
    plt.figure(figsize=(8, 4))
    plt.plot(x.numpy(), y.numpy(),      'o', c='b',
             markersize=2, label='Ground truth')
    plt.plot(x.numpy(), y_pred.numpy(), 'o', c='r',
             markersize=2, label=f'Prediction (RMSE={rmse:.4f})')
    plt.legend(loc='upper left', fontsize=9)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title(title)
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=100)
    plt.close()
    net.train()
    return rmse


# ─── Цикл обучения ────────────────────────────────────────────────────────────
def train(net: torch.nn.Module,
          x_train: torch.Tensor, y_train: torch.Tensor,
          x_test:  torch.Tensor, y_test:  torch.Tensor,
          tag: str, epochs: int = EPOCHS) -> float:
    """
    Обучение МНСПР алгоритмом Adam (адаптивный градиентный спуск).
    Каждые PRINT_EVERY эпох сохраняется промежуточный график.
    Возвращает итоговый RMSE на тестовой выборке.
    """
    optimizer = torch.optim.Adam(net.parameters(), lr=LR)
    net.train()

    for i in range(epochs):
        optimizer.zero_grad()
        y_pred = net.forward(x_train)
        loss_val = loss(y_pred, y_train)
        loss_val.backward()
        optimizer.step()

        if i % PRINT_EVERY == 0:
            rmse = float((((net.forward(x_test).detach() - y_test) ** 2).mean()) ** 0.5)
            print(f"  [{tag}] epoch {i:5d}  loss={float(loss_val):.5f}  RMSE={rmse:.5f}")
            path = os.path.join(RESULTS_DIR, f"{tag}_epoch{i}.png")
            predict(net, x_test, y_test,
                    title=f"{tag} | epoch {i}",
                    save_path=path)

    # Финальный график
    path_final = os.path.join(RESULTS_DIR, f"{tag}_final.png")
    rmse_final = predict(net, x_test, y_test,
                         title=f"{tag} | epoch {epochs} (final)",
                         save_path=path_final)
    print(f"  [{tag}] FINAL RMSE={rmse_final:.5f}  → {path_final}")
    return rmse_final


# ─── Итоговая таблица результатов ────────────────────────────────────────────
def plot_summary(results: dict):
    """
    Сводный график зависимости RMSE от числа нейронов
    для 1- и 2-скрытослойных сетей.
    """
    fig, ax = plt.subplots(figsize=(7, 4))
    for label, data in results.items():
        ns = sorted(data.keys())
        ax.plot(ns, [data[n] for n in ns], marker='o', label=label)
    ax.set_xlabel("Нейронов в скрытом слое")
    ax.set_ylabel("RMSE (тест)")
    ax.set_title("Зависимость ошибки от числа нейронов")
    ax.legend()
    ax.grid(True, alpha=0.4)
    plt.tight_layout()
    path = os.path.join(RESULTS_DIR, "summary_rmse.png")
    plt.savefig(path, dpi=100)
    plt.close()
    print(f"\nСводный график сохранён: {path}")


# ─── Дополнительное задание: зависимость от N ─────────────────────────────────
def experiment_n_samples():
    """
    Исследование зависимости ошибки от числа обучающих примеров
    (n = 10, 100, 1000) при фиксированной архитектуре Network(10).
    """
    print("\n=== Дополнительное задание: влияние числа обучающих примеров ===")
    _, _, x_test, y_test = make_data(1000)
    results = {}
    for n in TRAIN_N_VARIANTS:
        x_tr, y_tr, _, _ = make_data(n)
        net = Network(10)
        tag = f"N{n}_h10_2layers"
        rmse = train(net, x_tr, y_tr, x_test, y_test, tag, epochs=20000)
        results[n] = rmse

    fig, ax = plt.subplots(figsize=(6, 3))
    ax.plot(list(results.keys()), list(results.values()), marker='s', color='green')
    ax.set_xlabel("Число обучающих примеров N")
    ax.set_ylabel("RMSE (тест)")
    ax.set_title("Зависимость ошибки от числа примеров (hidden=10)")
    ax.set_xscale('log')
    ax.grid(True, alpha=0.4)
    plt.tight_layout()
    path = os.path.join(RESULTS_DIR, "summary_n_samples.png")
    plt.savefig(path, dpi=100)
    plt.close()
    print(f"График N-примеров сохранён: {path}")
    return results


# ─── Main ─────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    x_train, y_train, x_test, y_test = make_data(1000)

    # Сохранить эталонный график (шаг 6 методички)
    plt.figure(figsize=(8, 3))
    plt.plot(x_test.numpy(), y_test.numpy(), linewidth=1)
    plt.title("Эталонный график: y = sin(x + π/2)")
    plt.xlabel("x"); plt.ylabel("y")
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, "ground_truth.png"), dpi=100)
    plt.close()
    print("Эталонный график сохранён.")

    results = {"1 скрытый слой": {}, "2 скрытых слоя": {}}

    for h in HIDDEN_VARIANTS:
        print(f"\n=== 1 скрытый слой, hidden={h} ===")
        net1 = Network1Hidden(h)
        tag1 = f"1hidden_h{h}"
        rmse1 = train(net1, x_train, y_train, x_test, y_test, tag1)
        results["1 скрытый слой"][h] = rmse1

        print(f"\n=== 2 скрытых слоя, hidden={h} ===")
        net2 = Network(h)
        tag2 = f"2hidden_h{h}"
        rmse2 = train(net2, x_train, y_train, x_test, y_test, tag2)
        results["2 скрытых слоя"][h] = rmse2

    plot_summary(results)

    # Дополнительное задание
    n_results = experiment_n_samples()

    # Печать итоговой таблицы
    print("\n" + "="*55)
    print(f"{'Архитектура':<20} {'hidden':>8} {'RMSE':>12}")
    print("="*55)
    for label, data in results.items():
        for h, rmse in sorted(data.items()):
            print(f"{label:<20} {h:>8} {rmse:>12.5f}")
    print("\nЗависимость от N:")
    for n, rmse in n_results.items():
        print(f"  N={n:<5}  RMSE={rmse:.5f}")