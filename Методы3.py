import numpy as np
import matplotlib.pyplot as plt

# Ваши данные (из Figure_1.png, извлечённые вручную)
x_data = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
y_data = np.array([1.0, 0.8, 0.6, 0.4, 0.2, 0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -1.0, -1.0])

# Функции обработки (из предыдущего ответа)
def moving_average(data, window_size=3):
    if window_size % 2 == 0:
        window_size += 1
    half = window_size // 2
    smoothed = np.copy(data)
    for i in range(half, len(data) - half):
        smoothed[i] = np.mean(data[i - half : i + half + 1])
    return smoothed

def exponential_average(data, alpha=0.3):
    smoothed = np.zeros_like(data)
    smoothed[0] = data[0]
    for i in range(1, len(data)):
        smoothed[i] = alpha * data[i] + (1 - alpha) * smoothed[i-1]
    return smoothed

def kalman_filter_1d(data, process_var=0.01, measurement_var=0.1):
    n = len(data)
    x_est = np.zeros(n)
    P = np.zeros(n)
    x_est[0] = data[0]
    P[0] = 1.0
    for k in range(1, n):
        x_pred = x_est[k-1]
        P_pred = P[k-1] + process_var
        K = P_pred / (P_pred + measurement_var)
        x_est[k] = x_pred + K * (data[k] - x_pred)
        P[k] = (1 - K) * P_pred
    return x_est

# Применение фильтрации
y_ma = moving_average(y_data, window_size=3)
y_ema = exponential_average(y_data, alpha=0.4)
y_kalman = kalman_filter_1d(y_data)

# Интерполяция между точками 5 и 6 (x=5, y=0.0 и x=6, y=-0.2)
x1, y1 = x_data[5], y_data[5]   # (5, 0.0)
x2, y2 = x_data[6], y_data[6]   # (6, -0.2)
x_interp = 5.3
t = (x_interp - x1) / (x2 - x1)
y_interp = y1 + t * (y2 - y1)

# Экстраполяция за точку 12 (x=12, y=-1.0) на основе точек 11 и 12
x_extrap = 13.5
x1e, y1e = x_data[10], y_data[10]  # (10, -1.0)
x2e, y2e = x_data[11], y_data[11]  # (11, -1.0)
slope = (y2e - y1e) / (x2e - x1e)  # = 0
y_extrap = y2e + slope * (x_extrap - x2e)  # = -1.0

# Визуализация
fig, axes = plt.subplots(2, 3, figsize=(14, 8))
fig.suptitle('Обработка сигнальных данных (из Figure_1.png)', fontsize=14)

# Исходный график
axes[0,0].plot(x_data, y_data, 'ro-', linewidth=2, markersize=6, label='Исходные данные')
axes[0,0].set_title('Исходный график (13 точек)')
axes[0,0].grid(True)
axes[0,0].legend()

# Скользящее среднее
axes[0,1].plot(x_data, y_data, 'ro-', alpha=0.5, label='Исходные')
axes[0,1].plot(x_data, y_ma, 'b-', linewidth=2, label='Скользящее среднее (окно=3)')
axes[0,1].set_title('Скользящее среднее')
axes[0,1].grid(True)
axes[0,1].legend()

# Экспоненциальное
axes[0,2].plot(x_data, y_data, 'ro-', alpha=0.5, label='Исходные')
axes[0,2].plot(x_data, y_ema, 'm-', linewidth=2, label='Эксп. сглаживание (α=0.4)')
axes[0,2].set_title('Экспоненциальное среднее')
axes[0,2].grid(True)
axes[0,2].legend()

# Фильтр Калмана
axes[1,0].plot(x_data, y_data, 'ro-', alpha=0.5, label='Исходные')
axes[1,0].plot(x_data, y_kalman, 'c-', linewidth=2, label='Фильтр Калмана')
axes[1,0].set_title('Фильтр Калмана')
axes[1,0].grid(True)
axes[1,0].legend()

# Интерполяция
axes[1,1].plot(x_data, y_data, 'ro-', label='Исходные')
axes[1,1].plot(x1, y1, 'gs', markersize=10, label='Опорная точка 1')
axes[1,1].plot(x2, y2, 'gs', markersize=10, label='Опорная точка 2')
axes[1,1].plot(x_interp, y_interp, 'b*', markersize=12, label=f'Интерполяция x={x_interp}')
axes[1,1].set_title(f'Линейная интерполяция: y={y_interp:.3f}')
axes[1,1].grid(True)
axes[1,1].legend()

# Экстраполяция
axes[1,2].plot(x_data, y_data, 'ro-', label='Исходные')
axes[1,2].plot([x1e, x2e], [y1e, y2e], 'gs', markersize=10, label='Опорные точки')
axes[1,2].plot(x_extrap, y_extrap, 'b*', markersize=12, label=f'Экстраполяция x={x_extrap}')
axes[1,2].set_title(f'Линейная экстраполяция: y={y_extrap:.3f}')
axes[1,2].grid(True)
axes[1,2].legend()

plt.tight_layout()
plt.show()

print("Результаты:")
print(f"Интерполяция в x={x_interp} → y={y_interp:.4f}")
print(f"Экстраполяция в x={x_extrap} → y={y_extrap:.4f}")