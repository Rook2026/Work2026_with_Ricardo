import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import time
from typing import List, Tuple

# I. МАТЕМАТИЧЕСКАЯ МОДЕЛЬ ПРОЕКЦИИ 3D-ОБЪЕКТА НА ПЛОСКОСТЬ

class Box3D:
    _SIGNS = np.array([
        [-1,-1,-1],[-1,-1, 1],[-1, 1,-1],[-1, 1, 1],
        [ 1,-1,-1],[ 1,-1, 1],[ 1, 1,-1],[ 1, 1, 1],
    ], dtype=np.float64)

    def __init__(self, width=4.0, height=3.0, depth=2.0):
        self.half   = np.array([width/2, height/2, depth/2])
        self._local = self._SIGNS * self.half   # 8×3, константа

    # I.2 Матрица вращения ZYX 
    @staticmethod
    def rotation_matrix(a, b, g) -> np.ndarray:
        """R = Rz(γ)·Ry(β)·Rx(α)"""
        ca,sa = np.cos(a),np.sin(a)
        cb,sb = np.cos(b),np.sin(b)
        cg,sg = np.cos(g),np.sin(g)
        Rx = np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]])
        Ry = np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]])
        Rz = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        return Rz @ Ry @ Rx

    # I.1 Вершины в глобальной СК 
    def get_vertices(self, state: np.ndarray) -> np.ndarray:
        """Vglobal = R·Vlocal + T"""
        R = self.rotation_matrix(state[3], state[4], state[5])
        return (R @ self._local.T).T + state[:3]

    # I.3 Перспективная проекция + заполненная растеризация ──
    def render(self, state: np.ndarray,
               W=640, H=480, f=500) -> np.ndarray:
       
        img   = np.zeros((H, W), dtype=np.uint8)
        verts = self.get_vertices(state)
        cx, cy = W / 2.0, H / 2.0

        pts2d = []
        for v in verts:
            if v[2] > 0.1:
                pts2d.append([f * v[0] / v[2] + cx,
                               f * v[1] / v[2] + cy])
        if len(pts2d) < 3:
            return img

        pts_np = np.array(pts2d, dtype=np.float32)
        hull   = cv2.convexHull(pts_np).astype(np.int32)
        cv2.fillConvexPoly(img, hull, 255)
        return img

    def project_pts(self, state, W=640, H=480, f=500) -> np.ndarray:
        verts = self.get_vertices(state)
        cx, cy = W/2.0, H/2.0
        pts = []
        for v in verts:
            if v[2] > 0.1:
                pts.append([f*v[0]/v[2]+cx, f*v[1]/v[2]+cy])
        return np.array(pts) if pts else np.empty((0,2))



# III. ФУНКЦИЯ ПРИСПОСОБЛЕННОСТИ
class FitnessEvaluator:

    def __init__(self, target: np.ndarray, box: Box3D, wJ=0.6, wD=0.4):
        self.tgt     = (target > 0)
        self.box     = box
        self.wJ, self.wD = wJ, wD
        self.H, self.W = target.shape

    # III.2 Метрики сходства ──
    def _jaccard(self, s) -> float:
        """J(A,B) = |A∩B| / |A∪B|"""
        inter = np.logical_and(self.tgt, s).sum()
        union = np.logical_or (self.tgt, s).sum()
        return inter/union if union > 0 else 0.0

    def _dice(self, s) -> float:
        """D(A,B) = 2|A∩B| / (|A|+|B|)"""
        inter = np.logical_and(self.tgt, s).sum()
        total = self.tgt.sum() + s.sum()
        return 2.0*inter/total if total > 0 else 0.0

    # III.3 Штрафные функции ──
    def _pen_z(self, z: float) -> float:
        return 0.01 * abs(z - float(np.clip(z, 5.0, 50.0)))

    def _pen_bounds(self, state: np.ndarray) -> float:
        pts = self.box.project_pts(state, self.W, self.H)
        if len(pts) == 0:
            return 0.5
        on = ((pts[:,0]<=0)|(pts[:,0]>=self.W-1)|
              (pts[:,1]<=0)|(pts[:,1]>=self.H-1)).sum()
        return 0.05 * on / len(pts)

    # Итоговая оценка
    def evaluate(self, state: np.ndarray) -> float:
        synth = self.box.render(state, self.W, self.H) > 0
        J = self._jaccard(synth)
        D = self._dice(synth)
        return (self.wJ*J + self.wD*D
                - self._pen_z(state[2])
                - self._pen_bounds(state))

    def batch(self, pop: np.ndarray) -> np.ndarray:
        return np.array([self.evaluate(ind) for ind in pop])

# II. ГЕНЕТИЧЕСКИЙ АЛГОРИТМ

# I.4 Пространство поиска (Таблица)
BOUNDS = np.array([
    [-10., 10.], [-10., 10.], [5., 50.],
    [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi],
])
LO, HI = BOUNDS[:,0], BOUNDS[:,1]
RNG = HI - LO

def _clip(x): return np.clip(x, LO, HI)


def init_pop(n: int, center=None, spread=1.0) -> np.ndarray:
    pop = np.random.uniform(LO, HI, (n, 6))
    if center is not None:
        n2    = n // 2
        noise = np.random.randn(n2, 6) * spread * RNG * 0.1
        pop[:n2] = _clip(np.tile(center, (n2,1)) + noise)
    return pop


# II.3 Турнирная селекция (k=3)
def tournament(pop: np.ndarray, fit: np.ndarray, k=3) -> np.ndarray:
    n   = len(pop)
    idx = np.random.randint(0, n, (n, k))
    best= idx[np.arange(n), np.argmax(fit[idx], axis=1)]
    return pop[best]


# II.4 BLX-α кроссовер (α=0.5, pc=0.85) 
def crossover(parents: np.ndarray, pc=0.85, alpha=0.5):
    n  = len(parents)
    p2 = parents[np.random.permutation(n)]
    d  = p2 - parents
    a1 = np.random.uniform(-alpha, 1+alpha, (n,6))
    a2 = np.random.uniform(-alpha, 1+alpha, (n,6))
    c1 = _clip(parents + a1*d)
    c2 = _clip(p2      - a2*d)
    mask = (np.random.rand(n) < pc)[:,None]
    return np.where(mask, c1, parents), np.where(mask, c2, p2)


# II.5 Адаптивная гауссова мутация с затуханием 
def mutate(pop: np.ndarray, gen: int, pm=1/6, sigma0=0.15) -> np.ndarray:
    """σ(t) = σ0·exp(−t/200), плюс адаптация τ=1/√(2n)"""
    sigma = sigma0 * np.exp(-gen / 200.0)
    sigma *= np.exp(np.random.randn() / np.sqrt(12))
    sigma  = max(sigma, 1e-4)
    mask   = np.random.rand(*pop.shape) < pm
    return _clip(pop + mask * sigma * RNG * np.random.randn(*pop.shape))


# II.6 Основной цикл: (μ+λ)-ГА с элитарностью и перезапусками 
def run_ga(evaluator: FitnessEvaluator,
           pop_size=100, max_gen=300, patience=50,
           n_restarts=3, sigma0=0.15):
  
    g_best_fit = -np.inf
    g_best_ind = None
    g_history  = []

    for restart in range(n_restarts):
        spread = max(0.5, 2.0 - restart * 0.5)
        pop    = init_pop(pop_size, g_best_ind, spread)
        fit    = evaluator.batch(pop)

        loc_best = float(np.max(fit))
        loc_ind  = pop[np.argmax(fit)].copy()
        no_impr  = 0
        history  = [loc_best]

        for gen in range(max_gen):
            # Воспроизводство
            parents  = tournament(pop, fit, k=3)
            c1, c2   = crossover(parents, pc=0.85)
            children = np.vstack([
                mutate(c1, gen, sigma0=sigma0),
                mutate(c2, gen, sigma0=sigma0),
            ])
            # (μ+λ)-объединение
            combined     = np.vstack([pop, children])
            fit_combined = evaluator.batch(combined)
            top          = np.argsort(fit_combined)[-pop_size:]
            pop, fit     = combined[top], fit_combined[top]

            cur = float(np.max(fit))
            history.append(cur)

            if cur > loc_best + 1e-5:
                loc_best = cur
                loc_ind  = pop[np.argmax(fit)].copy()
                no_impr  = 0
            else:
                no_impr += 1

            if no_impr >= patience:
                print(f"    [перезапуск {restart+1}] "
                      f"поколение {gen:3d}  f={loc_best:.4f}")
                break

        g_history.extend(history)
        if loc_best > g_best_fit:
            g_best_fit = loc_best
            g_best_ind = loc_ind.copy()

        if g_best_fit >= 0.95:
            print(f"    Отличный результат f={g_best_fit:.4f}")
            break

    return g_best_ind, g_best_fit, g_history

# IV. ТЕСТОВОЕ ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ

def make_target(state, box, noise=0.0, occlusion=0.0, W=640, H=480):
    """Генерирует целевое изображение (Сценарии 1–3, раздел 6.1)."""
    img = box.render(state, W, H).astype(np.float32)

    # Сценарий 2: гауссов шум σ=20
    if noise > 0:
        img = np.clip(img + np.random.randn(H, W) * noise, 0, 255)

    # Сценарий 3: случайные прямоугольные окклюзии ~25%
    if occlusion > 0:
        for _ in range(np.random.randint(3, 7)):
            rw = np.random.randint(40, W//3)
            rh = np.random.randint(40, H//3)
            rx = np.random.randint(0, W-rw)
            ry = np.random.randint(0, H-rh)
            img[ry:ry+rh, rx:rx+rw] = 0

    return (img > 127).astype(np.uint8) * 255


def run_experiment(obj_id, true_state, scenario, box,
                   noise=0.0, occlusion=0.0) -> dict:
    print(f"\n{'='*60}")
    print(f"Объект {obj_id} | Сценарий: {scenario}")
    print(f"  Истинное состояние: {np.round(true_state, 3)}")
    print(f"{'='*60}")

    target    = make_target(true_state, box, noise, occlusion)
    evaluator = FitnessEvaluator(target, box)

    t0 = time.time()
    best, fit, hist = run_ga(
        evaluator,
        pop_size   = 100,   # P=100 (Таблица II.1)
        max_gen    = 300,   # Gmax
        patience   = 50,    # критерий ранней остановки
        n_restarts = 3,
        sigma0     = 0.15,
    )
    elapsed = time.time() - t0

    pos_err = np.linalg.norm(best[:3] - true_state[:3])
    # Угловая ошибка с учётом цикличности [-π, π]
    ang_err = np.mean(np.abs(
        np.arctan2(np.sin(best[3:]-true_state[3:]),
                   np.cos(best[3:]-true_state[3:]))))

    print(f"  Найденное состояние: {np.round(best, 3)}")
    print(f"  Приспособленность:   {fit:.4f}")
    print(f"  Ошибка положения:    {pos_err:.3f} ед.")
    print(f"  Ошибка ориентации:   {ang_err:.4f} рад ({np.degrees(ang_err):.2f}°)")
    print(f"  Поколений (всего):   {len(hist)}")
    print(f"  Время:               {elapsed:.1f} сек")

    return dict(obj_id=obj_id, scenario=scenario,
                true_state=true_state, best_state=best,
                fitness=fit, pos_err=pos_err, ang_err=ang_err,
                generations=len(hist), time=elapsed,
                history=hist, target=target)


def visualize(results: list, box: Box3D):
    scenarios = ["Идеальные условия", "Гауссов шум (σ=20)", "Окклюзия 25%"]
    colors    = ["#2196F3", "#FF9800", "#F44336"]

    # График сходимости
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle("Графики сходимости генетического алгоритма",
                 fontsize=14, fontweight="bold")
    for ax, scen, col in zip(axes, scenarios, colors):
        for r in [x for x in results if x["scenario"]==scen]:
            ax.plot(r["history"], color=col, alpha=0.85, linewidth=1.5,
                    label=f"Объект {r['obj_id']} (f={r['fitness']:.3f})")
        ax.set_title(scen, fontsize=11)
        ax.set_xlabel("Поколение"); ax.set_ylabel("Приспособленность")
        ax.set_ylim(0, 1.05); ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig("convergence.png", dpi=120, bbox_inches="tight")
    plt.close()
    print("\nСохранён: convergence.png")

    # Сравнение изображений (объект 1, все 3 сценария) 
    ex = [r for r in results if r["obj_id"]==1][:3]
    if len(ex) == 3:
        fig, axes = plt.subplots(3, 3, figsize=(12, 9))
        fig.suptitle("Целевое vs Синтезированное (Объект 1)",
                     fontsize=13, fontweight="bold")
        for row, r in enumerate(ex):
            synth = box.render(r["best_state"])
            diff  = np.abs(r["target"].astype(int) - synth.astype(int))
            for ci, (im, ttl) in enumerate([
                (r["target"], f"Целевое\n{r['scenario']}"),
                (synth,       f"Синтез f={r['fitness']:.3f}"),
                (diff,        "Разность"),
            ]):
                axes[row, ci].imshow(im, cmap="gray" if ci<2 else "hot")
                axes[row, ci].set_title(ttl, fontsize=8)
                axes[row, ci].axis("off")
        plt.tight_layout()
        plt.savefig("image_comparison.png", dpi=120, bbox_inches="tight")
        plt.close()
        print("Сохранён: image_comparison.png")

    # Сводная таблица результатов (Таблица 6.1)
    fig, ax = plt.subplots(figsize=(14, 4.5))
    ax.axis("off")
    hdrs = ["Объект", "Условия", "Приспособ-\nность",
            "Время\n(сек)", "Поколений",
            "Ошибка\nполож.", "Ошибка\nориент."]
    rows = []
    for r in results:
        rows.append([
            str(r["obj_id"]), r["scenario"],
            f"{r['fitness']:.4f}", f"{r['time']:.1f}",
            str(r["generations"]), f"{r['pos_err']:.3f}",
            f"{r['ang_err']:.4f} рад",
        ])
    tbl = ax.table(cellText=rows, colLabels=hdrs,
                   loc="center", cellLoc="center")
    tbl.auto_set_font_size(False); tbl.set_fontsize(9); tbl.scale(1, 1.9)
    for j in range(len(hdrs)):
        tbl[0,j].set_facecolor("#D9E1F2")
        tbl[0,j].set_text_props(fontweight="bold")
    for i in range(1, len(rows)+1):
        for j in range(len(hdrs)):
            tbl[i,j].set_facecolor("#F5F5F5" if i%2==0 else "white")
    ax.set_title("Таблица 6.1 — Результаты вычислительных экспериментов",
                 fontsize=11, fontweight="bold", pad=20)
    plt.tight_layout()
    plt.savefig("results_table.png", dpi=120, bbox_inches="tight")
    plt.close()
    print("Сохранена: results_table.png")



if __name__ == "__main__":
    box = Box3D(width=4.0, height=3.0, depth=2.0)

    # Три тестовых объекта (Таблица раздела 6.1)
    objects = [
        np.array([ 0.0,  0.0, 15.0,  0.2,  0.3,  0.5]),   # Объект 1
        np.array([-5.0,  2.0, 20.0,  0.5, -0.2,  1.0]),   # Объект 2
        np.array([ 3.0, -2.0, 12.0, -0.4,  0.6, -0.8]),   # Объект 3
    ]

    # Три сценария (раздел 6.1)
    scenarios = [
        dict(scenario="Идеальные условия",  noise=0.0,  occlusion=0.0 ),
        dict(scenario="Гауссов шум (σ=20)", noise=20.0, occlusion=0.0 ),
        dict(scenario="Окклюзия 25%",       noise=0.0,  occlusion=0.25),
    ]

    results = []
    for cfg in scenarios:
        for i, st in enumerate(objects, 1):
            results.append(run_experiment(i, st, box=box, **cfg))

    visualize(results, box)

    print("\n" + "="*60)
    print("Все эксперименты завершены.")
    print("="*60)