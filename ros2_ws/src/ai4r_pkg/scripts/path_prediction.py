import numpy as np
from collections import defaultdict

# ========== 工具函数 ==========

def polyfit_through_origin_scaled(x, y, degree: int, weights=None, l2=0.1):
    """
    强制过原点的多项式拟合（y = a1 x + a2 x^2 + ...），支持权重和岭正则。
    通过对 x 做缩放（x/xmax），减小数值不稳定；解完再按幂次还原系数。
    返回：np.poly1d（最高次在前，常数项为 0）
    """
    degree = max(1, int(degree))
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    xmax = np.max(np.abs(x)) or 1.0
    t = x / xmax
    T = np.vstack([t**k for k in range(1, degree+1)]).T  # [N, degree]

    if weights is not None:
        w = np.asarray(weights, float).reshape(-1, 1)
        Tw = T * w
        yw = y * w.ravel()
    else:
        Tw, yw = T, y

    I = np.eye(degree)
    b = np.linalg.solve(Tw.T @ Tw + l2 * I, Tw.T @ yw)  # 解出缩放域系数
    # 按幂次还原到原始 x 域：a_k = b_k / xmax^k
    a = np.array([b[k-1] / (xmax**k) for k in range(1, degree+1)], float)
    # poly1d 需要最高次在前，所以先反转，再拼一个 0（强制过原点）
    return np.poly1d(np.concatenate([a[::-1], [0.0]]))

def cubic_bridge_samples(x0, y0, dy0, n=40):
    """
    生成一段三次桥接曲线，使曲线从 (0,0) 平滑连接到 (x0, y0) 且在 x0 处导数为 dy0。
    形式：y = a x^3 + b x^2（满足 y(0)=0, y'(0)=0, y(x0)=y0, y'(x0)=dy0）
    返回：桥段采样 xs, ys
    """
    x0 = float(x0)
    if x0 <= 0:
        return np.array([0.0]), np.array([0.0])
    A = np.array([[x0**3, x0**2],
                  [3*x0**2, 2*x0]], float)
    b = np.array([y0, dy0], float)
    a, b2 = np.linalg.solve(A, b)
    xs = np.linspace(0, x0, n)
    ys = a*xs**3 + b2*xs**2
    return xs, ys

def normal_vertical_offset_arr(p, x, d, max_abs=None):
    """
    沿曲线法线方向偏移 d 的等效“竖直位移” Δy。
    几何关系：Δy = d / sqrt(1 + (p'(x))^2)
    若提供 max_abs，则对 Δy 取绝对值上限裁剪。
    """
    slope = np.polyder(p)(x)
    dv = d / np.sqrt(1.0 + slope**2)
    if max_abs is not None:
        dv = np.clip(dv, -abs(max_abs), abs(max_abs))
    return dv

# ========== 数据清理与锚点 ==========

def clean_single_far_point_lists(x_list, y_list, c_list,
                                 yellow_code=0, blue_code=1,
                                 half_rule='midpoint', include_equal=False):
    """
    删除“另一边只有一个点且该点落在对侧后半段”的孤立点。
    规则：
      - 若黄色仅 1 点，且蓝色 >=1，则计算蓝色的“半段阈值”（中点/中位数），
        若该黄点的 x 在阈值右侧（或含等号）则删除该黄点。
      - 蓝色同理。
    返回：清理后的列表
    """
    if not x_list:
        return x_list, y_list, c_list

    x = np.asarray(x_list, float)
    c = np.asarray(c_list, int)

    idx_y = np.where(c == yellow_code)[0]
    idx_b = np.where(c == blue_code)[0]

    def half_threshold(xs):
        if xs.size == 0:
            return None
        if half_rule == 'median':
            return float(np.median(xs))
        return float((xs.min() + xs.max()) / 2.0)

    to_remove = []
    if idx_y.size == 1 and idx_b.size >= 1:
        thr = half_threshold(x[idx_b])
        if thr is not None and (x[idx_y[0]] > thr if not include_equal else x[idx_y[0]] >= thr):
            to_remove.append(idx_y[0])
    if idx_b.size == 1 and idx_y.size >= 1:
        thr = half_threshold(x[idx_y])
        if thr is not None and (x[idx_b[0]] > thr if not include_equal else x[idx_b[0]] >= thr):
            to_remove.append(idx_b[0])

    if to_remove:
        keep = np.ones(len(x), bool)
        keep[to_remove] = False
        # 注意：y_list 需要同步裁剪
        y_arr = np.asarray(y_list, float)
        return x[keep].tolist(), y_arr[keep].tolist(), c[keep].tolist()
    return x_list, y_list, c_list

def add_conditional_anchor_cones_lists(x_list, y_list, c_list,
                                       offset=500.0,
                                       x_positions=(0.0, 250.0),
                                       origin_y=0.0,
                                       yellow_code=0, blue_code=1,
                                       dedup_tol=1e-6):
    """
    若某颜色本帧存在，则在 x_positions 处为该颜色添加锚点：
      - yellow: (x0, origin_y + offset)
      - blue  : (x0, origin_y - offset)
    同时避免与已有点重复（按 dedup_tol 判重）。
    返回：添加后的列表
    """
    cs = np.asarray(c_list, int) if c_list else np.array([])
    has_yellow = np.any(cs == yellow_code) if cs.size else False
    has_blue   = np.any(cs == blue_code)   if cs.size else False

    def exists(x0, y0, c0):
        for xx, yy, cc in zip(x_list, y_list, c_list):
            if cc == c0 and abs(xx - x0) <= dedup_tol and abs(yy - y0) <= dedup_tol:
                return True
        return False

    if has_yellow:
        for x0 in x_positions:
            y0 = origin_y + offset
            if not exists(x0, y0, yellow_code):
                x_list.append(float(x0)); y_list.append(float(y0)); c_list.append(int(yellow_code))
    if has_blue:
        for x0 in x_positions:
            y0 = origin_y - offset
            if not exists(x0, y0, blue_code):
                x_list.append(float(x0)); y_list.append(float(y0)); c_list.append(int(blue_code))
    return x_list, y_list, c_list

# ========== 核心拟合 ==========

def generate_middle_line(x_list, y_list, c_list,
                         degree=4, default_offset=500,
                         use_normal_offset=False,
                         ridge=None, tail_boost=1.0,
                         add_start_anchors=True,
                         anchor_positions=(0.0, 250.0),
                         do_clean_single_far=True,
                         max_normal_offset=None):
    """
    根据左右两侧锥桶点（颜色分组）拟合边界并生成“中线”的多项式函数。
    输入: x_list, y_list, c_list（颜色编码：默认黄色=0，蓝色=1）
    输出: poly_middle (numpy.poly1d)
    """
    x_list = list(map(float, x_list))
    y_list = list(map(float, y_list))
    c_list = list(map(int,   c_list))

    # 1) 清理孤立点
    if do_clean_single_far:
        x_list, y_list, c_list = clean_single_far_point_lists(x_list, y_list, c_list)

    # 2) 条件锚点（仅当某色存在时才加）
    if add_start_anchors:
        x_list, y_list, c_list = add_conditional_anchor_cones_lists(
            x_list, y_list, c_list, offset=default_offset, x_positions=anchor_positions)

    # 3) 按颜色分组并排序
    groups = defaultdict(list)
    for x, y, c in zip(x_list, y_list, c_list):
        groups[c].append((x, y))

    group_info = []
    for color, pts in groups.items():
        pts_sorted = sorted(pts, key=lambda p: p[0])
        xs = np.array([p[0] for p in pts_sorted], float)
        ys = np.array([p[1] for p in pts_sorted], float)
        group_info.append((color, len(pts_sorted), xs, ys))

    valid = [g for g in group_info if g[1] >= 2]  # 至少 2 点可拟合
    singles = [g for g in group_info if g[1] == 1]

    # 4) 构造统一的拟合采样网格
    x_max = float(np.max(x_list)) if len(x_list) else 1.0
    x_fit = np.linspace(0.0, max(x_max, 1.0), 400)

    # 5) 自动估计 ridge（防止过拟合/病态）
    if ridge is None:
        x_rng = max(1.0, x_max - 0.0)
        ridge = float(min(max(1e-4 * degree * (x_rng / 1000.0), 1e-6), 5e-3))

    # ---- 辅助：边界拟合（普通多项式） ----
    def _fit_edge(xs, ys):
        deg = int(max(1, min(degree, len(xs) - 1)))
        coefs = np.polyfit(xs, ys, deg)
        return np.poly1d(coefs)

    # ---- 辅助：在起点构建桥接段，增强拟合稳定性 ----
    def _build_bridge(x_fit, y_target, x_start):
        idx0 = np.searchsorted(x_fit, x_start)
        y0 = float(y_target[idx0])
        if idx0 + 1 < len(x_fit):
            dy0 = (y_target[idx0+1] - y_target[idx0]) / (x_fit[idx0+1] - x_fit[idx0])
        else:
            dy0 = 0.0
        xb, yb = cubic_bridge_samples(x_start, y0, dy0, n=40)
        X = np.concatenate([xb, x_fit[idx0:]])
        Y = np.concatenate([yb, y_target[idx0:]])
        # 前段桥接给较低权重，越往后权重线性增加（抑制起点抖动）
        W = np.concatenate([np.full_like(xb, 1.0, float),
                            np.linspace(1.0, tail_boost, len(x_fit)-idx0)])
        return X, Y, W

    # === 情况 A：两条边均可拟合 ===
    if len(valid) >= 2:
        (c1, _, x1, y1), (c2, _, x2, y2) = sorted(valid, key=lambda g: np.min(g[2]))
        p1 = _fit_edge(x1, y1)
        p2 = _fit_edge(x2, y2)

        if use_normal_offset:
            y_t1 = p1(x_fit) + (-1 if c1==0 else +1) * normal_vertical_offset_arr(p1, x_fit, default_offset, max_normal_offset)
            y_t2 = p2(x_fit) + (-1 if c2==0 else +1) * normal_vertical_offset_arr(p2, x_fit, default_offset, max_normal_offset)
        else:
            # 直接上下平移（不考虑法线），简单稳健
            off1 = (+default_offset if c1==1 else -default_offset)
            off2 = (+default_offset if c2==1 else -default_offset)
            y_t1 = p1(x_fit) + off1
            y_t2 = p2(x_fit) + off2

        # 基于“离该色点集的最近距离”构造支持度权重，平滑融合两侧信息
        def support_weight(xq, xs, h):
            dmin = np.min(np.abs(xq[:, None] - xs[None, :]), axis=1)
            return np.exp(-(dmin**2) / (2 * h * h))
        bandwidth = max(300.0, 0.1 * (x_fit.max() - x_fit.min()))
        w1 = support_weight(x_fit, x1, bandwidth)
        w2 = support_weight(x_fit, x2, bandwidth)
        s = w1 + w2 + 1e-8
        y_target = (w1/s) * y_t1 + (w2/s) * y_t2

        x_start = min(np.min(x1), np.min(x2))
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        return polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)

    # === 情况 B：一条边 + 对侧单点 ===
    if len(valid) == 1 and len(singles) == 1:
        color, _, xs, ys = valid[0]
        p = _fit_edge(xs, ys)
        _, _, x_s, y_s = singles[0]
        x_sv, y_sv = float(x_s[0]), float(y_s[0])
        y_at_sv = float(p(x_sv))
        # 用“边界在单点处的残差的一半”作为整体平移，估一个中线偏置
        offset_point = (y_sv - y_at_sv) / 2.0
        y_target = p(x_fit) + offset_point
        x_start = float(np.min(xs))
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        return polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)

    # === 情况 C：单边 ===
    if len(valid) == 1:
        color, _, xs, ys = valid[0]
        p = _fit_edge(xs, ys)
        if use_normal_offset:
            sign = -1 if color == 0 else +1  # yellow(0)在上 -> 中线往下；blue(1)在下 -> 中线往上
            y_target = p(x_fit) + sign * normal_vertical_offset_arr(p, x_fit, default_offset, max_normal_offset)
        else:
            # 不用法线，直接相对这条边平移 default_offset 得到“中线”
            offset = default_offset if color == 0 else -default_offset
            y_target = p(x_fit) - offset
        x_start = float(np.min(xs))
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        return polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)

    # === 情况 D：两侧都是单点 ===
    if len(singles) == 2:
        (_, _, x1, y1), (_, _, x2, y2) = singles
        x1v, y1v = float(x1[0]), float(y1[0])
        x2v, y2v = float(x2[0]), float(y2[0])
        x_start = min(x1v, x2v)
        # 直接取两点 y 的平均，作为常值“中线”目标
        y_target = np.full_like(x_fit, (y1v + y2v) / 2.0, float)
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        return polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)

    # === 情况 E：仅有一个单点 ===
    if len(singles) == 1:
        c0, _, x_s, y_s = singles[0]
        x_sv, y_sv = float(x_s[0]), float(y_s[0])
        # 以单点为基准，按颜色方向平移一个 default_offset 估中线
        ymid = y_sv - default_offset if c0 == 0 else y_sv + default_offset
        x_start = x_sv
        y_target = np.full_like(x_fit, ymid, float)
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        return polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)

    # === 情况 F：无数据 ===
    return np.poly1d([0.0])

