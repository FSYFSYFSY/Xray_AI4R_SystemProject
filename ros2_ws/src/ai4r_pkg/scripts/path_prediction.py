import numpy as np
from collections import defaultdict
from numpy.polynomial import Chebyshev, Polynomial

# ========== 工具函数 ==========

def cheby_polyfit_to_poly1d(x, y, degree):
    """切比雪夫最小二乘拟合，比普通多项式更稳"""
    degree = max(1, int(degree))
    dom = [float(np.min(x)), float(np.max(x))]
    ch = Chebyshev.fit(x, y, deg=degree, domain=dom)
    p = ch.convert(kind=Polynomial, domain=dom)
    return np.poly1d(p.coef[::-1])

def polyfit_through_origin_scaled(x, y, degree: int, weights=None, l2=0.0):
    """强制过原点拟合，支持权重和岭正则"""
    degree = max(1, int(degree))
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    xmax = np.max(np.abs(x)) or 1.0
    t = x / xmax
    T = np.vstack([t**k for k in range(1, degree+1)]).T

    if weights is not None:
        w = np.asarray(weights, float).reshape(-1, 1)
        Tw = T * w
        yw = y * w.ravel()
    else:
        Tw, yw = T, y

    I = np.eye(degree)
    b = np.linalg.solve(Tw.T @ Tw + l2 * I, Tw.T @ yw)
    a = np.array([b[k-1] / (xmax**k) for k in range(1, degree+1)], float)
    return np.poly1d(np.concatenate([a[::-1], [0.0]]))

def cubic_bridge_samples(x0, y0, dy0, n=40):
    """生成从(0,0)平滑到(x0,y0,dy0)的三次桥接曲线样本"""
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
    """法线方向偏移的等效竖直位移"""
    slope = np.polyder(p)(x)
    dv = d / np.sqrt(1.0 + slope**2)
    if max_abs is not None:
        dv = np.clip(dv, -abs(max_abs), abs(max_abs))
    return dv

# ========== 数据清理与锚点 ==========

def clean_single_far_point_lists(x_list, y_list, c_list,
                                 yellow_code=0, blue_code=1,
                                 half_rule='midpoint', include_equal=False):
    """删除落在另一边后半段的孤立点"""
    if not x_list:
        return x_list, y_list, c_list

    x = np.asarray(x_list, float)
    y = np.asarray(y_list, float)
    c = np.asarray(c_list, int)

    idx_y = np.where(c == yellow_code)[0]
    idx_b = np.where(c == blue_code)[0]

    def half_threshold(xs):
        if xs.size == 0: return None
        return float(np.median(xs)) if half_rule == 'median' else float((xs.min()+xs.max())/2.0)

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
        return x[keep].tolist(), y[keep].tolist(), c[keep].tolist()
    return x_list, y_list, c_list

def add_conditional_anchor_cones_lists(x_list, y_list, c_list,
                                       offset=500.0,
                                       x_positions=(0.0, 250.0),
                                       origin_y=0.0,
                                       yellow_code=0, blue_code=1,
                                       dedup_tol=1e-6):
    """若某颜色存在，则在x_positions处添加锚点（上下±offset）"""
    xs = np.asarray(x_list, float) if x_list else np.array([])
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
                         ridge=None, tail_boost=2.0,
                         add_start_anchors=True,
                         anchor_positions=(0.0, 250.0),
                         do_clean_single_far=True,
                         max_normal_offset=None):
    """
    输入: x_list, y_list, c_list
    输出: poly_middle (numpy.poly1d)
    """
    x_list = list(map(float, x_list))
    y_list = list(map(float, y_list))
    c_list = list(map(int,   c_list))

    # 清理孤立点
    if do_clean_single_far:
        x_list, y_list, c_list = clean_single_far_point_lists(x_list, y_list, c_list)

    # 条件锚点
    if add_start_anchors:
        x_list, y_list, c_list = add_conditional_anchor_cones_lists(
            x_list, y_list, c_list, offset=default_offset, x_positions=anchor_positions)

    # 分组
    groups = defaultdict(list)
    for x, y, c in zip(x_list, y_list, c_list):
        groups[c].append((x, y))

    group_info = []
    for color, pts in groups.items():
        pts_sorted = sorted(pts, key=lambda p: p[0])
        xs = np.array([p[0] for p in pts_sorted], float)
        ys = np.array([p[1] for p in pts_sorted], float)
        group_info.append((color, len(pts_sorted), xs, ys))

    valid = [g for g in group_info if g[1] >= 2]
    singles = [g for g in group_info if g[1] == 1]

    xs_all = np.array(x_list) if x_list else np.array([0.0])
    x_max = float(xs_all.max()) if xs_all.size else 1.0
    x_fit = np.linspace(0.0, max(x_max, 1.0), 400)

    # ridge 自动估计
    if ridge is None:
        x_rng = max(1.0, x_max - 0.0)
        ridge = float(min(max(1e-4 * degree * (x_rng / 1000.0), 1e-6), 5e-3))

    # ---- 根据类型决定拟合逻辑 ----
    def _fit_edge(xs, ys):
        return cheby_polyfit_to_poly1d(xs, ys, degree)

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
        W = np.concatenate([np.full_like(xb, 0.6, float),
                            np.linspace(1.0, tail_boost, len(x_fit)-idx0)])
        return X, Y, W

    # === 两条边 ===
    if len(valid) >= 2:
        (c1, n1, x1, y1), (c2, n2, x2, y2) = sorted(valid, key=lambda g: np.min(g[2]))
        p1 = _fit_edge(x1, y1)
        p2 = _fit_edge(x2, y2)

        if use_normal_offset:
            y_t1 = p1(x_fit) + (-1 if c1==0 else +1)*normal_vertical_offset_arr(p1, x_fit, default_offset, max_normal_offset)
            y_t2 = p2(x_fit) + (-1 if c2==0 else +1)*normal_vertical_offset_arr(p2, x_fit, default_offset, max_normal_offset)
        else:
            off1 = (+default_offset if c1==1 else -default_offset)
            off2 = (+default_offset if c2==1 else -default_offset)
            y_t1 = p1(x_fit) + off1
            y_t2 = p2(x_fit) + off2

        # 权重融合
        def support_weight(xq, xs, h):
            dmin = np.min(np.abs(xq[:,None] - xs[None,:]), axis=1)
            return np.exp(-(dmin**2)/(2*h*h))
        bandwidth = max(300.0, 0.1*(x_fit.max()-x_fit.min()))
        w1 = support_weight(x_fit, x1, bandwidth)
        w2 = support_weight(x_fit, x2, bandwidth)
        s = w1 + w2 + 1e-8
        y_target = (w1/s)*y_t1 + (w2/s)*y_t2

        x_start = min(np.min(x1), np.min(x2))
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        poly_middle = polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)
        return poly_middle

    # === 一条边 + 单点 ===
    if len(valid) == 1 and len(singles) == 1:
        color, _, xs, ys = valid[0]
        p = _fit_edge(xs, ys)
        sc, _, x_s, y_s = singles[0]
        x_sv, y_sv = float(x_s[0]), float(y_s[0])
        y_at_sv = float(p(x_sv))
        offset_point = (y_sv - y_at_sv)/2.0
        y_target = p(x_fit) + offset_point
        x_start = float(np.min(xs))
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        return polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)

    # === 单边 ===
    if len(valid) == 1:
        color, _, xs, ys = valid[0]
        p = _fit_edge(xs, ys)
        if use_normal_offset:
            sign = -1 if color==0 else +1
            y_target = p(x_fit) + sign*normal_vertical_offset_arr(p, x_fit, default_offset, max_normal_offset)
        else:
            offset = default_offset if color==0 else -default_offset
            y_target = p(x_fit) - offset
        x_start = float(np.min(xs))
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        return polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)

    # === 两个单点 ===
    if len(singles) == 2:
        (c1, _, x1, y1), (c2, _, x2, y2) = singles
        x1v, y1v = float(x1[0]), float(y1[0])
        x2v, y2v = float(x2[0]), float(y2[0])
        x_start = min(x1v, x2v)
        y_target = np.full_like(x_fit, (y1v + y2v)/2.0, float)
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        return polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)

    # === 一个单点 ===
    if len(singles) == 1:
        c0, _, x_s, y_s = singles[0]
        x_sv, y_sv = float(x_s[0]), float(y_s[0])
        ymid = y_sv - default_offset if c0==0 else y_sv + default_offset
        x_start = x_sv
        y_target = np.full_like(x_fit, ymid, float)
        X, Y, W = _build_bridge(x_fit, y_target, x_start)
        return polyfit_through_origin_scaled(X, Y, degree, weights=W, l2=ridge)

    # 无数据，返回零函数
    return np.poly1d([0.0])
