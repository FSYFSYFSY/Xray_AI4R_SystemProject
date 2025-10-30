import pandas as pd
import ast
import json
import numpy as np
from collections import defaultdict
import os



def update_Cones_Map(heading_angle_in_radians, wheel_speed, Map):
    v = wheel_speed
    x = np.cos(heading_angle_in_radians)
    y = np.sin(heading_angle_in_radians)
    dt = 0.1

    derta_x = x * wheel_speed * dt
    derta_y = y * wheel_speed * dt

    new_x, new_y, new_c = [], [], []
    for x, y, c in zip(Map["x_list"], Map["y_list"], Map["c_list"]):
        x_new = x - derta_x
        y_new = y - derta_y
        new_x.append(x_new)
        new_y.append(y_new)
        new_c.append(c)

    return {"x_list": new_x, "y_list": new_y, "c_list": new_c}

def filter_x_range(Map, x_min=0.0, x_max=750.0):
    fx, fy, fc = [], [], []
    for x, y, c in zip(Map["x_list"], Map["y_list"], Map["c_list"]):
        if x_min < x < x_max:
            fx.append(x); fy.append(y); fc.append(c)
    return {"x_list": fx, "y_list": fy, "c_list": fc}

def merge_maps(base, addon, vision=750):
    filtered_x, filtered_y, filtered_c = [], [], []
    for x, y, c in zip(base["x_list"], base["y_list"], base["c_list"]):
        if x > vision:
            filtered_x.append(x)
            filtered_y.append(y)
            filtered_c.append(c)
    return {
        "x_list": list(addon["x_list"]) + filtered_x,
        "y_list": list(addon["y_list"]) + filtered_y,
        "c_list": list(addon["c_list"]) + filtered_c,
    }

def is_empty_detection(frame):
    xs = frame.get("x_list", [])
    ys = frame.get("y_list", [])
    cs = frame.get("c_list", [])
    return (not xs) or (not ys) or (not cs)

def frame_to_map(frame):
    return {
        "x_list": frame.get("x_list", []),
        "y_list": frame.get("y_list", []),
        "c_list": frame.get("c_list", []),
    }

def select_degree(n_points: int) -> int:
    if n_points <= 2:
        return 1
    return 2

def weighted_polyfit(x, y, degree: int, weights=None, l2: float = 0.0) -> np.poly1d:
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    n = len(x)
    degree = int(max(0, min(degree, max(0, n - 1))))
    V = np.vstack([x ** k for k in range(degree, -1, -1)]).T
    if weights is not None:
        w = np.asarray(weights, dtype=float).ravel()
        W = np.diag(w)
        A = V.T @ W @ V
        b = V.T @ W @ y
    else:
        A = V.T @ V
        b = V.T @ y
    if l2 > 0:
        A = A + l2 * np.eye(A.shape[0])
    coeffs = np.linalg.solve(A, b)
    return np.poly1d(coeffs)

def handle_no_points(degree: int = 1):
    return np.poly1d([0.0])

def handle_two_valid_groups(valid_groups, x_fit_full, degree, default_offset, color_map,
                            ridge: float = 0.0, tail_boost: float = 2.0, bandwidth: float = 400.0):
    (c1, n1, x1, y1), (c2, n2, x2, y2) = sorted(valid_groups, key=lambda g: np.min(g[2]))
    deg1 = select_degree(len(x1))
    deg2 = select_degree(len(x2))
    p1 = weighted_polyfit(x1, y1, deg1)
    p2 = weighted_polyfit(x2, y2, deg2)
    off1 = (+default_offset if c1 == 1 else -default_offset)
    off2 = (+default_offset if c2 == 1 else -default_offset)
    y_t1 = p1(x_fit_full) + off1
    y_t2 = p2(x_fit_full) + off2
    def support_weight(xq, xs, h):
        xs = np.asarray(xs)
        dmin = np.min(np.abs(xq[:, None] - xs[None, :]), axis=1)
        return np.exp(-(dmin ** 2) / (2.0 * (h ** 2)))
    w1 = support_weight(x_fit_full, x1, bandwidth)
    w2 = support_weight(x_fit_full, x2, bandwidth)
    s = w1 + w2 + 1e-8
    y_target_blend = (w1 / s) * y_t1 + (w2 / s) * y_t2
    x1_max = np.max(x1); x2_min = np.min(x2)
    if x2_min - x1_max > 1e-6:
        idxL = np.searchsorted(x_fit_full, x1_max)
        idxR = np.searchsorted(x_fit_full, x2_min)
        xL, xR = x_fit_full[idxL], x_fit_full[idxR]
        yL, yR = y_target_blend[idxL], y_target_blend[idxR]
        dyL = (y_target_blend[min(idxL + 1, len(x_fit_full) - 1)] - y_target_blend[idxL]) / \
              max(x_fit_full[min(idxL + 1, len(x_fit_full) - 1)] - xL, 1e-6)
        dyR = (y_target_blend[idxR] - y_target_blend[max(idxR - 1, 0)]) / \
              max(xR - x_fit_full[max(idxR - 1, 0)], 1e-6)
        A = np.array([
            [xL ** 3, xL ** 2, xL, 1.0],
            [xR ** 3, xR ** 2, xR, 1.0],
            [3 * xL ** 2, 2 * xL, 1.0, 0.0],
            [3 * xR ** 2, 2 * xR, 1.0, 0.0]
        ], float)
        b = np.array([yL, yR, dyL, dyR], float)
        a3, a2, a1, a0 = np.linalg.solve(A, b)
        gap_mask = (x_fit_full >= xL) & (x_fit_full <= xR)
        y_target_blend[gap_mask] = a3 * x_fit_full[gap_mask] ** 3 + a2 * x_fit_full[gap_mask] ** 2 + a1 * x_fit_full[gap_mask] + a0
    W = np.linspace(1.0, tail_boost, len(x_fit_full))
    deg_mid = select_degree(len(x_fit_full))
    poly_middle = weighted_polyfit(x_fit_full, y_target_blend, deg_mid, weights=W, l2=ridge)
    return poly_middle

def handle_one_valid_one_single(valid_group, single_group, x_fit_full, degree, color_map,
                                default_offset: float = 500.0, ridge: float = 0.0, tail_boost: float = 2.0):
    color, _, x_vals, y_vals = valid_group
    xs = np.asarray(x_vals, float); ys = np.asarray(y_vals, float)
    deg_base = select_degree(len(xs))
    base_poly = weighted_polyfit(xs, ys, deg_base)
    sc, _, x_s, y_s = single_group
    x_sv, y_sv = float(x_s[0]), float(y_s[0])
    y_at_sv = base_poly(x_sv)
    offset_point = (y_sv - y_at_sv) / 2.0
    y_target_full = base_poly(x_fit_full) + offset_point
    x_start = float(np.min(xs))
    idx0 = np.searchsorted(x_fit_full, x_start)
    X = x_fit_full[idx0:]; Y = y_target_full[idx0:]
    W = np.linspace(1.0, tail_boost, len(X))
    deg_mid = select_degree(len(X))
    poly_middle = weighted_polyfit(X, Y, deg_mid, weights=W, l2=ridge)
    return poly_middle

def handle_one_valid_only(valid_group, x_fit_full, degree, default_offset, color_map,
                          ridge: float = 0.0, tail_boost: float = 2.0):
    color, _, x_vals, y_vals = valid_group
    xs = np.asarray(x_vals, float); ys = np.asarray(y_vals, float)
    deg_base = select_degree(len(xs))
    base_poly = weighted_polyfit(xs, ys, deg_base)
    offset = default_offset if color == 0 else -default_offset
    y_target_full = base_poly(x_fit_full) - offset
    x_start = float(np.min(xs))
    idx0 = np.searchsorted(x_fit_full, x_start)
    X = x_fit_full[idx0:]; Y = y_target_full[idx0:]
    W = np.linspace(1.0, tail_boost, len(X))
    deg_mid = select_degree(len(X))
    poly_middle = weighted_polyfit(X, Y, deg_mid, weights=W, l2=ridge)
    return poly_middle

def handle_two_single_points(single_groups, x_fit_full, degree, color_map, ridge: float = 0.0, tail_boost: float = 2.0):
    (c1, _, x1, y1), (c2, _, x2, y2) = single_groups
    x1v, y1v = float(x1[0]), float(y1[0])
    x2v, y2v = float(x2[0]), float(y2[0])
    x_start = min(x1v, x2v)
    target_const = (y1v + y2v) / 2.0
    idx0 = np.searchsorted(x_fit_full, x_start)
    X = x_fit_full[idx0:]; Y = np.full_like(X, target_const, dtype=float)
    W = np.linspace(1.0, tail_boost, len(X))
    deg_mid = select_degree(len(X))
    poly_middle = weighted_polyfit(X, Y, deg_mid, weights=W, l2=ridge)
    return poly_middle

def handle_one_single_point(single_group, x_fit_full, degree, default_offset, color_map,
                            ridge: float = 0.0, tail_boost: float = 2.0):
    color, _, x_s, y_s = single_group
    x_sv, y_sv = float(x_s[0]), float(y_s[0])
    c_name = color_map.get(color, 'gray')
    if (y_sv - 50) <= 0 <= (y_sv + 50):
        if c_name == 'blue':
            ymid = y_sv + default_offset
        elif c_name == 'yellow':
            ymid = y_sv - default_offset
        else:
            ymid = y_sv
    else:
        ymid = y_sv
    x_start = x_sv
    idx0 = np.searchsorted(x_fit_full, x_start)
    X = x_fit_full[idx0:]; Y = np.full_like(X, ymid, dtype=float)
    W = np.linspace(1.0, tail_boost, len(X))
    deg_mid = select_degree(len(X))
    poly_middle = weighted_polyfit(X, Y, deg_mid, weights=W, l2=ridge)
    return poly_middle

def handle_error_case():
    print("Error case, no middle line generated")
    return None

def clean_single_far_point(frame, yellow_code=0, blue_code=1,
                           half_rule='midpoint', include_equal=False):
    x_list = frame.get('x_list', [])
    y_list = frame.get('y_list', [])
    c_list = frame.get('c_list', [])
    if not x_list or not y_list or not c_list:
        return []
    x = np.asarray(x_list, dtype=float)
    y = np.asarray(y_list, dtype=float)
    c = np.asarray(c_list, dtype=int)
    idx_yellow = np.where(c == yellow_code)[0]
    idx_blue = np.where(c == blue_code)[0]
    to_remove = []
    def half_threshold(xs_other):
        if xs_other.size == 0:
            return None
        if half_rule == 'median':
            return float(np.median(xs_other))
        else:
            return float((xs_other.min() + xs_other.max()) / 2.0)
    if idx_yellow.size == 1 and idx_blue.size >= 1:
        i = idx_yellow[0]
        thr = half_threshold(x[idx_blue])
        if thr is not None:
            cond = (x[i] >= thr) if include_equal else (x[i] > thr)
            if cond:
                to_remove.append(i)
    if idx_blue.size == 1 and idx_yellow.size >= 1:
        i = idx_blue[0]
        thr = half_threshold(x[idx_yellow])
        if thr is not None:
            cond = (x[i] >= thr) if include_equal else (x[i] > thr)
            if cond:
                to_remove.append(i)
    removed_info = []
    if to_remove:
        to_keep = np.ones(len(x_list), dtype=bool)
        to_keep[to_remove] = False
        for i in to_remove:
            removed_info.append((int(c[i]), float(x[i]), float(y[i]), int(i)))
        frame['x_list'] = [float(v) for v in x[to_keep]]
        frame['y_list'] = [float(v) for v in y[to_keep]]
        frame['c_list'] = [int(v) for v in c[to_keep]]
    return removed_info


# Main function to generate middle line polynomial
def generate_middle_line(frame, degree=3, default_offset=500,
                         ridge=0.0, tail_boost=2.0):
    x_list = frame.get('x_list', [])
    y_list = frame.get('y_list', [])
    c_list = frame.get('c_list', [])
    color_map = {0: 'yellow', 1: 'blue'}
    removed = clean_single_far_point(frame,
                                     yellow_code=0, blue_code=1,
                                     half_rule='midpoint', include_equal=False)
    if not x_list or not y_list:
        return handle_no_points(degree)
    color_groups = defaultdict(list)
    for x, y, c in zip(x_list, y_list, c_list):
        color_groups[c].append((x, y))
    group_info = []
    for color, pts in color_groups.items():
        pts_sorted = sorted(pts, key=lambda p: p[0])
        xs = np.array([p[0] for p in pts_sorted], dtype=float)
        ys = np.array([p[1] for p in pts_sorted], dtype=float)
        group_info.append((color, len(pts_sorted), xs, ys))
    valid_groups = [g for g in group_info if g[1] >= 2]
    single_groups = [g for g in group_info if g[1] == 1]
    xs_all = np.concatenate([g[2] for g in group_info]) if group_info else np.array([0.0])
    x_max = float(xs_all.max()) if xs_all.size > 0 else 1.0
    x_fit_full = np.linspace(0.0, x_max, 400)
    if len(valid_groups) >= 2:
        return handle_two_valid_groups(valid_groups, x_fit_full, degree, default_offset, color_map,
                                       ridge=ridge, tail_boost=tail_boost)
    if len(valid_groups) == 1 and len(single_groups) == 1:
        _, _, x_vals, _ = valid_groups[0]
        median_x = float(np.median(x_vals))
        _, _, x_s, _ = single_groups[0]
        x_sv = float(x_s[0])
        if x_sv > median_x:
            return handle_one_valid_only(valid_groups[0], x_fit_full, degree, default_offset, color_map,
                                         ridge=ridge, tail_boost=tail_boost)
        else:
            return handle_one_valid_one_single(valid_groups[0], single_groups[0], x_fit_full, degree, color_map,
                                               default_offset=default_offset, ridge=ridge, tail_boost=tail_boost)
    if len(valid_groups) == 1 and not single_groups:
        return handle_one_valid_only(valid_groups[0], x_fit_full, degree, default_offset, color_map,
                                     ridge=ridge, tail_boost=tail_boost)
    if not valid_groups and len(single_groups) == 2:
        return handle_two_single_points(single_groups, x_fit_full, degree, color_map,
                                        ridge=ridge, tail_boost=tail_boost)
    if not valid_groups and len(single_groups) == 1:
        return handle_one_single_point(single_groups[0], x_fit_full, degree, default_offset, color_map,
                                       ridge=ridge, tail_boost=tail_boost)
    return handle_error_case()


#The protocal of implementing the path prediction
def process_frame_and_update(
    x_array,
    y_array,
    c_array,
    Global_Map,
    heading_angle_in_radians = 0.0,
    wheel_speed = 1000.0
):
    frame = {
        "x_list": list(x_array),
        "y_list": list(y_array),
        "c_list": list(c_array),
        "heading_angle_in_radians": heading_angle_in_radians,
        "wheel_speed": wheel_speed
    }

    if is_empty_detection(frame):
        return generate_middle_line(Global_Map), Global_Map

    if is_empty_detection(Global_Map):
        Global_Map = frame_to_map(frame)
        return generate_middle_line(Global_Map), Global_Map

    predicted_map = update_Cones_Map(heading_angle_in_radians, wheel_speed, Global_Map)

    predicted_filtered = filter_x_range(predicted_map, x_min=0.0, x_max=750.0)

    current_map = frame_to_map(frame)

    fused_map = merge_maps(current_map, predicted_filtered)

    poly_middle = generate_middle_line(fused_map)

    Global_Map = fused_map

    return poly_middle, Global_Map
        
