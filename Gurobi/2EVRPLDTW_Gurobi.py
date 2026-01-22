# -*- coding: utf-8 -*-
import os
import math
import time
import numpy as np
import pandas as pd
import gurobipy as gp
from gurobipy import GRB
from collections import defaultdict


def safe_save_table(df: pd.DataFrame, xlsx_path: str):
    base, _ = os.path.splitext(xlsx_path)
    try:
        import openpyxl  # noqa: F401
        df.to_excel(xlsx_path, index=False)
        print(f"[save] 已保存 Excel: {xlsx_path}")
    except Exception:
        try:
            import xlsxwriter  # noqa: F401
            df.to_excel(xlsx_path, index=False, engine="xlsxwriter")
            print(f"[save] 已保存 Excel(xlsxwriter): {xlsx_path}")
        except Exception:
            csv_path = base + ".csv"
            df.to_csv(csv_path, index=False, encoding="utf-8-sig")
            print(f"[save] 未检测到 Excel 引擎，已改存 CSV: {csv_path}")


BASE_DIR   = r"F:/Onedrive/PlatEMO-2EVRPLDTW/Instance"
TIME_LIMIT = 3600
MIP_GAP    = 0.0
OUTPUT_LOG = 1

# If True: force full-pool exact benchmarking on small instances
EXACT_SOLVE_MODE = True



ROUTE_MAX_STOPS_EXACT = 10**9
ENUM_TIME_CAP         = None
ENUM_NODE_CAP         = None

PRICING_ROUNDS     = 0
PRICING_BEAM_WIDTH = 50
PRICING_MAX_STOPS  = 7
PRICING_ADD_PER_S  = 10
RC_EPS             = 1e-6

MAX_TRUCKS_MULT    = 3


USE_DUMMY_COLUMNS  = True
DUMMY_COST         = 1e6

# In exact benchmarking mode, dummy columns must be disabled; otherwise the model may use
# dummy coverage at a penalty, which is not a true full-model optimum.
if 'EXACT_SOLVE_MODE' in globals() and EXACT_SOLVE_MODE:
    USE_DUMMY_COLUMNS = False


# ========= 成本与物理参数 =========
Q1 = 100.0
Q2 = 10
h1, h2 = 12.5, 3.36
c_v, c_u = 0.1286, 0.001


DR = {'W':5.0, 'f_s':10.0, 'E':550.0, 'n':4, 'rho':1.225, 'zeta':100.0*math.pi, 'g':9.81}


def read_instance(csv_path):

    df0 = pd.read_csv(csv_path)
    try:
        _ = df0.iloc[:, 0:2].to_numpy(dtype=float)
        df = df0
    except Exception:
        df = pd.read_csv(csv_path, skiprows=1)

    coords         = df.iloc[:, 0:2].to_numpy(dtype=float)
    demands_raw    = df.iloc[:, 4].to_numpy(dtype=float)
    attributes     = df.iloc[:, 6].astype(str).to_numpy()
    cust_type_raw  = df.iloc[:, 9].to_numpy()

    # ---- 节点类型划分：depot / sat / cust ----
    idx_depot = np.where(attributes == "depot")[0].tolist()
    idx_sat   = np.where(attributes == "sat")[0].tolist()
    idx_cust  = np.where(attributes == "cust")[0].tolist()

    if len(idx_depot) < 1 or len(idx_sat) < 1 or len(idx_cust) < 1:
        raise RuntimeError("实例必须包含 >=1 depot, >=1 sat, >=1 customer")

    coord_dep = coords[idx_depot, :]
    coord_sat = coords[idx_sat, :]
    coord_cus = coords[idx_cust, :]
    all_coords = np.vstack([coord_dep, coord_sat, coord_cus])

    num_dep = len(idx_depot)
    num_sat = len(idx_sat)
    num_cus = len(idx_cust)
    num_nodes = num_dep + num_sat + num_cus

    # ---- 需求方向与量（送货 -1 / 取件 +1）----
    demand_cus = (demands_raw[idx_cust] / 10.0).astype(float)

    def _to_type(x):
        try:
            v = float(x)
            if v == -1: return -1
            if v == 1:  return  1
        except Exception:
            s = str(x).strip().lower()
            if s in ["delivery","deliver","d","-1"]: return -1
            if s in ["pickup","pick","p","+1","1"]:  return  1
        return -1

    cust_type = np.array([_to_type(x) for x in cust_type_raw[idx_cust]], dtype=int)
    Dd = np.where(cust_type == -1, np.rint(demand_cus), 0.0).astype(int)
    Dp = np.where(cust_type ==  1, np.rint(demand_cus), 0.0).astype(int)

    # ---- 索引映射 ----
    P = list(range(0, num_dep))                          # 仓库：0..|P|-1
    S = list(range(num_dep, num_dep + num_sat))          # 卫星：|P|..|P|+|S|-1
    Z = list(range(num_dep + num_sat, num_nodes))        # 客户
    def cust_local_to_node(k): return num_dep + num_sat + k

    # ---- 距离矩阵（单位保持和你原代码一致：/10）----
    dist = np.zeros((num_nodes, num_nodes))
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                dist[i, j] = math.hypot(all_coords[i,0]-all_coords[j,0],
                                        all_coords[i,1]-all_coords[j,1])
    dist /= 10.0

    # ---- 时间窗与服务时间：如果实例里有则读，没有则给宽松默认 ----
    tw_start = np.zeros(num_cus, dtype=float)
    tw_end   = np.full(num_cus, float('inf'), dtype=float)
    service  = np.zeros(num_cus, dtype=float)

    # 统一把列名转成小写+去空格，方便匹配
    colmap = {c.lower().strip(): i for i, c in enumerate(df.columns)}

    # 尝试一些常见命名：这里包含你当前实例的 "start TW" / "end TW" / "service time"
    for name in ["start tw", "start_tw", "ready", "earliest", "e"]:
        key = name.lower()
        if key in colmap:
            tw_start = df.iloc[idx_cust, colmap[key]].to_numpy(dtype=float)
            break

    for name in ["end tw", "end_tw", "due", "latest", "l"]:
        key = name.lower()
        if key in colmap:
            tw_end = df.iloc[idx_cust, colmap[key]].to_numpy(dtype=float)
            break

    for name in ["service time", "servicetime", "service"]:
        key = name.lower()
        if key in colmap:
            service = df.iloc[idx_cust, colmap[key]].to_numpy(dtype=float)
            break


    # 卡车弧集：P∪S，禁 P->P
    nodes_truck = list(range(num_dep + num_sat))
    A_truck = [(i,j) for i in nodes_truck for j in nodes_truck if i!=j and not (i in P and j in P)]

    return {
        "coords_all": all_coords,
        "num_dep": num_dep, "num_sat": num_sat, "num_cus": num_cus, "num_nodes": num_nodes,
        "P": P, "S": S, "Z": Z,
        "Dd": Dd, "Dp": Dp,
        "dist": dist,
        "A_truck": A_truck,
        "cust_local_to_node": cust_local_to_node,
        "tw_start": tw_start,
        "tw_end": tw_end,
        "service": service
    }



_const = (DR['g']**3) / math.sqrt(2.0*DR['rho']*DR['zeta']*DR['n'])
_const = (DR['g']**3) / math.sqrt(2.0*DR['rho']*DR['zeta']*DR['n'])

def energy_segment(load, d_ij):
    """飞行段能耗（Wh）。load 为当前载荷（kg 或与需求同量纲），d_ij 为距离（与 dist 同量纲）。"""
    t_hours = d_ij / max(1e-9, DR['f_s'])
    power_W = (DR['W'] + load)**1.5 * _const
    return power_W * t_hours

def hover_energy(load, wait_h):
    """悬停等待能耗（Wh）。wait_h 与时间窗/飞行时间使用同一时间单位（通常是小时）。"""
    if wait_h <= 1e-12:
        return 0.0
    power_W = (DR['W'] + load)**1.5 * _const
    return power_W * wait_h

def route_feasible_energy(s, seq, inst):
    """
    检查给定卫星 s 出发、访问客户序列 seq 的无人机子回路是否可行，
    并返回 (可行性, 总能耗E_total, 起飞载重start_load)。

    关键规则（与你的需求一致）：
    1) 能耗 = 各飞行段能耗 +（除首客户外）时间窗等待悬停能耗（按当段到达时载荷计算）；
    2) 首个客户不等待：允许延后起飞，使到达首客户时间 = max(飞行时间, 首客户时间窗起点)；
    3) 最后一段返回“最近卫星”（按距离最小）。
    """
    P, S, Z = inst["P"], inst["S"], inst["Z"]
    num_dep, num_sat = inst["num_dep"], inst["num_sat"]
    Dd, Dp = inst["Dd"], inst["Dp"]
    dist   = inst["dist"]

    tw_start = inst.get("tw_start", None)
    tw_end   = inst.get("tw_end", None)
    service  = inst.get("service", None)
    has_tw   = (tw_start is not None) and (tw_end is not None)

    if not seq:
        return False, 0.0, 0

    # 起飞载重 = 该子回路的总送货量
    start_load = 0
    for z in seq:
        zloc = z - (num_dep + num_sat)
        start_load += int(Dd[zloc])
    if start_load > Q2:
        return False, 0.0, 0

    load = float(start_load)
    E_total = 0.0

    # ---------- 首客户：延后起飞（不产生等待悬停能耗） ----------
    prev = s
    z0 = seq[0]
    zloc0 = z0 - (num_dep + num_sat)
    d0 = float(dist[prev, z0])
    t_fly0 = d0 / max(1e-9, DR['f_s'])
    if has_tw:
        e0 = float(tw_start[zloc0]) / 60.0  # minutes -> hours
        l0 = float(tw_end[zloc0]) / 60.0   # minutes -> hours
        # 若直接飞行都超过最晚到达，则不可行
        if t_fly0 > l0 + 1e-9:
            return False, 0.0, 0
        t = max(t_fly0, e0)   # 通过延后起飞消除首客户等待
    else:
        t = t_fly0

    # 飞行能耗：从卫星到首客户
    E_total += energy_segment(load, d0)
    if E_total > DR['E'] + 1e-9:
        return False, 0.0, 0

    # 首客户服务
    if service is not None and len(service) > 0:
        t += float(service[zloc0]) / 60.0  # minutes -> hours

    # 载重更新：送货减少，取件增加
    load = load - float(Dd[zloc0]) + float(Dp[zloc0])
    if load < -1e-9 or load > Q2 + 1e-9:
        return False, 0.0, 0

    prev = z0

    # ---------- 后续客户：时间窗等待产生悬停能耗 ----------
    for z in seq[1:]:
        zloc = z - (num_dep + num_sat)
        dij = float(dist[prev, z])
        t_fly = dij / max(1e-9, DR['f_s'])
        t += t_fly

        if has_tw:
            e = float(tw_start[zloc]) / 60.0  # minutes -> hours
            l = float(tw_end[zloc]) / 60.0   # minutes -> hours
            if t > l + 1e-9:
                return False, 0.0, 0
            if t < e - 1e-9:
                wait_h = e - t
                # 等待发生在服务前，载荷为“到达该客户时”的载荷
                E_total += hover_energy(load, wait_h)
                if E_total > DR['E'] + 1e-9:
                    return False, 0.0, 0
                t = e

        # 服务时间
        if service is not None and len(service) > 0:
            t += float(service[zloc]) / 60.0  # minutes -> hours

        # 飞行能耗（prev->z），按“起飞前载荷”（即到达前载荷）计算
        E_total += energy_segment(load, dij)
        if E_total > DR['E'] + 1e-9:
            return False, 0.0, 0

        # 载重更新
        load = load - float(Dd[zloc]) + float(Dp[zloc])
        if load < -1e-9 or load > Q2 + 1e-9:
            return False, 0.0, 0

        prev = z

    # ---------- 返航：返回最近卫星 ----------
    best_sat = None
    best_d = float("inf")
    # 返航：返回“最后一个客户”最近的卫星（按距离最小）
    # 注意：这里用 prev（即最后一个客户）计算距离，而不是用起飞卫星 s
    for s2 in S:
        d_back = float(dist[prev, s2])
        if d_back < best_d:
            best_d = d_back
            best_sat = s2

    if best_sat is None:
        return False, None, None
# 返航飞行能耗
    E_total += energy_segment(load, best_d)
    if E_total > DR['E'] + 1e-9:
        return False, 0.0, 0

    return True, float(E_total), int(start_load)

def build_pool_exact(inst):
    """
    生成“全列池（full pool）”：
    - 不限制 route 长度（上限设为客户数），DFS 枚举所有可行序列；
    - 对每个 coverage set 仅保留能耗最小的那条序列；
    - 再做一次简单的支配性过滤（子集+能耗不优的剔除）。
    """
    P, S, Z = inst["P"], inst["S"], inst["Z"]
    dist = inst["dist"]
    num_dep, num_sat = inst["num_dep"], inst["num_sat"]
    num_cus = inst["num_cus"]

    max_stops = min(num_cus, ROUTE_MAX_STOPS_EXACT if ROUTE_MAX_STOPS_EXACT is not None else num_cus)

    routes_by_s = {s: [] for s in S}
    energy_by_sr, cover_mat, deliv_mat = {}, {}, {}

    for s in S:
        Z_sorted = sorted(Z, key=lambda z: dist[s, z])

        # memo: (tuple(path)) -> (ok, E, ds)
        memo = {}

        def eval_path(path):
            key = tuple(path)
            if key in memo:
                return memo[key]
            ok, E, ds = route_feasible_energy(s, list(path), inst)
            memo[key] = (ok, E, ds)
            return memo[key]

        # best per subset (bitmask over customers 0..num_cus-1)
        best = {}  # mask -> (E, path_tuple, ds)

        def mask_of(path):
            m = 0
            for z in path:
                zloc = z - (num_dep + num_sat)
                m |= (1 << zloc)
            return m

        def dfs(path):
            if len(path) > 0:
                ok, E, ds = eval_path(path)
                if ok:
                    msk = mask_of(path)
                    cur = best.get(msk, None)
                    if (cur is None) or (E < cur[0] - 1e-12):
                        best[msk] = (E, tuple(path), ds)

            if len(path) >= max_stops:
                return

            used = set(path)
            # 按离卫星距离排序扩展（更容易先找到可行长路）
            for z in Z_sorted:
                if z in used:
                    continue
                ok, _, _ = eval_path(tuple(path) + (z,))
                if ok:
                    dfs(tuple(path) + (z,))

        dfs(tuple())

        # 转为列表并做简单支配过滤：如果 coverage_i ⊂ coverage_j 且 E_i >= E_j 则删 i
        cov_list = []
        for msk, (E, path, ds) in best.items():
            cov = {i for i in range(num_cus) if (msk >> i) & 1}
            cov_list.append((cov, path, float(E), int(ds)))
        cov_list.sort(key=lambda x: (len(x[0]), x[2]))

        final = []
        for i, (cov_i, path_i, E_i, ds_i) in enumerate(cov_list):
            dominated = False
            for j, (cov_j, path_j, E_j, ds_j) in enumerate(cov_list):
                if i == j:
                    continue
                if cov_i and cov_i.issubset(cov_j) and cov_i != cov_j and E_i >= E_j - 1e-12:
                    dominated = True
                    break
            if not dominated:
                final.append((path_i, E_i, ds_i))

        for r, (path, E, ds) in enumerate(final):
            routes_by_s[s].append(list(path))
            energy_by_sr[(s, r)] = float(E)
            cover_mat[(s, r)] = set(z - (num_dep + num_sat) for z in path)
            deliv_mat[(s, r)] = int(ds)

    return routes_by_s, energy_by_sr, cover_mat, deliv_mat

def warn_uncovered_customers(inst, cover_mat, routes_by_s):
    num_cus = inst["num_cus"]
    uncovered = []
    for zloc in range(num_cus):
        appears = any(zloc in cover_mat[(s, r)]
                      for s in inst["S"] for r in range(len(routes_by_s[s])))
        if not appears:
            uncovered.append(zloc)
    if uncovered:
        print("[WARN] 列池不完整：",
              [u+1 for u in uncovered])  # 1-based
    return uncovered


def build_master(inst, routes_by_s, energy_by_sr, cover_mat, deliv_mat, as_lp=False, w1=1.0, w2=1.0):
    P,S = inst["P"],inst["S"]
    dist = inst["dist"]
    A_truck = inst["A_truck"]
    Dd, Dp = inst["Dd"], inst["Dp"]
    num_cus = inst["num_cus"]

    total_dem = float((inst["Dd"] + inst["Dp"]).sum())
    max_trucks = max(1, math.ceil(max(1e-9, total_dem) / Q1) * 2)
    max_trucks = min(max_trucks, len(inst["S"]))  # 也可直接 = len(S)
    V = list(range(max_trucks))

    m = gp.Model("2E_master_exact")
    m.Params.OutputFlag = OUTPUT_LOG
    V = list(range(max_trucks))
    VTYPE = GRB.CONTINUOUS if as_lp else GRB.BINARY

    x   = m.addVars(A_truck, V, lb=0, ub=1, vtype=VTYPE, name="x")
    tau = m.addVars(V,               lb=0, ub=1, vtype=VTYPE, name="tau")
    u_tr = m.addVars(S, V, lb=0.0, ub=len(S), vtype=GRB.CONTINUOUS, name="u_tr")
    t_unload = m.addVars(S, V, lb=0.0, ub=Q1, vtype=GRB.CONTINUOUS, name="t_unload")
    y_dep = m.addVars(P, V, vtype=VTYPE, name="y_dep")

    gamma = {}
    for s in S:
        for r in range(len(routes_by_s[s])):
            gamma[(s,r)] = m.addVar(lb=0, ub=1, vtype=VTYPE, name=f"gamma[{s},{r}]")
    m.update()

    # ---------------- Weighted-sum scalarization (two objectives) ----------------
    # obj1 (fixed):    h1 * (#trucks used) + h2 * (#drone routes used)
    # obj2 (variable): truck travel cost + drone energy cost
    #
    # Gurobi solves:   min  w1*obj1 + w2*obj2
    #
    # NOTE: Sweeping (w1,w2) may produce repeated optimal solutions (same (obj1,obj2))
    # but it is still a standard way to generate multiple Pareto-relevant points.

    # Dummy columns (LP only): allow cover constraints == 1 even if the pool is incomplete.
    # Penalty is added to the objective directly to discourage dummy usage.
    var_dum = {}
    dummy_penalty = 0.0
    if USE_DUMMY_COLUMNS and as_lp:
        for zloc in range(num_cus):
            v = m.addVar(lb=0, ub=1, vtype=GRB.CONTINUOUS, name=f"gamma_dum[{zloc}]")
            var_dum[zloc] = v
            dummy_penalty += DUMMY_COST * v
        m.update()

    obj1_expr = gp.quicksum(h1 * tau[v] for v in V) \
              + gp.quicksum(h2 * gamma[(s, r)] for s in S for r in range(len(routes_by_s[s])))

    obj2_expr = gp.quicksum(c_v * dist[i, j] * x[i, j, v] for (i, j) in A_truck for v in V) \
              + gp.quicksum(c_u * energy_by_sr[(s, r)] * gamma[(s, r)]
                            for s in S for r in range(len(routes_by_s[s])))

    obj_weighted = float(w1) * obj1_expr + float(w2) * obj2_expr + dummy_penalty
    m.setObjective(obj_weighted, GRB.MINIMIZE)


    for v in V:
        m.addConstr(gp.quicksum(x[i,j,v] for i in P for j in S if (i,j) in A_truck) == tau[v], name=f"dep_v{v}")
        m.addConstr(gp.quicksum(x[i,j,v] for i in S for j in P if (i,j) in A_truck) == tau[v], name=f"ret_v{v}")

    for v in V:
        m.addConstr(gp.quicksum(y_dep[p, v] for p in P) == tau[v], name=f"one_dep_v{v}")
        for p in P:
            m.addConstr(gp.quicksum(x[p, s, v] for s in S if (p, s) in A_truck) == y_dep[p, v],
                        name=f"start_at_p{p}_v{v}")
            m.addConstr(gp.quicksum(x[s, p, v] for s in S if (s, p) in A_truck) == y_dep[p, v],
                        name=f"return_to_p{p}_v{v}")


    for v in V:
        for s in S:
            m.addConstr(gp.quicksum(x[i,s,v] for i in (P+S) if (i,s) in A_truck) ==
                        gp.quicksum(x[s,j,v] for j in (P+S) if (s,j) in A_truck),
                        name=f"flow_s{s}_v{v}")

    # MTZ on S
    if len(S) >= 2:
        T = len(S)
        for v in V:
            for i in S:
                for j in S:
                    if i!=j and (i,j) in A_truck:
                        m.addConstr(u_tr[i,v] - u_tr[j,v] + T*x[i,j,v] <= T-1,
                                    name=f"mtz_s{i}_s{j}_v{v}")


    for v in V:
        for (i,j) in A_truck:
            m.addConstr(x[i,j,v] <= tau[v], name=f"use_x_tau_{i}_{j}_v{v}")

    for s in S:
        visit_s = gp.quicksum(x[i, s, v] for v in V for i in (P + S) if (i, s) in A_truck)
        for r in range(len(routes_by_s[s])):
            m.addConstr(gamma[(s, r)] <= visit_s, name=f"route_visit_link_s{s}_r{r}")


    cover_constr = {}

    for zloc in range(num_cus):
        expr = gp.quicksum(gamma[(s,r)] for s in S for r in range(len(routes_by_s[s]))
                           if zloc in cover_mat[(s,r)])
        if USE_DUMMY_COLUMNS and as_lp:
            expr = expr + var_dum[zloc]
        cc = m.addConstr(expr == 1, name=f"cover_z{zloc}")
        cover_constr[zloc] = cc


    for s in S:
        for v in V:
            m.addConstr(t_unload[s,v] <= Q1 * gp.quicksum(x[i,s,v] for i in (P+S) if (i,s) in A_truck),
                        name=f"unload_visit_s{s}_v{v}")
    for v in V:
        m.addConstr(gp.quicksum(t_unload[s,v] for s in S) <= Q1 * tau[v], name=f"unload_cap_v{v}")
    supply_constr = {}
    for s in S:
        rhs = gp.quicksum(deliv_mat[(s,r)] * gamma[(s,r)] for r in range(len(routes_by_s[s])))
        sc = m.addConstr(gp.quicksum(t_unload[s,v] for v in V) >= rhs, name=f"supply_s{s}")
        supply_constr[s] = sc


    for v in range(len(V)-1):
        m.addConstr(tau[v] >= tau[v+1], name=f"sym_tau_{v}")

    m._V = V
    m._inst = inst
    m._w1 = float(w1)
    m._w2 = float(w2)
    return m, gamma, cover_constr, supply_constr

def pricing_round(m, routes_by_s, energy_by_sr, cover_mat, deliv_mat, cover_constr, supply_constr):
    inst = m._inst
    P,S,Z = inst["P"],inst["S"],inst["Z"]
    num_dep, num_sat = inst["num_dep"],inst["num_sat"]
    dist = inst["dist"]

    try:
        pi  = {z: cover_constr[z].Pi for z in cover_constr}
        lam = {s: supply_constr[s].Pi for s in supply_constr}
    except gp.GurobiError:
        return 0

    added = 0
    for s in S:
        cusp = sorted(Z, key=lambda z: dist[s, z])
        seeds=[]
        for z in cusp:
            ok, E, ds = route_feasible_energy(s, [z], inst)
            if ok:
                rc = (h2 + c_u*E) - pi.get(z-(num_dep+num_sat),0.0) - lam.get(s,0.0)*ds
                seeds.append((-rc, [z], E, ds, rc))
        seeds.sort(reverse=True)
        frontier = seeds[:PRICING_BEAM_WIDTH]
        cand=[]
        for _depth in range(2, PRICING_MAX_STOPS+1):
            new_front=[]
            for _, seq, Ecur, dcur, rc_cur in frontier:
                used=set(seq)
                for z in cusp:
                    if z in used: continue
                    ok, E, ds = route_feasible_energy(s, seq+[z], inst)
                    if not ok: continue
                    rc = (h2 + c_u*E) \
                         - sum(pi.get(zz-(num_dep+num_sat),0.0) for zz in (seq+[z])) \
                         - lam.get(s,0.0)*ds
                    new_front.append((-rc, seq+[z], E, ds, rc))
            cand += frontier
            new_front.sort(reverse=True)
            frontier = new_front[:PRICING_BEAM_WIDTH]
        cand += frontier

        seen = set(tuple(seq) for seq in routes_by_s[s])
        negs=[]
        for _, seq, E, ds, rc in cand:
            if rc < -RC_EPS and tuple(seq) not in seen:
                negs.append((rc, seq, E, ds))
        negs.sort(key=lambda x: x[0])

        for rc, seq, E, ds in negs[:PRICING_ADD_PER_S]:
            r = len(routes_by_s[s])
            routes_by_s[s].append(list(seq))
            energy_by_sr[(s,r)] = E
            cover_mat[(s,r)] = set(z-(num_dep+num_sat) for z in seq)
            deliv_mat[(s,r)] = int(ds)
            var = m.addVar(lb=0, ub=1, vtype=GRB.CONTINUOUS, name=f"gamma[{s},{r}]")
            w1 = getattr(m, '_w1', 1.0)
            w2 = getattr(m, '_w2', 1.0)
            var.Obj = w1*h2 + w2*c_u*E
            # 覆盖
            for zloc in cover_mat[(s,r)]:
                m.chgCoeff(m.getConstrByName(f"cover_z{zloc}"), var, 1.0)
            # 供给
            m.chgCoeff(m.getConstrByName(f"supply_s{s}"), var, -ds)
            # 访问联动
            visit_s = gp.quicksum(m.getVarByName(f"x[{i},{s},{v}]")
                                  for v in m._V for i in (inst["P"] + inst["S"]) if (i, s) in inst["A_truck"])
            m.addConstr(var <= visit_s, name=f"route_visit_link_pricing_s{s}_r{r}")
            added += 1

    if added > 0:
        m.update()
    return added


def extract_paths(m, inst, routes_by_s):
    P, S = inst["P"], inst["S"]

    PS = P + S
    ps_pos = {node: idx for idx, node in enumerate(PS)}

    def ps_local_12(node):
        # 仓库 1..|P|, 卫星 |P|+1..|P|+|S|
        return ps_pos[node] + 1

    first_seq = []
    Vidx = sorted({
        int(v.VarName.split('[')[1].split(']')[0])
        for v in m.getVars() if v.VarName.startswith('tau[')
    })

    # ---------- 一层路径提取 ----------
    for v in Vidx:
        tv = m.getVarByName(f"tau[{v}]")
        if tv is None or tv.X < 0.5:
            continue

        # 1) 找起点仓库 p0：看 y_dep[p,v]
        p0 = None
        for p in P:
            yv = m.getVarByName(f"y_dep[{p},{v}]")
            if yv is not None and yv.X > 0.5:
                p0 = p
                break
        if p0 is None:
            continue

        # 2) 找第一个访问的卫星 s1：看 x[p0,s,v]
        s1 = None
        for s in S:
            xv = m.getVarByName(f"x[{p0},{s},{v}]")
            if xv is not None and xv.X > 0.5:
                s1 = s
                break
        if s1 is None:
            continue

        first_seq += [ps_local_12(p0), ps_local_12(s1)]

        # 3) 继续在卫星之间按 x[s_cur,s_next,v] 拓展
        cur = s1
        visitedS = {cur}
        while True:
            nxt = None
            for s in S:
                xv = m.getVarByName(f"x[{cur},{s},{v}]")
                if xv is not None and xv.X > 0.5:
                    nxt = s
                    break
            if nxt is None:
                break
            first_seq.append(ps_local_12(nxt))
            if nxt in visitedS:
                break
            visitedS.add(nxt)
            cur = nxt

    first_str = " ".join(map(str, first_seq)) if first_seq else ""

    # ---------- 二层路径提取（增加“返回最近站点”） ----------
    num_dep, num_sat = inst["num_dep"], inst["num_sat"]
    dist = inst["dist"]

    def sat_local_12(s):
        """卫星局部编号：1..|S|"""
        return (s - num_dep) + 1

    def cus_local_12(z):
        """客户局部编号：|S|+1..|S|+|Z|"""
        return num_sat + (z - (num_dep + num_sat)) + 1

    second_list = []
    for s in S:
        for r in range(len(routes_by_s[s])):
            gv = m.getVarByName(f"gamma[{s},{r}]")
            if gv is None or gv.X is None or gv.X <= 0.5:
                continue

            route = routes_by_s[s][r]  # 这是一个客户节点序列（全局编号）

            if len(route) == 0:
                continue

            # 起点卫星
            second_list.append(sat_local_12(s))
            # 依次输出该子回路内的客户
            for z in route:
                second_list.append(cus_local_12(z))

            # ---- 新增：最后一个客户返回最近卫星 ----
            last_z = route[-1]
            best_sat = None
            best_d = float("inf")
            for s2 in S:
                d2 = dist[last_z, s2]
                if d2 < best_d:
                    best_d = d2
                    best_sat = s2

            if best_sat is None:
                best_sat = s  # 极端兜底（理论上不会发生）

            second_list.append(sat_local_12(best_sat))

    second_str = " ".join(map(str, second_list)) if second_list else ""

    return first_str, second_str



def mip_progress_cb(model, where):
    if where == GRB.Callback.MIP:
        pass


def solve_instance(csv_path, w1=1.0, w2=1.0):
    t0 = time.perf_counter()
    inst = read_instance(csv_path)

    # Exact benchmarking: build a (near-)full route pool by exhaustive enumeration.
    # Note: this is only practical for small instances.
    routes_by_s, energy_by_sr, cover_mat, deliv_mat = build_pool_exact(inst)

    # Pool statistics
    pool_cols = sum(len(routes_by_s[s]) for s in routes_by_s)
    print(f"[pool] total drone-route columns = {pool_cols}")

    miss = warn_uncovered_customers(inst, cover_mat, routes_by_s)
    if miss and not USE_DUMMY_COLUMNS:

        return {"file": os.path.basename(csv_path), "status": "Unsolvable"}

    m, gamma, cover_constr, supply_constr = build_master(inst, routes_by_s, energy_by_sr, cover_mat, deliv_mat, as_lp=True, w1=w1, w2=w2)
    m.Params.Method = 1
    m.Params.OutputFlag = OUTPUT_LOG
    m.Params.TimeLimit  = TIME_LIMIT
    m.optimize()
    if m.Status not in (GRB.OPTIMAL, GRB.SUBOPTIMAL):
        return {"file": os.path.basename(csv_path), "status": "Unsolvable"}
    lp_val = float(m.ObjVal)

    for rnd in range(1, PRICING_ROUNDS+1):
        added = pricing_round(m, routes_by_s, energy_by_sr, cover_mat, deliv_mat, cover_constr, supply_constr)
        if added == 0: break
        m.optimize()
        if m.Status not in (GRB.OPTIMAL, GRB.SUBOPTIMAL):
            break
        lp_val = float(m.ObjVal)

    for v in m.getVars():
        n = v.VarName
        if (n.startswith("gamma[") or n.startswith("x[") or
                n.startswith("tau[") or n.startswith("y_dep[")):
            v.VType = GRB.BINARY
    m.update()

    m.Params.Presolve   = 2
    m.Params.Cuts       = 2
    m.Params.Heuristics = 0.2
    m.Params.MIPFocus   = 3
    m.Params.MIPGap     = 0.0
    m.Params.TimeLimit  = 3600
    m.Params.NodefileStart = 0.5
    m.optimize(mip_progress_cb)

    if m.SolCount == 0 or m.Status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD):
        return {"file": os.path.basename(csv_path), "status": "Unsolvable"}

    proven_optimal = (m.Status == GRB.OPTIMAL) and (abs(float(m.ObjBound) - float(m.ObjVal)) <= 1e-6)
    solve_status = int(m.Status)
    solve_status_str = {
        GRB.OPTIMAL: 'OPTIMAL',
        GRB.TIME_LIMIT: 'TIME_LIMIT',
        GRB.INFEASIBLE: 'INFEASIBLE',
        GRB.INF_OR_UNBD: 'INF_OR_UNBD',
        GRB.INTERRUPTED: 'INTERRUPTED',
        GRB.SUBOPTIMAL: 'SUBOPTIMAL',
    }.get(m.Status, str(m.Status))

    print(f"[mip] status={solve_status_str}, ObjVal={m.ObjVal}, ObjBound={m.ObjBound}, "
          f"MIPGap={getattr(m, 'MIPGap', float('nan'))}, proven_optimal={proven_optimal}")

    print("[check] rows, cols, nz =", m.NumConstrs, m.NumVars, m.NumNZs)

    # 计算 obj1 和 obj2
    dist = inst["dist"]
    obj1 = 0.0  # f2：固定成本
    obj2 = 0.0  # f1：行驶成本 + 能耗成本
    for var in m.getVars():
        name = var.VarName
        val = var.X
        if abs(val) < 1e-8:
            continue
        if name.startswith("x["):
            inside = name[2:-1]
            i_str, j_str, v_str = inside.split(",")
            i = int(i_str)
            j = int(j_str)
            obj2 += c_v * dist[i, j] * val
        elif name.startswith("gamma["):
            inside = name[6:-1]
            s_str, r_str = inside.split(",")
            s = int(s_str)
            r = int(r_str)
            E = energy_by_sr.get((s, r), 0.0)
            obj2 += c_u * E * val
            obj1 += h2 * val
        elif name.startswith("tau["):
            v_str = name[4:-1]
            obj1 += h1 * val

    sum_obj = obj2 + obj1
    weighted_obj = float(w1)*obj1 + float(w2)*obj2
    weighted_obj_norm = weighted_obj / max(1e-12, (float(w1)+float(w2)))
    gap = float(m.MIPGap) if hasattr(m, "MIPGap") else max(0.0, (m.ObjBound - m.ObjVal)/max(1e-9,abs(m.ObjVal)))

    first_str, second_str = extract_paths(m, inst, routes_by_s)
    elapsed = time.perf_counter() - t0
    return {
        "file": os.path.basename(csv_path),
        "time": round(elapsed, 2),
        "first": first_str,
        "second": second_str,
        "obj1": round(obj1, 6),
        "obj2": round(obj2, 6),
        "sum_obj": round(sum_obj, 6),
        "w1": float(w1),
        "w2": float(w2),
        "weighted_obj": round(weighted_obj, 6),
        "weighted_obj_norm": round(weighted_obj_norm, 6),
        "lp": round(lp_val, 6),
        "gap": round(gap, 6),
        "status": "OK"
    }


def run_batch():
    Set1 = ['Ca1-2,3,15','Ca1-3,5,15','Ca1-6,4,15','Ca2-2,3,15','Ca2-3,5,15','Ca2-6,4,15',
            'Ca3-2,3,15','Ca3-3,5,15','Ca3-6,4,15','Ca4-2,3,15','Ca4-3,5,15','Ca4-6,4,15',
            'Ca5-2,3,15','Ca5-3,5,15','Ca5-6,4,15']
    Set2 = ['Ca1-2,3,30', 'Ca1-3,5,30', 'Ca1-6,4,30', 'Ca2-2,3,30', 'Ca2-3,5,30', 'Ca2-6,4,30',
            'Ca3-2,3,30', 'Ca3-3,5,30', 'Ca3-6,4,30', 'Ca4-2,3,30', 'Ca4-3,5,30', 'Ca4-6,4,30',
            'Ca5-2,3,30', 'Ca5-3,5,30', 'Ca5-6,4,30']

    Set3 = [
            'Ca1-2,3,50','Ca1-3,5,50','Ca1-6,4,50','Ca2-2,3,50','Ca2-3,5,50','Ca2-6,4,50',
            'Ca3-2,3,50','Ca3-3,5,50','Ca3-6,4,50','Ca4-2,3,50','Ca4-3,5,50','Ca4-6,4,50',
            'Ca5-2,3,50','Ca5-3,5,50','Ca5-6,4,50']
    Set4 = [
            'Ca1-2,3,100','Ca1-3,5,100','Ca1-6,4,100','Ca2-2,3,100','Ca2-3,5,100','Ca2-6,4,100',
            'Ca3-2,3,100','Ca3-3,5,100','Ca3-6,4,100','Ca4-2,3,100','Ca4-3,5,100','Ca4-6,4,100',
            'Ca5-2,3,100','Ca5-3,5,100','Ca5-6,4,100',]

    all_sets = [(1, Set1),(2, Set2),(3, Set3),(4, Set4)]

    for set_id, name_list in all_sets:
        rows = []
        for name in name_list:
            csv_path = os.path.join(BASE_DIR, name + ".csv")
            # 9 weight settings: (1,9),(2,8),...,(9,1)
            weight_pairs = [(i, 10 - i) for i in range(1, 10)]

            for (w1, w2) in weight_pairs:
                try:
                    res = solve_instance(csv_path, w1=w1, w2=w2)
                    if res.get("status") == "OK":
                        rows.append([
                            res["file"],
                            w1, w2,
                            res["time"],
                            res["first"],
                            res["second"],
                            res["obj1"],
                            res["obj2"],
                            res["sum_obj"],
                            res["weighted_obj"],
                            res["weighted_obj_norm"],
                            res["lp"],
                            res["gap"],
                        ])
                    else:
                        rows.append([os.path.basename(csv_path), w1, w2] + ["Unsolvable"]*10)
                except Exception:
                    rows.append([os.path.basename(csv_path), w1, w2] + ["Unsolvable"]*10)

        dfout = pd.DataFrame(rows, columns=[
            "Instances", "w1", "w2", "Time(s)", "First-echelon", "Second-echelon",
            "obj1", "obj2", "Sum_obj", "Weighted_obj", "Weighted_obj_norm", "LP Value", "Gap"
        ])
        out_xlsx = os.path.join(BASE_DIR, f"WeightedSweep_Set{set_id}_results.xlsx")
        safe_save_table(dfout, out_xlsx)

if __name__ == "__main__":
    run_batch()