"""Baselines for the comparison in the write-up:

  1. Frank-Wolfe (link based)
  2. path-based gradient projection (reduced gradient on the demand simplex)
  3. flat Simplicial Decomposition over primitive path columns
  4. flat SD initialised with frozen aggregated columns (no split/promote)

All report the same relative full-space gap
    rel_gap = (x . t(x) - q c*) / (q c*)
so the certificates are directly comparable with HSD.
"""

import time

import numpy as np

from master import solve_rmp


def _rel_gap(x, t, q, cstar):
    return (float(x @ t) - q * cstar) / (q * cstar)


# ---------------------------------------------------------------- Frank-Wolfe
def frank_wolfe(net, origin, dest, q, eps_rel=5e-3, max_iter=5000, x0=None):
    t0 = time.perf_counter()
    sp_calls = 0
    if x0 is None:
        _, seq = net.shortest_path(net.link_times(np.zeros(net.num_links)),
                                   origin, dest)
        sp_calls += 1
        x = np.zeros(net.num_links)
        for e in seq:
            x[e] += q
    else:
        x = x0.copy()
    history = []
    rel = np.inf
    for it in range(1, max_iter + 1):
        t = net.link_times(x)
        cstar, seq = net.shortest_path(t, origin, dest)
        sp_calls += 1
        rel = _rel_gap(x, t, q, cstar)
        history.append(rel)
        if rel <= eps_rel:
            break
        y = np.zeros(net.num_links)
        for e in seq:
            y[e] += q
        d = y - x
        lo, hi = 0.0, 1.0                      # exact line search: bisect phi'(g)
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if float(d @ net.link_times(x + mid * d)) > 0:
                hi = mid
            else:
                lo = mid
        x = x + 0.5 * (lo + hi) * d
    return {"x": x, "rel_gap": rel, "iters": it, "sp_calls": sp_calls,
            "vars": net.num_links, "peak_vars": net.num_links, "rmp_solves": 0,
            "history": history, "cpu_s": time.perf_counter() - t0}


# ------------------------------------------------- path-based gradient projection
def gradient_projection(net, registry, origin, dest, q, eps_rel=5e-3,
                        max_iter=5000, warm_flows=None):
    """Jayakrishnan-style GP: pairwise reduced-gradient shifts toward the
    cheapest working path, scaled by second derivatives on the symmetric
    difference of links.  Demand conservation is maintained exactly."""
    t0c = time.perf_counter()
    sp_calls = 0
    flows = dict(warm_flows) if warm_flows else {}
    if not flows:
        _, seq = net.shortest_path(net.link_times(np.zeros(net.num_links)),
                                   origin, dest)
        sp_calls += 1
        flows[registry.add(seq)] = q
    else:                                       # rescale warm start to new demand
        s = sum(flows.values())
        flows = {p: f * q / s for p, f in flows.items()}
    history = []
    rel = np.inf
    for it in range(1, max_iter + 1):
        ids = sorted(flows)
        f = np.array([flows[p] for p in ids])
        x = registry.A[:, ids] @ f
        t = net.link_times(x)
        cstar, seq = net.shortest_path(t, origin, dest)
        sp_calls += 1
        pstar = registry.add(seq)
        if pstar not in flows:
            flows[pstar] = 0.0
            ids = sorted(flows)
            f = np.array([flows[p] for p in ids])
        rel = _rel_gap(x, t, q, cstar)
        history.append(rel)
        if rel <= eps_rel:
            break
        # Gauss-Seidel pairwise shifts: re-evaluate times after every shift so
        # shared congested links cannot be over-shifted by parallel updates
        for p in sorted(ids, key=lambda pp: -flows[pp]):
            t = net.link_times(x)
            c = registry.costs(t)
            b = ids[int(np.argmin([c[pp] for pp in ids]))]
            if p == b or c[p] <= c[b]:
                continue
            tp = net.link_time_derivs(x)
            diff = list(set(registry.link_seqs[p]) ^ set(registry.link_seqs[b]))
            s = float(np.sum(tp[diff])) + 1e-12
            shift = min(flows[p], (c[p] - c[b]) / s)
            if shift > 0:
                flows[p] -= shift
                flows[b] += shift
                x = x + shift * (registry.A[:, b] - registry.A[:, p])
    return {"flows": flows, "rel_gap": rel, "iters": it, "sp_calls": sp_calls,
            "vars": sum(1 for v in flows.values() if v > 1e-9),
            "peak_vars": len(flows), "rmp_solves": 0,
            "history": history, "cpu_s": time.perf_counter() - t0c,
            "x": registry.A[:, sorted(flows)] @ np.array(
                [flows[p] for p in sorted(flows)])}


# ------------------------------------------------------ flat SD (and aggregated)
def flat_sd(net, registry, origin, dest, q, eps_rel=5e-3, max_rounds=200,
            init_columns=None, warm_lam=None, eps_rmp=1e-11):
    """Simplicial decomposition over an explicit column list.

    init_columns: list of (column_vector, label, path_id_or_None).  Primitive
    path columns are tracked by path id so pricing does not re-add them;
    aggregated columns stay frozen forever (that is baseline 4's handicap).
    """
    t0c = time.perf_counter()
    sp_calls = 0
    cols, labels, path_of = [], [], []
    for col, lab, pid in (init_columns or []):
        cols.append(np.asarray(col, float))
        labels.append(lab)
        path_of.append(pid)
    if not cols:
        _, seq = net.shortest_path(net.link_times(np.zeros(net.num_links)),
                                   origin, dest)
        sp_calls += 1
        pid = registry.add(seq)
        cols.append(registry.A[:, pid].copy())
        labels.append(registry.names[pid])
        path_of.append(pid)
    lam = warm_lam
    history = []
    rel = np.inf
    peak = len(cols)
    for it in range(1, max_rounds + 1):
        H = np.column_stack(cols)
        peak = max(peak, H.shape[1])
        if lam is None or len(lam) != H.shape[1]:
            lam = np.full(H.shape[1], 1.0 / H.shape[1])
        lam, info = solve_rmp(net, H, q, lam0=lam, rel_tol=eps_rmp)
        x = q * (H @ lam)
        t = net.link_times(x)
        cstar, seq = net.shortest_path(t, origin, dest)
        sp_calls += 1
        rel = _rel_gap(x, t, q, cstar)
        history.append(rel)
        if rel <= eps_rel:
            break
        pid = registry.add(seq)
        if pid in path_of:
            continue                            # column present; RMP will fix it
        cols.append(registry.A[:, pid].copy())
        labels.append(registry.names[pid])
        path_of.append(pid)
        lam = np.append(lam, 0.0)
    return {"x": x, "rel_gap": rel, "iters": it, "sp_calls": sp_calls,
            "vars": len(cols), "peak_vars": peak, "rmp_solves": it,
            "labels": labels, "lam": lam,
            "columns": list(zip(cols, labels, path_of)),
            "history": history, "cpu_s": time.perf_counter() - t0c}


# --------------------------------------------------------- exact UE reference
def reference_ue(net, registry, origin, dest, q):
    """Exact UE over the *entire* enumerated path space (small network only).

    Solves the master over all primitive path columns to machine tolerance;
    used for link-flow error metrics."""
    for seq in net.all_simple_paths(origin, dest):
        registry.add(seq)
    H = registry.A.copy()
    lam, info = solve_rmp(net, H, q, rel_tol=1e-13, max_iter=200000)
    x = q * (H @ lam)
    return {"x": x, "Z": net.beckmann(x), "lam": lam,
            "rgap_rel": info["rgap_rel"]}
