"""Scaling benchmark: HSD vs. flat SD / frozen aggregation / GP / FW on
multi-corridor networks of growing size, over the repeated-scenario chain
N_normal -> I_incident -> R_recovery (warm-started, same certificate).

The four headline metrics, per stage and cumulative:
    outer     outer iterations to reach the certificate
    RMP       restricted-master solves (HSD/SD; 0 for FW/GP)
    SP        exact shortest-path pricing calls
    peakV     peak number of master variables (columns) ever held

The point being measured: as the path space grows exponentially (C * 2^m
primitive paths), a frontier of well-proportioned latent items stays at
O(C) columns and only expands around the incident, while flat SD must keep
accumulating primitive columns and never shrinks again.

Run:  python3 benchmark_grid.py            (takes ~a minute at the XL scale)
"""

import csv
import os

import numpy as np

from baselines import flat_sd, frank_wolfe, gradient_projection
from grid import (apply_grid_scenario, build_corridor_atoms,
                  build_corridor_network, grid_scenarios)
from hsd import HSD, HSDParams
from network import PathRegistry

EPS = 1e-3
SCALES = [(3, 3), (4, 5), (6, 7), (8, 9)]      # (corridors C, diamonds m)
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")

GRID_PARAMS = dict(eps_full=EPS, tau_flow=0.25, tau_minor=5e-3,
                   stable_rounds=3, cooldown=2, quiet_rounds=8)


def _stage_row(scale, stage, method, iters, rmp, sp, peak, final, gap, cpu_s):
    return {"scale": scale, "stage": stage, "method": method, "outer": iters,
            "rmp_solves": rmp, "sp_calls": sp, "peak_vars": peak,
            "final_vars": final, "rel_gap": gap, "cpu_ms": cpu_s * 1000}


def bench_hsd(net, meta, scens):
    reg = PathRegistry(net)
    atoms = build_corridor_atoms(net, reg, meta)
    solver = HSD(net, reg, meta["origin"], meta["dest"],
                 HSDParams(**GRID_PARAMS))
    solver.install(atoms)
    rows = []
    scale = f"C{meta['C']}m{meta['m']}"
    for scen in scens:
        q = apply_grid_scenario(net, scen)
        before = dict(solver.counters)
        res = solver.run(q, scenario=scen["name"])
        recs = [r for r in solver.history if r.scenario == scen["name"]]
        to_cert = next((i + 1 for i, r in enumerate(recs)
                        if r.rel_gap <= EPS), len(recs))
        rows.append(_stage_row(
            scale, scen["name"], "HSD (ours)", to_cert,
            solver.counters["rmp_solves"] - before["rmp_solves"],
            solver.counters["sp_calls"] - before["sp_calls"],
            max(r.frontier_size for r in recs), len(solver.frontier),
            res["rel_gap"], res["cpu_s"]))
    return rows, solver


def bench_sd(net, meta, scens, aggregated):
    reg = PathRegistry(net)
    cols = None
    if aggregated:
        atoms = build_corridor_atoms(net, reg, meta)
        cols = [(a.column(reg), a.name, None) for a in atoms]
    lam = None
    rows = []
    scale = f"C{meta['C']}m{meta['m']}"
    name = "Flat SD (aggregated init)" if aggregated else "Flat SD (paths)"
    for scen in scens:
        q = apply_grid_scenario(net, scen)
        r = flat_sd(net, reg, meta["origin"], meta["dest"], q, eps_rel=EPS,
                    init_columns=cols, warm_lam=lam, max_rounds=400)
        cols, lam = r["columns"], r["lam"]
        rows.append(_stage_row(scale, scen["name"], name, r["iters"],
                               r["rmp_solves"], r["sp_calls"], r["peak_vars"],
                               r["vars"], r["rel_gap"], r["cpu_s"]))
    return rows


def bench_gp(net, meta, scens):
    reg = PathRegistry(net)
    flows = None
    rows = []
    scale = f"C{meta['C']}m{meta['m']}"
    for scen in scens:
        q = apply_grid_scenario(net, scen)
        r = gradient_projection(net, reg, meta["origin"], meta["dest"], q,
                                eps_rel=EPS, warm_flows=flows, max_iter=5000)
        flows = r["flows"]
        rows.append(_stage_row(scale, scen["name"], "Gradient projection",
                               r["iters"], 0, r["sp_calls"], r["peak_vars"],
                               r["vars"], r["rel_gap"], r["cpu_s"]))
    return rows


def bench_fw(net, meta, scens):
    x, q_prev = None, None
    rows = []
    scale = f"C{meta['C']}m{meta['m']}"
    for scen in scens:
        q = apply_grid_scenario(net, scen)
        x0 = None if x is None else x * (q / q_prev)
        r = frank_wolfe(net, meta["origin"], meta["dest"], q, eps_rel=EPS,
                        x0=x0, max_iter=8000)
        x, q_prev = r["x"], q
        rows.append(_stage_row(scale, scen["name"], "Frank-Wolfe", r["iters"],
                               0, r["sp_calls"], 0, 0, r["rel_gap"],
                               r["cpu_s"]))
    return rows


def format_stage_table(rows, scale, n_paths):
    out = [f"scale {scale}  ({n_paths} primitive paths)"]
    hdr = (f"  {'stage':16s} {'method':26s} {'outer':>5s} {'RMP':>4s} "
           f"{'SP':>5s} {'peakV':>5s} {'endV':>4s} {'rel gap':>9s} {'ms':>8s}")
    out.append(hdr)
    out.append("  " + "-" * (len(hdr) - 2))
    for r in rows:
        out.append(f"  {r['stage']:16s} {r['method']:26s} {r['outer']:5d} "
                   f"{r['rmp_solves']:4d} {r['sp_calls']:5d} "
                   f"{r['peak_vars']:5d} {r['final_vars']:4d} "
                   f"{r['rel_gap'] * 100:8.4f}% {r['cpu_ms']:8.1f}")
    return "\n".join(out)


def cumulative(rows):
    agg = {}
    for r in rows:
        key = (r["scale"], r["method"])
        a = agg.setdefault(key, {"scale": r["scale"], "method": r["method"],
                                 "outer": 0, "rmp_solves": 0, "sp_calls": 0,
                                 "peak_vars": 0, "final_vars": 0,
                                 "cpu_ms": 0.0})
        a["outer"] += r["outer"]
        a["rmp_solves"] += r["rmp_solves"]
        a["sp_calls"] += r["sp_calls"]
        a["peak_vars"] = max(a["peak_vars"], r["peak_vars"])
        a["final_vars"] = r["final_vars"]
        a["cpu_ms"] += r["cpu_ms"]
    return list(agg.values())


# -------------------------------------------------------------------- plots
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID_C = "#e1e0d9"
BASE = "#c3c2b7"
SURFACE = "#fcfcfb"
SERIES = {
    "HSD (ours)": "#2a78d6",
    "Frank-Wolfe": "#eb6834",
    "Gradient projection": "#1baf7a",
    "Flat SD (paths)": "#eda100",
    "Flat SD (aggregated init)": "#e87ba4",
}


def plot_scaling(cum, sizes, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.4, 4.4), dpi=180)
    fig.patch.set_facecolor(SURFACE)
    panels = [(ax1, "peak_vars", "peak master variables"),
              (ax2, "sp_calls", "total shortest-path calls")]
    methods = [m for m in SERIES if any(c["method"] == m for c in cum)]
    for ax, key, title in panels:
        ax.set_facecolor(SURFACE)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(BASE)
        ax.tick_params(colors=MUTED, labelsize=9)
        ax.grid(True, color=GRID_C, linewidth=0.8, alpha=0.9)
        ax.set_axisbelow(True)
    seen = {}
    for ax, key, title in panels:
        for m in methods:
            pts = [(sizes[c["scale"]], c[key]) for c in cum
                   if c["method"] == m and c[key] > 0]
            if not pts:
                continue
            xs, ys = zip(*sorted(pts))
            line, = ax.plot(xs, ys, color=SERIES[m], linewidth=2, marker="o",
                            markersize=4, markeredgecolor=SURFACE,
                            markeredgewidth=0.7, label=m, clip_on=False)
            seen.setdefault(m, line)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel("primitive paths in the network", color=INK2, fontsize=10)
        ax.set_title(title, color=INK, fontsize=11, loc="left", pad=10)
    fig.legend(seen.values(), seen.keys(), loc="lower center",
               ncol=len(seen), fontsize=8.5, frameon=False, labelcolor=INK2)
    fig.suptitle("Whole repeated-shock chain (normal + 3x incident/recovery), "
                 f"certificate {EPS * 100:.1f}%", color=INK2, fontsize=10,
                 x=0.01, ha="left", y=1.0)
    fig.tight_layout(rect=(0, 0.07, 1, 0.96))
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    all_rows, sizes = [], {}
    for C, m in SCALES:
        q = 100.0 * C
        net, meta = build_corridor_network(C, m, q)
        scens = grid_scenarios(meta)
        scale = f"C{C}m{m}"
        sizes[scale] = C * 2 ** m
        rows = []
        r_hsd, _ = bench_hsd(net, meta, scens)
        rows += r_hsd
        rows += bench_sd(net, meta, scens, aggregated=False)
        rows += bench_sd(net, meta, scens, aggregated=True)
        rows += bench_gp(net, meta, scens)
        rows += bench_fw(net, meta, scens)
        all_rows += rows
        print("\n" + format_stage_table(rows, scale, sizes[scale]))

    cum = cumulative(all_rows)
    print("\n=== cumulative over the whole chain ===")
    hdr = (f"{'scale':8s} {'method':26s} {'outer':>5s} {'RMP':>4s} "
           f"{'SP':>5s} {'peakV':>5s} {'endV':>4s} {'ms':>9s}")
    print(hdr + "\n" + "-" * len(hdr))
    for c in sorted(cum, key=lambda r: (sizes[r["scale"]], r["method"])):
        print(f"{c['scale']:8s} {c['method']:26s} {c['outer']:5d} "
              f"{c['rmp_solves']:4d} {c['sp_calls']:5d} {c['peak_vars']:5d} "
              f"{c['final_vars']:4d} {c['cpu_ms']:9.1f}")

    with open(os.path.join(OUT, "grid_benchmark.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0]))
        w.writeheader()
        w.writerows(all_rows)
    plot_scaling(cum, sizes, os.path.join(OUT, "fig_grid_scaling.png"))
    print(f"\nwrote {OUT}/grid_benchmark.csv and fig_grid_scaling.png")


if __name__ == "__main__":
    main()
