"""Experiment driver: HSD vs. four baselines over the scenario chain
A_normal -> B1_incident -> B2_surge -> C_recovery -> D_reshock.

Every method is warm-started from its own previous-scenario state, targets the
same relative full-space gap (EPS), and is scored on iterations, shortest-path
calls, master variables, link-flow RMSE against an exact full-path-space UE
reference, and CPU time.

Outputs (results/):
    comparison.csv        per-scenario metric table, all methods
    hsd_rounds.csv        HSD per-round trace (Z, gap, K, actions)
    atom_features.csv     per-atom feature vectors z_j (gradient-network data)
    summary.md            tables + narrative + answers to the five questions
    fig_convergence_A.png cold-start convergence, scenario A, all methods
    fig_hsd_lifecycle.png gap + frontier size across the whole scenario chain
"""

import csv
import io
import os

import numpy as np

from baselines import flat_sd, frank_wolfe, gradient_projection, reference_ue
from hsd import HSD, HSDParams
from network import PathRegistry
from scenarios import DEST, ORIGIN, SCENARIOS, apply_scenario, build

EPS = 5e-3
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")


# ----------------------------------------------------------------- reference
def compute_references():
    refs = {}
    for scen in SCENARIOS:
        net, _, _, _ = build()
        q = apply_scenario(net, scen)
        reg = PathRegistry(net)                 # private registry: full path space
        refs[scen["name"]] = reference_ue(net, reg, ORIGIN, DEST, q)
    return refs


def rmse(x, x_ref):
    return float(np.sqrt(np.mean((x - x_ref) ** 2)))


# ------------------------------------------------------------------- chains
def run_hsd_chain(log):
    net, reg, _, atoms = build()
    solver = HSD(net, reg, ORIGIN, DEST, HSDParams(eps_full=EPS))
    solver.install(atoms)
    per_scen = {}
    for scen in SCENARIOS:
        q = apply_scenario(net, scen)
        before = dict(solver.counters)
        res = solver.run(q, scenario=scen["name"], log=log)
        acts = {k: solver.counters[k] - before[k] for k in before}
        recs = [r for r in solver.history if r.scenario == scen["name"]]
        # rounds actually needed to certify (first round with gap <= EPS)
        to_cert = next((i + 1 for i, r in enumerate(recs) if r.rel_gap <= EPS),
                       len(recs))
        per_scen[scen["name"]] = {
            "method": "HSD (ours)", "x": res["x"], "rel_gap": res["rel_gap"],
            "iters": to_cert, "sp_calls": acts["sp_calls"],
            "rmp_solves": acts["rmp_solves"],
            "vars": len(solver.frontier),
            "peak_vars": max(r.frontier_size for r in recs),
            "cpu_s": res["cpu_s"],
            "actions": ", ".join(
                f"{k}:{v}" for k, v in acts.items()
                if v and k not in ("sp_calls", "rmp_solves", "rmp_iters")) or "-",
            "gap_history": [r.rel_gap for r in recs],
        }
    return solver, per_scen


def run_fw_chain():
    per_scen = {}
    net, _, _, _ = build()
    x, q_prev = None, None
    for scen in SCENARIOS:
        q = apply_scenario(net, scen)
        x0 = None if x is None else x * (q / q_prev)
        r = frank_wolfe(net, ORIGIN, DEST, q, eps_rel=EPS, x0=x0)
        x, q_prev = r["x"], q
        r["method"], r["actions"], r["gap_history"] = "Frank-Wolfe", "-", r["history"]
        per_scen[scen["name"]] = r
    return per_scen


def run_gp_chain():
    per_scen = {}
    net, _, _, _ = build()
    reg = PathRegistry(net)
    flows = None
    for scen in SCENARIOS:
        q = apply_scenario(net, scen)
        r = gradient_projection(net, reg, ORIGIN, DEST, q, eps_rel=EPS,
                                warm_flows=flows)
        flows = r["flows"]
        r["method"], r["actions"], r["gap_history"] = \
            "Gradient projection", "-", r["history"]
        per_scen[scen["name"]] = r
    return per_scen


def run_sd_chain(aggregated):
    """aggregated=False: flat SD over primitive paths (baseline 3).
    aggregated=True: initial frozen aggregated columns h_A, h_B (baseline 4)."""
    per_scen = {}
    net, reg, _, atoms = build()
    cols = None
    lam = None
    if aggregated:
        cols = [(a.column(reg), a.name, None) for a in atoms]
    for scen in SCENARIOS:
        q = apply_scenario(net, scen)
        r = flat_sd(net, reg, ORIGIN, DEST, q, eps_rel=EPS,
                    init_columns=cols, warm_lam=lam)
        cols, lam = r["columns"], r["lam"]
        r["method"] = "Flat SD (aggregated init)" if aggregated else "Flat SD (paths)"
        r["actions"], r["gap_history"] = "-", r["history"]
        per_scen[scen["name"]] = r
    return per_scen


# ------------------------------------------------------------------- tables
COLUMNS = ("method", "iters", "rmp_solves", "sp_calls", "peak_vars", "vars",
           "rel_gap", "rmse", "cpu_ms", "actions")


def make_table(refs, chains):
    rows = []
    for scen in SCENARIOS:
        name = scen["name"]
        for chain in chains:
            r = chain[name]
            rows.append({
                "scenario": name, "method": r["method"], "iters": r["iters"],
                "rmp_solves": r["rmp_solves"], "sp_calls": r["sp_calls"],
                "peak_vars": r["peak_vars"], "vars": r["vars"],
                "rel_gap": r["rel_gap"], "rmse": rmse(r["x"], refs[name]["x"]),
                "cpu_ms": r["cpu_s"] * 1000,
                "actions": r["actions"],
            })
    return rows


def format_table(rows, scen_name):
    buf = io.StringIO()
    sub = [r for r in rows if r["scenario"] == scen_name]
    hdr = (f"{'method':26s} {'iters':>5s} {'RMP':>4s} {'SP':>4s} "
           f"{'peakV':>5s} {'vars':>4s} {'rel gap':>9s} {'flow RMSE':>10s} "
           f"{'ms':>7s}  structural actions")
    buf.write(hdr + "\n" + "-" * len(hdr) + "\n")
    for r in sub:
        buf.write(f"{r['method']:26s} {r['iters']:5d} {r['rmp_solves']:4d} "
                  f"{r['sp_calls']:4d} {r['peak_vars']:5d} {r['vars']:4d} "
                  f"{r['rel_gap'] * 100:8.4f}% "
                  f"{r['rmse']:10.4f} {r['cpu_ms']:7.1f}  {r['actions']}\n")
    return buf.getvalue()


# -------------------------------------------------------------------- plots
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASE = "#c3c2b7"
SURFACE = "#fcfcfb"
SERIES = {  # categorical slots, fixed order (validated palette)
    "HSD (ours)": "#2a78d6",
    "Frank-Wolfe": "#eb6834",
    "Gradient projection": "#1baf7a",
    "Flat SD (paths)": "#eda100",
    "Flat SD (aggregated init)": "#e87ba4",
}


def _style_axis(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASE)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.8, alpha=0.9)
    ax.set_axisbelow(True)


def plot_convergence(chains, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8.2, 4.6), dpi=180)
    fig.patch.set_facecolor(SURFACE)
    _style_axis(ax)
    floor = EPS / 50
    for chain in chains:
        r = chain["A_normal"]
        g = np.maximum(np.array(r["gap_history"], dtype=float), floor)
        it = np.arange(1, len(g) + 1)
        ax.plot(it, g * 100, color=SERIES[r["method"]], linewidth=2,
                marker="o", markersize=3.5, markeredgecolor=SURFACE,
                markeredgewidth=0.6, label=r["method"], clip_on=False)
        ax.annotate(r["method"], (it[-1], g[-1] * 100),
                    xytext=(6, 0), textcoords="offset points",
                    fontsize=8.5, color=INK2, va="center")
    ax.axhline(EPS * 100, color=MUTED, linewidth=1, linestyle=(0, (4, 3)))
    ax.annotate(f"certificate ε = {EPS * 100:.1f}%", (0.99, EPS * 100),
                xycoords=("axes fraction", "data"), xytext=(0, 5),
                textcoords="offset points", ha="right", fontsize=8.5,
                color=MUTED)
    ax.set_yscale("log")
    ax.set_xlabel("iteration (one exact shortest-path pricing each)",
                  color=INK2, fontsize=10)
    ax.set_ylabel("relative full-space gap (%)", color=INK2, fontsize=10)
    ax.set_title("Scenario A (cold start): certified convergence by method",
                 color=INK, fontsize=12, loc="left", pad=12)
    ax.legend(loc="upper right", fontsize=8.5, frameon=False,
              labelcolor=INK2)
    ax.set_xlim(left=1)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def plot_lifecycle(solver, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    recs = solver.history
    rounds = np.arange(1, len(recs) + 1)
    gaps = np.maximum(np.array([r.rel_gap for r in recs]), EPS / 50)
    K = np.array([r.frontier_size for r in recs])
    scen_of = [r.scenario for r in recs]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.6, 6.2), dpi=180,
                                   sharex=True,
                                   gridspec_kw={"height_ratios": [1.4, 1]})
    fig.patch.set_facecolor(SURFACE)
    for ax in (ax1, ax2):
        _style_axis(ax)

    # scenario bands
    bounds = [1]
    for i in range(1, len(recs)):
        if scen_of[i] != scen_of[i - 1]:
            bounds.append(i + 1)
    bounds.append(len(recs) + 1)
    for k in range(len(bounds) - 1):
        if k % 2 == 1:
            for ax in (ax1, ax2):
                ax.axvspan(bounds[k] - 0.5, bounds[k + 1] - 0.5,
                           color=GRID, alpha=0.35, linewidth=0)
        ax1.annotate(scen_of[bounds[k] - 1],
                     ((bounds[k] + bounds[k + 1] - 1) / 2, 1.02),
                     xycoords=("data", "axes fraction"), ha="center",
                     fontsize=8.5, color=INK2)

    blue = SERIES["HSD (ours)"]
    ax1.plot(rounds, gaps * 100, color=blue, linewidth=2, marker="o",
             markersize=3.2, markeredgecolor=SURFACE, markeredgewidth=0.6)
    ax1.axhline(EPS * 100, color=MUTED, linewidth=1, linestyle=(0, (4, 3)))
    ax1.annotate(f"ε = {EPS * 100:.1f}%", (0.995, EPS * 100),
                 xycoords=("axes fraction", "data"), xytext=(0, 4),
                 textcoords="offset points", ha="right", fontsize=8.5,
                 color=MUTED)
    ax1.set_yscale("log")
    ax1.set_ylabel("relative full-space gap (%)", color=INK2, fontsize=10)
    ax1.set_title("HSD lifecycle over the scenario chain: certificate (top) "
                  "and frontier size with structural actions (bottom)",
                  color=INK, fontsize=12, loc="left", pad=22)

    ax2.step(rounds, K, where="post", color=blue, linewidth=2)
    ax2.set_ylabel("frontier size K", color=INK2, fontsize=10)
    ax2.set_xlabel("HSD round (global, warm-started across scenarios)",
                   color=INK2, fontsize=10)
    ax2.set_ylim(0, K.max() + 1.6)
    ax2.set_yticks(range(0, int(K.max()) + 2))
    tags = {"generate": "G", "promote": "P", "split": "S", "merge": "M",
            "demote": "D", "drop": "×"}
    for i, r in enumerate(recs):
        if r.actions == "-":
            continue
        tag = next((v for k, v in tags.items() if r.actions.startswith(k)), "?")
        ax2.annotate(tag, (rounds[i], K[i] + 0.55), ha="center", fontsize=9,
                     color=INK, fontweight="bold")
    ax2.annotate("G generate   P promote   S split   M merge   × drop",
                 (0.01, 0.94), xycoords="axes fraction", fontsize=8.5,
                 color=MUTED, va="top")
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


# ------------------------------------------------------------------ summary
def write_summary(refs, rows, solver, chains):
    lines = ["# HSD single-OD prototype - experiment summary", ""]
    lines.append(f"Certificate target: relative full-space gap <= {EPS * 100:.1f}% "
                 "(same for every method). All methods warm-start from their own "
                 "previous-scenario state. Reference flows: exact UE over the "
                 "full enumerated path space.")
    lines.append("")
    for scen in SCENARIOS:
        name = scen["name"]
        lines.append(f"## Scenario {name} (q={scen['q']:.0f}, "
                     f"cap overrides {scen['cap_overrides'] or 'none'})")
        lines.append("")
        lines.append("```")
        lines.append(format_table(rows, name).rstrip())
        lines.append("```")
        lines.append("")
    lines.append("## HSD round-by-round narrative")
    lines.append("")
    lines.append("```")
    for r in solver.history:
        lines.append(f"[{r.scenario:12s}] round {r.round:3d}  gap="
                     f"{r.rel_gap * 100:8.4f}%  K={r.frontier_size}  {r.actions}")
    lines.append("```")
    lines.append("")
    lines.append("## Final frontier")
    lines.append("")
    for a, l in zip(solver.frontier, solver.lam):
        lines.append(f"- lambda = {l:.4f}  `{a!r}`")
    lines.append("")
    with open(os.path.join(OUT, "summary.md"), "w") as f:
        f.write("\n".join(lines))


def main():
    os.makedirs(OUT, exist_ok=True)
    print("computing exact full-path-space UE references ...")
    refs = compute_references()
    for name, ref in refs.items():
        print(f"  {name:12s} Z={ref['Z']:.4f}  rgap={ref['rgap_rel']:.2e}")

    print("\n=== HSD chain ===")
    solver, hsd_res = run_hsd_chain(log=print)
    print("\n=== baselines ===")
    fw_res = run_fw_chain()
    gp_res = run_gp_chain()
    sd_res = run_sd_chain(aggregated=False)
    agg_res = run_sd_chain(aggregated=True)

    chains = [hsd_res, fw_res, gp_res, sd_res, agg_res]
    rows = make_table(refs, chains)
    for scen in SCENARIOS:
        print(f"\nScenario {scen['name']} (q={scen['q']:.0f}, "
              f"overrides {scen['cap_overrides'] or 'none'})")
        print(format_table(rows, scen["name"]))

    # ---------------------------------------------------------------- files
    with open(os.path.join(OUT, "comparison.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["scenario"] + list(COLUMNS))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(OUT, "hsd_rounds.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "round", "Z", "rel_gap", "frontier_size",
                    "explicit_paths", "represented_paths", "rmp_iters",
                    "actions"])
        for r in solver.history:
            w.writerow([r.scenario, r.round, f"{r.Z:.6f}", f"{r.rel_gap:.3e}",
                        r.frontier_size, r.explicit_paths, r.represented_paths,
                        r.rmp_iters, r.actions])
    with open(os.path.join(OUT, "atom_features.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(solver.atom_feature_log[0]))
        w.writeheader()
        w.writerows(solver.atom_feature_log)
    write_summary(refs, rows, solver, chains)
    plot_convergence(chains, os.path.join(OUT, "fig_convergence_A.png"))
    plot_lifecycle(solver, os.path.join(OUT, "fig_hsd_lifecycle.png"))
    print(f"\nwrote results to {OUT}/")


if __name__ == "__main__":
    main()
