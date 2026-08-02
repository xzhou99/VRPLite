# HSD Prototype — Hierarchical Simplicial Decomposition for single-OD traffic assignment

A deterministic, numpy-only research prototype of **Hierarchical Simplicial
Decomposition (HSD)** for the single-OD, single-commodity traffic assignment
problem, built around three named ideas:

| Name | Role |
|---|---|
| **Latent Flow Item** | concept — a hidden group of paths kept as one unit |
| **Feasible Composite Atom** | mathematical object — the latent item written as a fixed-proportion column `h_j = Σ_p π_{p|j} a_p`, `π ≥ 0`, `Σπ = 1` |
| **Hierarchical Simplicial Decomposition (HSD)** | algorithm — decides when a latent item stays hidden and when it is released |

The one-sentence core of the method:

> **HSD uses reduced gradients not only to adjust the coefficients of active
> columns, but also to decide whether the fixed internal proportions of a
> latent flow item should remain constrained or be released through
> hierarchical refinement.**

## The three principles (verified numerically in `verify_principles.py`)

1. **A latent item must first be a feasible composite atom.**
   With a master coefficient `λ_j` on the demand simplex, the implied path
   flows `f_p = q λ_j π_{p|j}` are nonnegative and conserve demand
   automatically. (`atoms.py::Atom`)
2. **All column adjustment uses reduced gradients.**
   The path gradient is the path cost `c_p = a_p^T t(x)`; the master gradient
   of an atom is `q·c̄_j`; movement is always pairwise against a reference
   column, so demand conservation is never violated. (`master.py::solve_rmp`)
3. **The hierarchy is controlled by reduced-gradient heterogeneity.**
   The within-atom reduced gradient `δ_{p|j} = c_p − c̄_j` and the
   heterogeneity `R_j = max_p c_p − min_p c_p` decide keep / promote / split —
   not geometric closeness. (`hsd.py`)

## Structural actions (all zero-disruption at the instant they apply)

| Action | Trigger | Effect on the frontier |
|---|---|---|
| **Generate** | full-space gap > ε and the priced path `p*` is not represented | add primitive column with `λ = 0` |
| **Promote** | `p*` hidden inside an atom, or `c̄_j − min_p c_p > τ_promote` | `{h_j} → {a_p, h_{j\p}}`, `λ` split as `(π_p λ_j, (1−π_p) λ_j)` |
| **Split** | `R_j > τ_split` and `λ_j > τ_weight` | partition `S_j` by the sign of `δ_{p|j}`; `λ` split by internal mass `α` |
| **Demote** | explicit path with `λ_p < τ_minor`, cost-close to a family, stable L rounds | fold back into the composite atom |
| **Merge** | `|c̄_i − c̄_j| < τ_cost` and `‖h_i − h_j‖_W < τ_flow` (W = diag t′(x)), stable L rounds | re-form a latent item **at the learned λ-ratio proportions** |

Hysteresis (split thresholds ≫ merge thresholds, cooldowns, stability
counters) prevents split→merge→split oscillation.  The congestion-weighted
distance `‖h_i − h_j‖_W` is a second-order estimate of the flexibility lost by
freezing the proportion between two atoms: immediate loss is zero, future loss
is what the reduced gradients monitor.

Exact shortest-path pricing (Dijkstra on the real link graph, **not** a scan
of enumerated paths) gives the full-space Frank–Wolfe gap
`g = q(Σ λ_j c̄_j − c_{p*})` as a certificate over the entire path space at
every round.

## Files

| File | Contents |
|---|---|
| `network.py` | BPR links, Beckmann objective, Dijkstra pricing, path registry |
| `atoms.py` | `Atom` + promote / split / merge operators + `‖·‖_W` distance |
| `master.py` | restricted master `min Z(qHλ)` on the simplex (projected gradient + Barzilai–Borwein; swap `solve_rmp` for CPLEX/scipy if desired — it is the single entry point) |
| `hsd.py` | the HSD loop: two-level reduced gradients, pricing, action policy, hysteresis, per-atom feature log `z_j` |
| `baselines.py` | Frank–Wolfe, path-based gradient projection (Gauss–Seidel), flat SD, flat aggregated SD, exact full-path-space UE reference |
| `scenarios.py` | Step-0 toy network: 2 corridor families × 2 paths + a bypass, shared congested exit; scenario chain A → B1 → B2 → C → D |
| `verify_principles.py` | numeric proof of the five claims (run this first) |
| `run_experiments.py` | toy-network comparison, all methods × 5 scenarios, plots |
| `grid.py` | scalable multi-corridor networks (`C · 2^m` paths), corridor-local-equilibrium latent items |
| `benchmark_grid.py` | scaling benchmark over a repeated-shock chain with the four headline metrics |

Run order:

```bash
python3 verify_principles.py   # 5 claims, all PASS
python3 run_experiments.py     # toy lifecycle + method comparison + figures
python3 benchmark_grid.py      # scaling benchmark (a few minutes)
```

Requires only `numpy` (+ `matplotlib` for figures).

## Headline metrics (why the hierarchy pays off)

Four counters, per stage and cumulative: **outer iterations**, **restricted
master solves**, **shortest-path pricing calls**, **peak master variables**.
Quiet rounds after a merge/demote cost nothing — merge is zero-disruption, so
the cached solution and certificate remain valid and no RMP/SP call is spent.

On the toy network the equilibrium ultimately uses all paths, so HSD can only
demonstrate the *mechanism* (as the flat-vs-HSD tie in the uploaded prototype
also showed).  The scaling benchmark is where the trade-off appears.  Corridor
networks with `C` corridors × `2^m` paths, over a repeated-shock chain
(normal, then 3 × incident/recovery in different corridors), certificate
0.1%, cumulative (`C8m9` = 4096 primitive paths):

| method | outer | RMP | SP | peak vars | end vars |
|---|--:|--:|--:|--:|--:|
| **HSD (ours)** | 27 | 27 | 27 | **13** | **8 (= C, fully re-merged)** |
| Flat SD (aggregated init) | 17 | 17 | 17 | 18 | 18 (grows every shock: 8→13→16→18) |
| Flat SD (paths) | 44 | 44 | 45 | 38 | 38 |
| Gradient projection | 67 | – | 68 | 63 | 58 |
| Frank–Wolfe | 1185 | – | 1186 | – | – |

The pattern that matters: methods that can only *add* master variables grow
monotonically across repeated scenarios; HSD expands only the affected branch
during each incident and **breathes back to `K = C`** after each recovery.

## The five phase-1 questions, answered empirically

1. **When are fixed internal proportions valid?**  When the within-atom
   reduced gradients are small (`R_j` below tolerance): scenario `A_normal`
   certifies at 0.42% gap with 2 latent items and zero structural actions;
   the grid normal stage certifies with exactly `K = C` corridor atoms.
   Caveat (visible in the flow-RMSE column): the certificate bounds *cost*
   suboptimality, not disaggregate link-flow error — frozen proportions can
   meet a loose ε while individual link flows are off; tightening ε forces
   the refinement that fixes them.
2. **Does reduced-gradient heterogeneity trigger splits correctly?**  Yes:
   the incident makes `R_{family_A}` jump (5.0% gap), pricing finds the
   suppressed path hidden in the atom, promotion releases it, and one
   re-optimisation returns below ε.  Family B is never touched — only the
   affected branch expands.
3. **When must a merge be re-split?**  When a new shock pushes `R_j` back over
   `τ_split` (scenario `D_reshock`): the item merged in `C_recovery` at the
   learned 0.50/0.50 proportions is re-opened by a single promotion.
   Hysteresis (`τ_merge ≪ τ_split`, stability counters, cooldowns) prevents
   oscillation between the two.
4. **Does HSD reach the same gap with fewer master variables?**  At scale,
   yes — peak 13 / end 8 columns vs. 38 for flat SD at 4096 paths, same
   certificate (table above).
5. **After a demand/capacity shock, does HSD expand only the affected
   branch?**  Yes: `I_incident_c4` touches corridor 4's atom only; the other
   `C−1` corridor atoms stay latent, and after recovery the frontier merges
   back to `K = C` with proportions *learned from the master coefficients*
   (e.g. the initial mis-specified 0.3/0.7 family-B item re-forms as
   0.70/0.30 after one expand/contract cycle).

## Outputs (`results/`)

- `summary.md` — per-scenario tables + round-by-round HSD narrative
- `comparison.csv`, `hsd_rounds.csv`, `grid_benchmark.csv` — raw metrics
- `atom_features.csv` — per-atom feature vectors `z_j = [λ_j, c̄_j, R_j,
  min δ, Var(c_p), |S_j|]` logged every state change: this is the training
  data for the phase-3 **gradient network** (the network recommends,
  the optimization algorithm verifies)
- `fig_convergence_A.png`, `fig_hsd_lifecycle.png`, `fig_grid_scaling.png`

## Roadmap

- **Phase 1 (this prototype)**: single OD, path-level atoms — done.
- **Phase 2**: leaves become origin-flow patterns; a bush + fixed approach
  proportions is a composite origin-flow atom; a bush allowed to re-optimise
  internally is a local subproblem.  The atom/master interfaces here are
  already agnostic to what a "column" aggregates.
- **Phase 3**: a gradient network maps `z_j` to a recommended structural
  action {keep, promote, split, demote, merge}; every recommendation is
  checked by the same feasibility/pricing machinery before being applied.
