"""Hierarchical Simplicial Decomposition (HSD) for a single OD pair.

Core sentence of the method:

    HSD uses reduced gradients not only to adjust the coefficients of active
    columns, but also to decide whether the fixed internal proportions of a
    latent flow item should remain constrained or be released through
    hierarchical refinement.

Per round:
  1. solve the restricted master over the current frontier (atom-level
     reduced gradients move the lambda coefficients);
  2. compute the two-level reduced gradients
        atom level    rbar_j     = q (cbar_j - cbar_b)
        within atom   delta_{p|j} = c_p - cbar_j,   R_j = max c_p - min c_p;
  3. exact shortest-path pricing over the full network gives the full-space
     Frank-Wolfe gap certificate
        g(x) = q * (sum_j lambda_j cbar_j - c_{p*});
  4. structural actions in strict priority order
        Generate > Promote > Split  >>  (only when converged & stable)
        Demote > Merge > Drop,
     with hysteresis (cooldowns + stability counters) so that split/merge
     cannot oscillate.  At most one refinement action per round; merge and
     split never happen in the same round.
"""

import time
from dataclasses import dataclass, field

import numpy as np

from atoms import (Atom, merge_atoms, promote, split_by_cost,
                   weighted_column_distance)
from master import solve_rmp


@dataclass
class HSDParams:
    eps_full: float = 5e-3      # relative full-space gap certificate
    eps_rmp: float = 1e-11      # relative restricted-master gap
    tau_promote: float = 1.0    # cbar_j - min_p c_p (time units) to promote
    tau_split: float = 2.0      # within-atom heterogeneity R_j to split
    tau_weight: float = 0.05    # min lambda_j for a split to be worthwhile
    tau_cost: float = 0.05      # |cbar_i - cbar_j| for merge (hysteresis: << tau_split)
    tau_flow: float = 0.10      # congestion-weighted column distance for merge
    tau_minor: float = 1e-3     # lambda below which an explicit path may demote
    tau_demote: float = 0.10    # cost proximity |c_p - cbar_j| for demotion
    stable_rounds: int = 4      # consecutive converged rounds before demote/merge
    cooldown: int = 3           # rounds a new atom is protected from merge/demote
    drop_tol: float = 1e-8      # lambda below which a primitive is dead weight
    quiet_rounds: int = 6       # converged & action-free rounds before stopping
    max_rounds: int = 200


@dataclass
class RoundRecord:
    scenario: str
    round: int
    Z: float
    rel_gap: float
    frontier_size: int
    explicit_paths: int
    represented_paths: int
    rmp_iters: int
    actions: str


class HSD:
    def __init__(self, net, registry, origin, dest, params=None):
        self.net = net
        self.reg = registry
        self.origin = origin
        self.dest = dest
        self.p = params or HSDParams()
        self.frontier = []                    # list[Atom]
        self.lam = np.zeros(0)
        self.q = None
        self.history = []                     # list[RoundRecord]
        self.atom_feature_log = []            # training data for the future gradient network
        self.counters = {k: 0 for k in
                         ("generate", "promote", "split", "merge", "demote",
                          "drop", "sp_calls", "rmp_solves", "rmp_iters")}
        self._merge_stab = {}                 # (uid_i, uid_j) -> consecutive rounds
        self._demote_stab = {}                # uid -> (parent_uid, rounds)
        self._drop_stab = {}                  # uid -> rounds with lambda ~ 0

    # ----------------------------------------------------------------- setup
    def install(self, atoms, lam=None):
        self.frontier = list(atoms)
        for a in self.frontier:
            a.born_round = -10 ** 9
        K = len(self.frontier)
        self.lam = np.full(K, 1.0 / K) if lam is None else np.asarray(lam, float)

    def columns(self):
        return np.column_stack([a.column(self.reg) for a in self.frontier])

    # ------------------------------------------------------------------- run
    def run(self, q, scenario="", log=None, max_rounds=None):
        """Run HSD until certified convergence + structural quiescence."""
        self.q = q
        p = self.p
        say = log if log is not None else (lambda s: None)
        t_start = time.perf_counter()
        quiet = 0
        result = None
        peak_K = len(self.frontier)
        # any state carried over from a previous scenario is stale
        self._dirty = True
        x = t = tp = c = None
        Z = rel_gap = np.nan
        cstar = pstar = None
        rmp_iters = 0
        for rnd in range(1, (max_rounds or p.max_rounds) + 1):
            if self._dirty:
                # the frontier / lambda / network changed: re-solve and re-price
                H = self.columns()
                peak_K = max(peak_K, H.shape[1])
                self.lam, rmp = solve_rmp(self.net, H, q, lam0=self.lam,
                                          rel_tol=p.eps_rmp)
                self.counters["rmp_solves"] += 1
                self.counters["rmp_iters"] += rmp["iters"]
                rmp_iters = rmp["iters"]
                Z = rmp["Z"]
                x = q * (H @ self.lam)
                t = self.net.link_times(x)
                tp = self.net.link_time_derivs(x)
                # exact full-space pricing (certificate over ALL paths in the
                # network, not just the ones already discovered)
                cstar, seq = self.net.shortest_path(t, self.origin, self.dest)
                self.counters["sp_calls"] += 1
                pstar = self.reg.add(seq)
                c = self.reg.costs(t)
                cbar = np.array([a.mean_cost(c) for a in self.frontier])
                rel_gap = (float(self.lam @ cbar) - cstar) / cstar
                self._log_atom_features(scenario, rnd, c, tp)
                self._dirty = False
            else:
                # merge/demote are zero-disruption: x, t, c and the pricing
                # certificate are mathematically unchanged, so quiet rounds
                # cost no RMP solve and no shortest-path call
                rmp_iters = 0
            gap_ok = rel_gap <= p.eps_full

            actions = []
            if not gap_ok:
                actions += self._refine(pstar, c, rel_gap)
            if gap_ok and not actions:
                actions += self._consolidate(c, tp)

            rep = sorted({pp for a in self.frontier for pp in a.paths})
            rec = RoundRecord(scenario, rnd, Z, rel_gap,
                              len(self.frontier),
                              sum(a.is_primitive for a in self.frontier),
                              len(rep), rmp_iters,
                              "; ".join(actions) if actions else "-")
            self.history.append(rec)
            say(f"[{scenario}] round {rnd:3d}  Z={rmp['Z']:12.4f}  "
                f"gap={rel_gap * 100:8.4f}%  K={len(self.frontier)}  "
                f"actions: {rec.actions}")

            for a in self.frontier:            # age atoms
                if a.born_round == -10 ** 9:
                    a.born_round = rnd - 1
            if gap_ok and not actions:
                quiet += 1
                if quiet >= p.quiet_rounds:
                    result = "converged"
                    break
            else:
                quiet = 0
        cpu = time.perf_counter() - t_start
        H = self.columns()
        x = q * (H @ self.lam)
        return {"status": result or "max_rounds", "x": x,
                "Z": self.net.beckmann(x), "rel_gap": rel_gap,
                "rounds": rnd, "cpu_s": cpu, "peak_K": peak_K,
                "frontier": [repr(a) for a in self.frontier],
                "lam": self.lam.copy()}

    # -------------------------------------------------- refinement (gap > eps)
    def _refine(self, pstar, c, rel_gap):
        """Priority: Generate missing priced path > Promote > Split.

        One refinement per round.  All lambda mappings are zero-disruption.
        """
        p = self.p
        explicit = {a.paths[0]: j for j, a in enumerate(self.frontier)
                    if a.is_primitive}
        hidden_in = next((j for j, a in enumerate(self.frontier)
                          if not a.is_primitive and pstar in a.paths), None)

        # 1. Generate: the priced path is missing entirely from the frontier
        if pstar not in explicit and hidden_in is None:
            prim = Atom([pstar], [1.0], name=f"p{pstar + 1}")
            self.frontier.append(prim)
            self.lam = np.append(self.lam, 0.0)
            self.counters["generate"] += 1
            self._dirty = True
            return [f"generate {self.reg.names[pstar]} (new priced path)"]

        # 2. Promote: the priced path is locked inside a composite atom
        if hidden_in is not None:
            return [self._do_promote(hidden_in, pstar, reason="priced path hidden")]

        # 2b. Promote the most attractive hidden path anywhere
        best = None
        for j, a in enumerate(self.frontier):
            if a.is_primitive or self.lam[j] <= p.drop_tol:
                continue
            adv = a.mean_cost(c) - float(c[a.paths].min())
            if adv > p.tau_promote and (best is None or adv > best[0]):
                best = (adv, j, int(a.paths[int(np.argmin(c[a.paths]))]))
        if best is not None:
            return [self._do_promote(best[1], best[2],
                                     reason=f"delta advantage {best[0]:.2f}")]

        # 3. Split: high-weight atom with large internal heterogeneity
        cand = None
        for j, a in enumerate(self.frontier):
            if a.is_primitive:
                continue
            R = a.heterogeneity(c)
            if R > p.tau_split and self.lam[j] > p.tau_weight:
                if cand is None or R > cand[0]:
                    cand = (R, j)
        if cand is not None:
            return [self._do_split(cand[1], c, reason=f"R={cand[0]:.2f}")]

        # Escape hatch: gap certificate not met, priced path already explicit,
        # no threshold fired -> release the tightest remaining latent freedom
        # so the certificate stays reachable (falls back to plain SD behaviour).
        forced = None
        for j, a in enumerate(self.frontier):
            if a.is_primitive or self.lam[j] <= p.drop_tol:
                continue
            adv = a.mean_cost(c) - float(c[a.paths].min())
            if forced is None or adv > forced[0]:
                forced = (adv, j, int(a.paths[int(np.argmin(c[a.paths]))]))
        if forced is not None and forced[0] > 0:
            return [self._do_promote(forced[1], forced[2], reason="forced (certificate)")]
        return []

    def _do_promote(self, j, path_id, reason=""):
        atom = self.frontier[j]
        prim, rest, w = promote(atom, path_id)
        lam_j = self.lam[j]
        self.frontier[j:j + 1] = [prim, rest]
        self.lam = np.concatenate([self.lam[:j], [w * lam_j, (1 - w) * lam_j],
                                   self.lam[j + 1:]])
        self.counters["promote"] += 1
        self._dirty = True
        return (f"promote {self.reg.names[path_id]} out of {atom.name}"
                + (f" [{reason}]" if reason else ""))

    def _do_split(self, j, c, reason=""):
        atom = self.frontier[j]
        c1, c2, alpha = split_by_cost(atom, c)
        lam_j = self.lam[j]
        self.frontier[j:j + 1] = [c1, c2]
        self.lam = np.concatenate([self.lam[:j], [alpha * lam_j, (1 - alpha) * lam_j],
                                   self.lam[j + 1:]])
        self.counters["split"] += 1
        self._dirty = True
        return (f"split {atom.name} -> {{{c1.name}, {c2.name}}}"
                + (f" [{reason}]" if reason else ""))

    # ------------------------------------ consolidation (converged & stable)
    def _consolidate(self, c, tp):
        """Demote > Merge > Drop, each guarded by stability counters.

        Merge freezes the sibling proportion at the current (learned) ratio;
        the congestion-weighted column distance ||h_i - h_j||_W bounds the
        second-order flexibility loss of doing so.
        """
        p = self.p
        rnd = len(self.history) + 1
        uid2idx = {a.uid: j for j, a in enumerate(self.frontier)}

        # ---- demotion: minor explicit path folds back into a nearby family
        demote_now = None
        seen = set()
        for j, a in enumerate(self.frontier):
            if not a.is_primitive or self.lam[j] >= p.tau_minor \
                    or self.lam[j] <= p.drop_tol or rnd - a.born_round < p.cooldown:
                continue
            best = None
            for k, b in enumerate(self.frontier):
                if k == j or b.is_primitive:
                    continue
                if abs(c[a.paths[0]] - b.mean_cost(c)) < p.tau_demote and \
                        weighted_column_distance(a.column(self.reg),
                                                 b.column(self.reg), tp) < p.tau_flow:
                    best = b.uid
                    break
            if best is not None:
                seen.add(a.uid)
                parent, n = self._demote_stab.get(a.uid, (best, 0))
                n = n + 1 if parent == best else 1
                self._demote_stab[a.uid] = (best, n)
                if n >= p.stable_rounds and demote_now is None:
                    demote_now = (a.uid, best)
        for uid in list(self._demote_stab):
            if uid not in seen:
                del self._demote_stab[uid]
        if demote_now is not None:
            return [self._do_merge(uid2idx[demote_now[0]], uid2idx[demote_now[1]],
                                   kind="demote")]

        # ---- merge: two stable, cost-close, flow-close siblings re-aggregate
        merge_now = None
        seen = set()
        for j in range(len(self.frontier)):
            for k in range(j + 1, len(self.frontier)):
                a, b = self.frontier[j], self.frontier[k]
                if min(rnd - a.born_round, rnd - b.born_round) < p.cooldown:
                    continue
                if self.lam[j] <= p.drop_tol or self.lam[k] <= p.drop_tol:
                    continue                      # dead columns are dropped, not merged
                if abs(a.mean_cost(c) - b.mean_cost(c)) >= p.tau_cost:
                    continue
                if weighted_column_distance(a.column(self.reg), b.column(self.reg),
                                            tp) >= p.tau_flow:
                    continue
                key = (min(a.uid, b.uid), max(a.uid, b.uid))
                seen.add(key)
                n = self._merge_stab.get(key, 0) + 1
                self._merge_stab[key] = n
                if n >= p.stable_rounds and merge_now is None:
                    merge_now = (j, k)
        for key in list(self._merge_stab):
            if key not in seen:
                del self._merge_stab[key]
        if merge_now is not None:
            return [self._do_merge(*merge_now, kind="merge")]

        # ---- drop: primitive columns that stayed at lambda ~ 0
        seen = set()
        drop_now = None
        for j, a in enumerate(self.frontier):
            if a.is_primitive and self.lam[j] <= p.drop_tol \
                    and rnd - a.born_round >= p.cooldown:
                seen.add(a.uid)
                n = self._drop_stab.get(a.uid, 0) + 1
                self._drop_stab[a.uid] = n
                if n >= p.stable_rounds and drop_now is None:
                    drop_now = j
        for uid in list(self._drop_stab):
            if uid not in seen:
                del self._drop_stab[uid]
        if drop_now is not None:
            a = self.frontier.pop(drop_now)
            self.lam = np.delete(self.lam, drop_now)
            self.lam = self.lam / self.lam.sum()
            self.counters["drop"] += 1
            self._dirty = True         # renormalisation is not exactly zero-change
            return [f"drop {a.name} (lambda ~ 0)"]
        return []

    def _do_merge(self, j, k, kind="merge"):
        a, b = self.frontier[j], self.frontier[k]
        merged, lam_m = merge_atoms(a, b, self.lam[j], self.lam[k])
        merged.born_round = len(self.history) + 1
        keep = [i for i in range(len(self.frontier)) if i not in (j, k)]
        self.frontier = [self.frontier[i] for i in keep] + [merged]
        self.lam = np.append(self.lam[keep], lam_m)
        self.counters[kind] += 1
        pi_txt = ("/".join(f"{w:.2f}" for w in merged.pi)
                  if len(merged.pi) <= 6 else f"|S|={len(merged.pi)} paths")
        return f"{kind} {{{a.name}, {b.name}}} -> {merged.name} (pi={pi_txt})"

    # ------------------------------------------------------------- diagnostics
    def _log_atom_features(self, scenario, rnd, c, tp):
        """Per-atom feature vector z_j — future training data for the
        gradient-network structural policy (phase 3)."""
        for j, a in enumerate(self.frontier):
            cj = c[a.paths]
            self.atom_feature_log.append({
                "scenario": scenario, "round": rnd, "atom": a.name,
                "lambda": float(self.lam[j]) if j < len(self.lam) else 0.0,
                "cbar": a.mean_cost(c),
                "R": a.heterogeneity(c),
                "min_delta": float(cj.min() - a.mean_cost(c)),
                "var_c": float(np.var(cj)),
                "n_paths": len(a.paths),
            })
