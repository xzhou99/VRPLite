"""Scalable multi-corridor test networks for the HSD scaling benchmark.

Topology (single OD, C corridors, m "diamond" segments per corridor):

    O ──entry_c──> [diamond 1] ──> [diamond 2] ──> ... ──> [diamond m] ──> S ──> D

Each diamond is a pair of parallel links (upper/lower) with deliberately
asymmetric capacities, so the equilibrium split differs diamond by diamond
and a corridor genuinely needs 2^m primitive paths to describe exactly.
All corridors share the congested collector link S -> D, so corridors are
NOT independent parallel routes.

Path count = C * 2^m  (exponential in m) while the link count stays
C*(2m+2) + 1  (linear).  This is the regime where flat SD must accumulate
many primitive columns but a single well-proportioned latent item per
corridor already describes the interior almost exactly.

Initial latent items: for each corridor we freeze the *corridor-local
equilibrium* proportions -- per diamond, the scalar split s_i solving
t_u(s f) = t_l((1-s) f) at the nominal corridor flow f = q/C, and the path
distribution is the product measure over diamonds (this is what a historical
solution / an origin-based subproblem would hand us as a latent flow item).
"""

import itertools

import numpy as np

from atoms import Atom
from network import Network, PathRegistry


def build_corridor_network(C, m, q, seed=7):
    """Returns (net, meta).  meta records per-corridor link ids."""
    rng = np.random.default_rng(seed)
    links = []
    meta = {"C": C, "m": m, "q": q, "corridors": []}
    n_nodes = 3 + C * (m + 1)          # O, S, D + per corridor: entry + m chain
    O, S, D = 0, 1, 2

    def add(u, v, t0, cap):
        links.append((u, v, t0, cap))
        return len(links) - 1

    node = 3
    f_nom = q / C
    for c in range(C):
        entry = node
        chain = [node + 1 + i for i in range(m)]
        node += m + 1
        cor = {"entry": None, "diamonds": [], "exit": None}
        # entry links differentiate the corridors and carry the inter-corridor
        # congestion (cap around the nominal share)
        cor["entry"] = add(O, entry, 10.0 + 0.4 * c, 1.10 * f_nom)
        prev = entry
        for i in range(m):
            t0u = 2.0 * (1 + 0.001 * rng.standard_normal())
            t0l = 2.0 * (1 + 0.001 * rng.standard_normal())
            cap_u = 0.75 * f_nom * (1.0 + 0.20 * rng.uniform(-1, 1))
            cap_l = 0.75 * f_nom * (1.0 + 0.20 * rng.uniform(-1, 1))
            up = add(prev, chain[i], t0u, cap_u)
            lo = add(prev, chain[i], t0l, cap_l)
            cor["diamonds"].append((up, lo))
            prev = chain[i]
        cor["exit"] = add(prev, S, 2.0, 1.50 * f_nom)
        meta["corridors"].append(cor)
    add(S, D, 3.0, 1.05 * q)           # shared collector: cross-corridor coupling
    meta["collector"] = len(links) - 1
    net = Network(n_nodes, links)
    meta["origin"], meta["dest"] = O, D
    return net, meta


def _diamond_split(net, up, lo, f):
    """Scalar equilibrium split s in (0,1): t_up(s f) = t_lo((1-s) f)."""
    def imbalance(s):
        tu = net.t0[up] * (1 + net.alpha * (s * f / net.cap[up]) ** net.beta)
        tl = net.t0[lo] * (1 + net.alpha * ((1 - s) * f / net.cap[lo]) ** net.beta)
        return tu - tl
    lo_s, hi_s = 1e-6, 1 - 1e-6
    if imbalance(lo_s) > 0:
        return lo_s
    if imbalance(hi_s) < 0:
        return hi_s
    for _ in range(80):
        mid = 0.5 * (lo_s + hi_s)
        if imbalance(mid) > 0:
            hi_s = mid
        else:
            lo_s = mid
    return 0.5 * (lo_s + hi_s)


def build_corridor_atoms(net, reg, meta):
    """One latent item per corridor, frozen at corridor-local equilibrium
    proportions (product measure over the per-diamond splits)."""
    q, C, m = meta["q"], meta["C"], meta["m"]
    f_nom = q / C
    atoms = []
    for c, cor in enumerate(meta["corridors"]):
        splits = [_diamond_split(net, up, lo, f_nom)
                  for up, lo in cor["diamonds"]]
        path_ids, weights = [], []
        for choice in itertools.product((0, 1), repeat=m):
            seq = [cor["entry"]]
            w = 1.0
            for i, take_lower in enumerate(choice):
                up, lo = cor["diamonds"][i]
                seq.append(lo if take_lower else up)
                w *= (1 - splits[i]) if take_lower else splits[i]
            seq += [cor["exit"], meta["collector"]]
            path_ids.append(reg.add(tuple(seq), name=f"c{c}_" +
                                    "".join(str(b) for b in choice)))
            weights.append(w)
        atoms.append(Atom(path_ids, weights, name=f"corridor_{c}"))
    return atoms


def grid_scenarios(meta, incident_corridors=None):
    """Repeated-shock chain:  N -> (I@c, R) for each incident corridor.

    Each incident knocks 75% of the capacity off the upper link of the middle
    diamond of one corridor; demand stays the same, the shock is structural.
    The repetition is the point: a method that can only *add* master columns
    grows monotonically over the chain, a hierarchy that can merge back does
    not."""
    q, C = meta["q"], meta["C"]
    if incident_corridors is None:
        incident_corridors = sorted({0, C // 2, C - 1})
    chain = [{"name": "N_normal", "q": q, "cap_overrides": {}}]
    for c in incident_corridors:
        cor = meta["corridors"][c]
        inc_link = cor["diamonds"][len(cor["diamonds"]) // 2][0]
        chain.append({"name": f"I_incident_c{c}", "q": q,
                      "cap_overrides": {inc_link: None}})
        chain.append({"name": f"R_recovery_c{c}", "q": q, "cap_overrides": {}})
    return chain


def apply_grid_scenario(net, scen):
    net.cap[:] = net.base_cap
    for e, cap in scen["cap_overrides"].items():
        net.cap[e] = 0.25 * net.base_cap[e] if cap is None else cap
    return scen["q"]
