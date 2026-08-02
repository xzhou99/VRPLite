"""Numerical verification of the five claims the minimal HSD model must prove:

  1. fixed-proportion atoms preserve feasibility;
  2. reduced gradients identify wrong fixed proportions;
  3. split releases freedom with zero disruption at the split instant;
  4. merge compresses freedom with zero disruption at the merge instant;
  5. exact shortest-path pricing is a full-space certificate.

Run:  python3 verify_principles.py
"""

import numpy as np

from atoms import Atom, merge_atoms, promote, split_by_cost
from master import solve_rmp
from scenarios import DEST, ORIGIN, apply_scenario, build

RNG = np.random.default_rng(0)


def check(label, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""))
    if not ok:
        raise SystemExit(1)


def main():
    net, reg, (p1, p2, p3, p4), (h_A, h_B) = build()
    q = apply_scenario(net, {"q": 100.0, "cap_overrides": {}})

    # ---------------------------------------------------------------- claim 1
    print("Claim 1: a latent item is first of all a feasible composite atom")
    lam = np.array([0.55, 0.45])
    f = np.concatenate([q * lam[0] * h_A.pi, q * lam[1] * h_B.pi])
    check("implied path flows nonnegative", np.all(f >= 0))
    check("demand conservation sum f = q", abs(f.sum() - q) < 1e-12,
          f"sum={f.sum():.12f}")
    H = np.column_stack([h_A.column(reg), h_B.column(reg)])
    x_atom = q * (H @ lam)
    x_path = reg.A[:, [p1, p2, p3, p4]] @ f
    check("h_j = sum_p pi a_p reproduces link flows",
          np.max(np.abs(x_atom - x_path)) < 1e-12)

    # ---------------------------------------------------------------- claim 2
    print("Claim 2: reduced gradients identify wrong fixed proportions")
    # congest the family-A branches so proportions matter, then freeze a bad pi
    net.cap[1] = net.cap[2] = net.cap[3] = net.cap[4] = 30.0
    bad = Atom([p1, p2], [0.9, 0.1], name="bad_A")
    Hb = np.column_stack([bad.column(reg), h_B.column(reg)])
    lam_b, _ = solve_rmp(net, Hb, q)
    x = q * (Hb @ lam_b)
    c = reg.costs(net.link_times(x))
    delta = bad.deltas(c)
    check("overloaded path has delta > 0 (should shed flow)", delta[0] > 0,
          f"delta_p1={delta[0]:.3f}")
    check("starved path has delta < 0 (should gain flow)", delta[1] < 0,
          f"delta_p2={delta[1]:.3f}")
    Z_frozen = net.beckmann(x)
    prim, rest, w = promote(bad, p1)          # release the frozen proportion
    Hp = np.column_stack([prim.column(reg), rest.column(reg), h_B.column(reg)])
    lam_p0 = np.array([w * lam_b[0], (1 - w) * lam_b[0], lam_b[1]])
    x_map = q * (Hp @ lam_p0)
    check("promotion lambda mapping is zero-disruption",
          np.max(np.abs(x_map - x)) < 1e-10)
    lam_p, _ = solve_rmp(net, Hp, q, lam0=lam_p0)
    Z_free = net.beckmann(q * (Hp @ lam_p))
    check("re-optimising after release strictly lowers Z", Z_free < Z_frozen - 1e-6,
          f"{Z_frozen:.4f} -> {Z_free:.4f}")
    net.cap[:] = net.base_cap

    # ---------------------------------------------------------------- claim 3
    print("Claim 3: split is a zero-disruption refinement")
    Hc = np.column_stack([h_A.column(reg), h_B.column(reg)])
    lam_c, _ = solve_rmp(net, Hc, q)
    x_before = q * (Hc @ lam_c)
    c = reg.costs(net.link_times(x_before))
    c1, c2, alpha = split_by_cost(h_A, c)
    Hs = np.column_stack([c1.column(reg), c2.column(reg), h_B.column(reg)])
    lam_s = np.array([alpha * lam_c[0], (1 - alpha) * lam_c[0], lam_c[1]])
    x_after = q * (Hs @ lam_s)
    check("link flows unchanged at split instant",
          np.max(np.abs(x_after - x_before)) < 1e-12)
    check("objective unchanged at split instant",
          abs(net.beckmann(x_after) - net.beckmann(x_before)) < 1e-9)
    check("lambda still on the simplex", abs(lam_s.sum() - 1) < 1e-12)

    # ---------------------------------------------------------------- claim 4
    print("Claim 4: merge is zero-disruption compression")
    lam_s2, _ = solve_rmp(net, Hs, q, lam0=lam_s)
    x_pre = q * (Hs @ lam_s2)
    merged, lam_m = merge_atoms(c1, c2, lam_s2[0], lam_s2[1])
    Hm = np.column_stack([merged.column(reg), h_B.column(reg)])
    x_post = q * (Hm @ np.array([lam_m, lam_s2[2]]))
    check("link flows unchanged at merge instant",
          np.max(np.abs(x_post - x_pre)) < 1e-12)
    check("merged proportions = learned lambda ratio",
          abs(merged.pi[0] - lam_s2[0] / (lam_s2[0] + lam_s2[1])) < 1e-12)

    # ---------------------------------------------------------------- claim 5
    print("Claim 5: exact pricing is a full-space certificate")
    x = x_pre
    t = net.link_times(x)
    cstar, seq = net.shortest_path(t, ORIGIN, DEST)
    all_paths = net.all_simple_paths(ORIGIN, DEST)
    costs = [sum(t[e] for e in p) for p in all_paths]
    check("Dijkstra matches brute-force enumeration over ALL simple paths",
          abs(cstar - min(costs)) < 1e-12,
          f"{len(all_paths)} paths enumerated")
    lam_full, info = solve_rmp(net, Hm, q, lam0=np.array([lam_m, lam_s2[2]]))
    x = q * (Hm @ lam_full)
    t = net.link_times(x)
    cstar, _ = net.shortest_path(t, ORIGIN, DEST)
    gap = float(x @ t) - q * cstar
    check("full-space FW gap is nonnegative", gap >= -1e-9, f"gap={gap:.3e}")

    print("\nAll five claims verified numerically.")


if __name__ == "__main__":
    main()
