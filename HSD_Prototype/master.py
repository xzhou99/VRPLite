"""Restricted master problem (RMP) over a frontier of composite atoms.

    min_lambda  Z( q * H lambda )     s.t.  lambda >= 0,  sum_j lambda_j = 1

where H = [h_1 ... h_K] stacks the atom columns.  The gradient is
    dZ/dlambda_j = q * h_j^T t(x) = q * cbar_j,
i.e. exactly q times the atom mean cost, so the simplex-reduced gradient of
atom j w.r.t. the best atom b is  q (cbar_j - cbar_b)  (Principle 2 at the
atom level).  Solved by projected gradient with Barzilai-Borwein steps and an
Armijo backtracking safeguard; the restricted Frank-Wolfe gap
    q * (lambda . cbar - min_j cbar_j)
is the exactness certificate for the restricted problem.
"""

import numpy as np


def project_simplex(v):
    """Euclidean projection onto {w >= 0, sum w = 1} (Duchi et al. 2008)."""
    v = np.asarray(v, dtype=float)
    u = np.sort(v)[::-1]
    css = np.cumsum(u) - 1.0
    ind = np.arange(1, len(v) + 1)
    cond = u - css / ind > 0
    rho = ind[cond][-1]
    theta = css[cond][-1] / rho
    return np.maximum(v - theta, 0.0)


def solve_rmp(net, H, q, lam0=None, rel_tol=1e-11, max_iter=4000):
    """Returns (lambda, info).  info has iters, rgap_rel, Z, converged."""
    K = H.shape[1]
    if K == 1:
        lam = np.array([1.0])
        Z = net.beckmann(q * H[:, 0])
        return lam, {"iters": 0, "rgap_rel": 0.0, "Z": Z, "converged": True}

    lam = (np.full(K, 1.0 / K) if lam0 is None
           else project_simplex(np.asarray(lam0, dtype=float)))

    def evaluate(l):
        x = q * (H @ l)
        t = net.link_times(x)
        return net.beckmann(x), q * (H.T @ t)

    Z, g = evaluate(lam)
    step = 1.0 / (np.abs(g).max() + 1e-12)
    prev_lam = prev_g = None
    rgap_rel = np.inf
    it = 0
    for it in range(1, max_iter + 1):
        cbar = g / q
        cmin = float(cbar.min())
        rgap_rel = float(lam @ cbar - cmin) / max(cmin, 1e-12)
        if rgap_rel <= rel_tol:
            break
        if prev_lam is not None:
            s = lam - prev_lam
            y = g - prev_g
            sy = float(s @ y)
            if sy > 1e-16:
                step = float(s @ s) / sy
        step = float(np.clip(step, 1e-10, 1e6))
        prev_lam, prev_g = lam.copy(), g.copy()
        cand = lam
        for _ in range(60):
            cand = project_simplex(lam - step * g)
            d = cand - lam
            gd = float(g @ d)
            if gd >= -1e-18:                       # numerically stationary
                break
            Zc = net.beckmann(q * (H @ cand))
            if Zc <= Z + 1e-4 * gd:                # Armijo sufficient decrease
                break
            step *= 0.5
        if float(np.max(np.abs(cand - lam))) < 1e-16:
            break                                  # stalled at machine precision
        lam = cand
        Z, g = evaluate(lam)
    return lam, {"iters": it, "rgap_rel": rgap_rel, "Z": Z,
                 "converged": rgap_rel <= rel_tol}
