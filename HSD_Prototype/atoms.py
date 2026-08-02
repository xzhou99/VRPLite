"""Latent Flow Item (concept)  ==  Feasible Composite Atom (mathematical object).

An atom j over path set S_j with fixed internal proportions
    pi_{p|j} >= 0,   sum_p pi_{p|j} = 1
has the link-space column
    h_j = sum_{p in S_j} pi_{p|j} a_p.
Given a master coefficient lambda_j on the demand simplex, the implied path
flows are  f_p = q * lambda_j * pi_{p|j}  >= 0  and demand conservation holds
automatically  (Principle 1: every latent representation must first be a
feasible composite atom).

Reduced-gradient quantities (Principles 2-3):
    cbar_j     = sum_p pi_{p|j} c_p          mean cost of the atom
    delta_{p|j} = c_p - cbar_j               within-atom reduced gradient
    R_j        = max_p c_p - min_p c_p       within-atom heterogeneity

All structural operations below are *zero-disruption*: at the instant they are
applied the link flows, the objective and demand conservation are unchanged;
only the future degrees of freedom of the master problem change.
"""

import itertools

import numpy as np

_id_counter = itertools.count()


class Atom:
    def __init__(self, path_ids, pi, name):
        pi = np.asarray(pi, dtype=float)
        if np.any(pi < -1e-12):
            raise ValueError("internal proportions must be nonnegative")
        pi = np.maximum(pi, 0.0)
        s = pi.sum()
        if s <= 0:
            raise ValueError("internal proportions must sum to a positive value")
        self.paths = list(path_ids)
        self.pi = pi / s
        self.name = name
        self.uid = next(_id_counter)
        self.born_round = -10 ** 9   # overwritten when installed into a frontier

    # ------------------------------------------------------------------ basic
    @property
    def is_primitive(self):
        return len(self.paths) == 1

    def column(self, registry):
        """h_j = sum_p pi_{p|j} a_p.  Atoms are immutable once created
        (every structural action builds new atoms), so the column is cached."""
        if getattr(self, "_col", None) is None:
            self._col = registry.A[:, self.paths] @ self.pi
        return self._col

    def mean_cost(self, c):
        """cbar_j."""
        return float(c[self.paths] @ self.pi)

    def deltas(self, c):
        """delta_{p|j} = c_p - cbar_j for p in S_j."""
        return c[self.paths] - self.mean_cost(c)

    def heterogeneity(self, c):
        """R_j = max_p c_p - min_p c_p over S_j."""
        cj = c[self.paths]
        return float(cj.max() - cj.min())

    def __repr__(self):
        if len(self.paths) <= 4:
            inner = ", ".join(f"p{p + 1}:{w:.3f}"
                              for p, w in zip(self.paths, self.pi))
        else:
            top = np.argsort(self.pi)[::-1][:3]
            inner = f"|S|={len(self.paths)}, top: " + ", ".join(
                f"p{self.paths[k] + 1}:{self.pi[k]:.3f}" for k in top)
        return f"Atom[{self.name}]({inner})"


# --------------------------------------------------------------- structural ops
def promote(atom, path_id):
    """Promotion: pull one path out of a composite atom.

    h_j = pi_p a_p + (1 - pi_p) h_{j \\ p};  the frontier entry {h_j} becomes
    {a_p, h_{j\\p}} with the lambda mapping
        lambda_prim = pi_p * lambda_j,   lambda_rest = (1 - pi_p) * lambda_j.
    Returns (primitive_atom, remainder_atom, pi_p).
    """
    i = atom.paths.index(path_id)
    w = float(atom.pi[i])
    prim = Atom([path_id], [1.0], name=f"p{path_id + 1}")
    rest_paths = [p for k, p in enumerate(atom.paths) if k != i]
    rest_pi = np.array([atom.pi[k] for k in range(len(atom.paths)) if k != i])
    rest_name = rest_paths and (f"p{rest_paths[0] + 1}" if len(rest_paths) == 1
                                else f"{atom.name}\\p{path_id + 1}")
    rest = Atom(rest_paths, rest_pi, name=rest_name)
    if len(rest.name) > 24:
        rest.name = f"latent#{rest.uid}[{len(rest_paths)}p]"
    return prim, rest, w


def split_by_cost(atom, c):
    """Split: partition S_j by the sign of the within-atom reduced gradient.

    Group 1 = {p : delta_{p|j} <= 0} (cheap side), group 2 = the rest.
    h_j = alpha h_j1 + (1 - alpha) h_j2 with
        lambda_j1 = alpha lambda_j,   lambda_j2 = (1 - alpha) lambda_j.
    Returns (child1, child2, alpha).
    """
    delta = atom.deltas(c)
    g1 = [k for k in range(len(atom.paths)) if delta[k] <= 0]
    g2 = [k for k in range(len(atom.paths)) if delta[k] > 0]
    if not g1 or not g2:                       # degenerate: split off the cheapest
        order = np.argsort(delta)
        g1, g2 = [int(order[0])], [int(k) for k in order[1:]]
    alpha = float(sum(atom.pi[k] for k in g1))

    def make(idx, tag):
        paths = [atom.paths[k] for k in idx]
        pi = np.array([atom.pi[k] for k in idx])
        name = f"p{paths[0] + 1}" if len(paths) == 1 else f"{atom.name}.{tag}"
        return Atom(paths, pi, name=name)

    return make(g1, "lo"), make(g2, "hi"), alpha


def merge_atoms(a1, a2, lam1, lam2, name=None):
    """Merge: dynamically form a new latent flow item.

    h_m = (lam1 h_1 + lam2 h_2) / (lam1 + lam2),  lambda_m = lam1 + lam2.
    At the merge instant lambda_m h_m = lam1 h_1 + lam2 h_2, so link flows,
    objective and feasibility are all unchanged; the internal proportions are
    frozen at the *current* (learned) ratio.
    Returns (merged_atom, lambda_m).
    """
    tot = lam1 + lam2
    if tot <= 0:                               # both empty: fall back to 50/50
        lam1 = lam2 = 0.5
        tot = 1.0
    weights = {}
    for atom, lam in ((a1, lam1), (a2, lam2)):
        for p, w in zip(atom.paths, atom.pi):
            weights[p] = weights.get(p, 0.0) + lam * w / tot
    paths = sorted(weights)
    pi = np.array([weights[p] for p in paths])
    if name is None:
        name = f"({a1.name}+{a2.name})"
    atom = Atom(paths, pi, name=name)
    if len(atom.name) > 24:                    # keep lineage names readable
        atom.name = f"latent#{atom.uid}[{len(paths)}p]"
    return atom, lam1 + lam2


def weighted_column_distance(h1, h2, t_prime):
    """||h_1 - h_2||_W with W = diag(t_a'(x))  (congestion sensitivity).

    This is a second-order estimate of the flexibility lost by freezing the
    proportion between the two atoms: differences on uncongested links cost
    nothing, differences on congested links are penalised by the local slope
    of the delay function.
    """
    d = h1 - h2
    return float(np.sqrt(np.sum(t_prime * d * d)))
