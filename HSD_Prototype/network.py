"""Single-OD test network: BPR links, Beckmann objective, Dijkstra pricing,
and a registry of primitive path columns.

Notation (matches the HSD write-up):
    x_a   : link flow,           t_a(x) : BPR link travel time
    Z(x)  : Beckmann objective   sum_a  int_0^{x_a} t_a(w) dw
    a_p   : link-incidence column of primitive path p
    c_p   : a_p^T t(x)  = current cost of path p  (the path gradient dZ/df_p)
"""

import heapq

import numpy as np


class Network:
    """Directed link network with BPR volume-delay functions.

    t_a(x) = t0_a * (1 + alpha * (x / cap_a)^beta)
    """

    def __init__(self, num_nodes, links, alpha=0.15, beta=4.0):
        # links: list of (tail, head, t0, cap)
        self.num_nodes = num_nodes
        self.tail = np.array([l[0] for l in links], dtype=int)
        self.head = np.array([l[1] for l in links], dtype=int)
        self.t0 = np.array([l[2] for l in links], dtype=float)
        self.cap = np.array([l[3] for l in links], dtype=float)
        self.base_cap = self.cap.copy()
        self.alpha = alpha
        self.beta = beta
        self.num_links = len(links)
        self.out_links = [[] for _ in range(num_nodes)]
        for e in range(self.num_links):
            self.out_links[self.tail[e]].append(e)

    # ------------------------------------------------------------------ BPR
    def link_times(self, x):
        return self.t0 * (1.0 + self.alpha * (x / self.cap) ** self.beta)

    def link_time_derivs(self, x):
        b = self.beta
        return self.t0 * self.alpha * b * np.maximum(x, 0.0) ** (b - 1.0) / self.cap ** b

    def beckmann(self, x):
        r = x / self.cap
        return float(np.sum(self.t0 * (x + self.alpha * self.cap / (self.beta + 1.0)
                                       * r ** (self.beta + 1.0))))

    # ---------------------------------------------------------- shortest path
    def shortest_path(self, times, origin, dest):
        """Dijkstra. Returns (cost, tuple_of_link_ids). Exact full-space pricing."""
        dist = np.full(self.num_nodes, np.inf)
        pred = np.full(self.num_nodes, -1, dtype=int)
        dist[origin] = 0.0
        pq = [(0.0, origin)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u] + 1e-15:
                continue
            for e in self.out_links[u]:
                v = self.head[e]
                nd = d + times[e]
                if nd < dist[v] - 1e-15:
                    dist[v] = nd
                    pred[v] = e
                    heapq.heappush(pq, (nd, v))
        links = []
        v = dest
        while v != origin:
            e = pred[v]
            if e < 0:
                raise RuntimeError("destination unreachable")
            links.append(e)
            v = self.tail[e]
        links.reverse()
        return float(dist[dest]), tuple(links)

    def all_simple_paths(self, origin, dest):
        """DFS enumeration of every simple path (small graphs only).

        Used to (a) build the exact-UE reference solution over the full path
        space and (b) verify that Dijkstra pricing is a true full-space
        certificate.
        """
        paths = []

        def dfs(node, visited, seq):
            if node == dest:
                paths.append(tuple(seq))
                return
            for e in self.out_links[node]:
                v = self.head[e]
                if v not in visited:
                    visited.add(v)
                    seq.append(e)
                    dfs(v, visited, seq)
                    seq.pop()
                    visited.remove(v)

        dfs(origin, {origin}, [])
        return paths


class PathRegistry:
    """Registry of primitive paths discovered so far.

    Column matrix A has one link-incidence column a_p per registered path.
    Paths are only ever appended, so path ids are stable across the run.
    """

    def __init__(self, network):
        self.net = network
        self.link_seqs = []          # path id -> tuple of link ids
        self.index = {}              # tuple of link ids -> path id
        self.A = np.zeros((network.num_links, 0))
        self.names = []

    @property
    def num_paths(self):
        return len(self.link_seqs)

    def add(self, link_seq, name=None):
        link_seq = tuple(link_seq)
        if link_seq in self.index:
            return self.index[link_seq]
        col = np.zeros((self.net.num_links, 1))
        for e in link_seq:
            col[e, 0] += 1.0
        pid = self.num_paths
        self.link_seqs.append(link_seq)
        self.index[link_seq] = pid
        self.A = np.hstack([self.A, col])
        self.names.append(name if name is not None else f"p{pid + 1}")
        return pid

    def add_by_nodes(self, node_seq, name=None):
        links = []
        for u, v in zip(node_seq[:-1], node_seq[1:]):
            match = [e for e in self.net.out_links[u] if self.net.head[e] == v]
            if not match:
                raise ValueError(f"no link {u}->{v}")
            links.append(match[0])
        return self.add(tuple(links), name=name)

    def costs(self, times):
        """c_p = a_p^T t(x) for every registered path."""
        return self.A.T @ times

    def describe(self, pid):
        return f"{self.names[pid]}<{'-'.join(str(e) for e in self.link_seqs[pid])}>"
