#!/usr/bin/env python3
"""Bridge between slowmo_cam's web scoreboard and the score functions.

Reads a win matrix on stdin, prints two lines of scores on stdout:

    usage:  compute_scores.py [d]   (PageRank damping factor, default 0.85)
    input:  N  followed by N*N matrix entries (whitespace-separated),
            where M[i][j] = 1 if team i won the (single) match vs team j
    output: line 1 — bias PageRank  (page_rank_biliardino_algorithm_bias)
            line 2 — classic PageRank (page_rank_biliardino_algorithm)

Both algorithm files stay the single source of truth — edit them and the
live scoreboard follows. recursive_deletion() is intentionally NOT applied:
it disqualifies the least-connected teams, which is wrong for a scoreboard
that must rank everyone mid-tournament.
"""
import contextlib
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from page_rank_biliardino_algorithm_bias import pagerank as pagerank_bias
from page_rank_biliardino_algorithm import pagerank as pagerank_plain


def main():
    d = 0.85
    if len(sys.argv) > 1:
        try:
            d = float(sys.argv[1])
        except ValueError:
            sys.exit(f"bad damping factor {sys.argv[1]!r}")
        if not 0.0 <= d <= 0.99:
            sys.exit("damping factor must be in [0, 0.99]")
    data = sys.stdin.read().split()
    if data and data[0] == "H":
        # history mode: S prefix-matrices over the same n teams; prints S
        # lines of bias scores (one per snapshot) in one numpy session
        S, n = int(data[1]), int(data[2])
        vals = [float(x) for x in data[3:3 + S * n * n]]
        if len(vals) != S * n * n:
            sys.exit("history data incomplete")
        lines = []
        with contextlib.redirect_stdout(io.StringIO()):
            for k in range(S):
                M = np.array(vals[k * n * n:(k + 1) * n * n],
                             dtype=np.float64).reshape(n, n)
                v = pagerank_bias(M, d, participation_bias=True)
                lines.append(" ".join(f"{x:.10f}" for x in v))
        print("\n".join(lines))
        return
    if data and data[0] == "L":
        # leave-one-out mode: the impact of each match on the BIAS scores.
        # input: L S n, the full n*n matrix, then S (winner, loser) index
        # pairs. Output: S lines "dw dl" — winner's score with the match
        # minus without it, and the same for the loser — i.e. how many
        # points the win adds and the loss costs vs a tournament in which
        # that match was never played.
        S, n = int(data[1]), int(data[2])
        vals = [float(x) for x in data[3:3 + n * n]]
        pairs = [int(x) for x in data[3 + n * n:3 + n * n + 2 * S]]
        if len(vals) != n * n or len(pairs) != 2 * S:
            sys.exit("leave-one-out data incomplete")
        M = np.array(vals, dtype=np.float64).reshape(n, n)
        lines = []
        with contextlib.redirect_stdout(io.StringIO()):
            base = pagerank_bias(M.copy(), d, participation_bias=True)
            for k in range(S):
                w, l = pairs[2 * k], pairs[2 * k + 1]
                M2 = M.copy()
                M2[w][l] -= 1.0
                v = pagerank_bias(M2, d, participation_bias=True)
                lines.append(f"{base[w] - v[w]:.10f} {base[l] - v[l]:.10f}")
        print("\n".join(lines))
        return
    n = int(data[0])
    if n <= 0:
        print()
        print()
        return
    vals = [float(x) for x in data[1:1 + n * n]]
    if len(vals) != n * n:
        sys.exit("matrix data incomplete")
    M = np.array(vals, dtype=np.float64).reshape(n, n)
    # the classic pagerank() print()s its matrix — keep stdout clean
    with contextlib.redirect_stdout(io.StringIO()):
        vb = pagerank_bias(M.copy(), d, participation_bias=True)
        vp = pagerank_plain(M.copy(), d)
    print(" ".join(f"{x:.10f}" for x in vb))
    print(" ".join(f"{x:.10f}" for x in vp))


if __name__ == "__main__":
    main()
