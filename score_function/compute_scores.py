#!/usr/bin/env python3
"""Bridge between slowmo_cam's web scoreboard and the score functions.

Reads a win matrix on stdin, prints two lines of scores on stdout:

    usage:  compute_scores.py [d]   (PageRank damping factor, default 0.85)
    input:  N  followed by N*N matrix entries (whitespace-separated),
            where M[i][j] = 1 if team i won the (single) match vs team j
    output: line 1 — bias PageRank  (page_rank_billiardino_algorithm_bias)
            line 2 — classic PageRank (page_rank_billiardino_algorithm)

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
from page_rank_billiardino_algorithm_bias import pagerank as pagerank_bias
from page_rank_billiardino_algorithm import pagerank as pagerank_plain


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
