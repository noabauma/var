#!/usr/bin/env python3
"""Bridge between slowmo_cam's web scoreboard and the bias score function.

Reads a win matrix on stdin, prints one line of scores on stdout:

    input:  N  followed by N*N matrix entries (whitespace-separated),
            where M[i][j] = number of times team i beat team j
    output: N space-separated scores from pagerank(M, participation_bias=True)

The algorithm lives in page_rank_billiardino_algorithm_bias.py — edit it
there and the live scoreboard follows. recursive_deletion() is intentionally
NOT applied: it disqualifies the least-connected teams, which is wrong for a
scoreboard that must rank everyone mid-tournament; the participation bias
already compensates for uneven numbers of games.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from page_rank_billiardino_algorithm_bias import pagerank


def main():
    data = sys.stdin.read().split()
    n = int(data[0])
    if n <= 0:
        print()
        return
    vals = [float(x) for x in data[1:1 + n * n]]
    if len(vals) != n * n:
        sys.exit("matrix data incomplete")
    M = np.array(vals, dtype=np.float64).reshape(n, n)
    v = pagerank(M, 0.85, participation_bias=True)
    print(" ".join(f"{x:.10f}" for x in v))


if __name__ == "__main__":
    main()
