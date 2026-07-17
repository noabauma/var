#!/usr/bin/env python3
"""Statistics for single-player (2v2 pickup) mode, computed from the
singles.tsv log written by referee_singles.py.

    python3 singles_stats.py                    # pretty report
    python3 singles_stats.py --json             # machine-readable
    python3 singles_stats.py --file other.tsv   # e.g. a test log

Per player: matches, win %, attacker/defender share, win % per role,
overall Elo, attacker-Elo, defender-Elo, PageRank (bias + classic via the
tournament's own compute_scores.py). Per table: side A vs side B win bias
with a binomial p-value so noise isn't reported as bias.

Ratings, two flavors on purpose:

  Elo      2v2 team-average Elo (K=32, start 1000): expected score from
           the difference of the two team means; both teammates move by
           the same delta. Handles ad-hoc lineups and margins of any
           game count, and its expectations are calibrated win
           probabilities (usable as betting odds later). Role-split Elo
           (only the games played in that role) answers "most valuable
           defender".
  PageRank the tournament algorithm, applied to the accumulated PLAYER
           matrix: each match adds M[winner][loser] += 1 for all 2x2
           winner/loser pairs. Kept as the familiar who-beat-whom view,
           consistent with the team tournament page.
"""
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG = os.path.expanduser("~/recordings/singles.tsv")
COMPUTE = os.path.join(HERE, "../score_function/compute_scores.py")

ELO_K = 32.0
ELO_START = 1000.0
MIN_GAMES_MVP = 3  # matches in role before "most valuable" is meaningful


def parse_log(path):
    """-> list of dicts {A_def, A_att, B_def, B_att, games, winner, time}"""
    matches = []
    with open(path) as fh:
        for line in fh:
            line = line.rstrip("\n")
            if not line or line.startswith("#"):
                continue
            f = line.split("\t")
            if len(f) < 8 or f[0] != "match":
                continue
            matches.append({"time": f[1], "A_def": f[2], "A_att": f[3],
                            "B_def": f[4], "B_att": f[5],
                            "games": f[6], "winner": f[7]})
    return matches


def elo_expect(ra, rb):
    return 1.0 / (1.0 + 10.0 ** ((rb - ra) / 400.0))


def run_elo(matches, role_filter=None):
    """Team-average 2v2 Elo. role_filter 'def'/'att' rates players only on
    the matches they played in that role (the other three still enter at
    their overall strength so the opposition is realistic)."""
    overall = {}
    rated = {}

    def get(d, p):
        return d.setdefault(p, ELO_START)

    for m in matches:
        a = (m["A_def"], m["A_att"])
        b = (m["B_def"], m["B_att"])
        ra = (get(overall, a[0]) + get(overall, a[1])) / 2.0
        rb = (get(overall, b[0]) + get(overall, b[1])) / 2.0
        sa = 1.0 if m["winner"] == "A" else 0.0
        delta = ELO_K * (sa - elo_expect(ra, rb))
        if role_filter:
            in_role = {"def": (a[0], b[0]), "att": (a[1], b[1])}[role_filter]
            for p in in_role:
                sign = delta if p in a else -delta
                rated[p] = get(rated, p) + sign
        for p in a:
            overall[p] += delta
        for p in b:
            overall[p] -= delta
    return rated if role_filter else overall


def run_pagerank(matches):
    """Accumulate the player matrix (M[w][l] += 1 per winner-loser pair)
    and feed it to the tournament's own compute_scores.py."""
    players = sorted({m[k] for m in matches
                      for k in ("A_def", "A_att", "B_def", "B_att")})
    idx = {p: i for i, p in enumerate(players)}
    n = len(players)
    M = [[0] * n for _ in range(n)]
    for m in matches:
        w = ("A_def", "A_att") if m["winner"] == "A" else ("B_def", "B_att")
        l = ("B_def", "B_att") if m["winner"] == "A" else ("A_def", "A_att")
        for wk in w:
            for lk in l:
                M[idx[m[wk]]][idx[m[lk]]] += 1
    flat = " ".join(str(v) for row in M for v in row)
    try:
        out = subprocess.run(
            ["python3", COMPUTE, "0.85"], input=f"{n} {flat}\n",
            capture_output=True, text=True, timeout=30, check=True).stdout
        lines = [l for l in out.splitlines() if l.strip()]
        bias = [float(v) for v in lines[0].split()]
        classic = [float(v) for v in lines[1].split()]
    except Exception as e:
        print(f"(compute_scores.py unavailable: {e})", file=sys.stderr)
        bias = classic = [0.0] * n
    return players, dict(zip(players, bias)), dict(zip(players, classic))


def side_bias(matches):
    """Does one table side win more? Two-sided binomial p-value included:
    with few matches a 60/40 split is expected noise, not bias."""
    n = len(matches)
    wins_a = sum(1 for m in matches if m["winner"] == "A")
    if n == 0:
        return {"matches": 0}
    k = max(wins_a, n - wins_a)
    p = sum(math.comb(n, i) for i in range(k, n + 1)) / 2.0 ** n * 2.0
    return {"matches": n, "side_A_wins": wins_a, "side_B_wins": n - wins_a,
            "side_A_winrate": round(wins_a / n, 3),
            "p_value": round(min(1.0, p), 4),
            "verdict": ("side bias likely real" if p < 0.05 and n >= 10
                        else "no significant side bias (yet)")}


def player_table(matches):
    stats = {}

    def st(p):
        return stats.setdefault(p, {"matches": 0, "wins": 0,
                                    "as_att": 0, "as_def": 0,
                                    "att_wins": 0, "def_wins": 0,
                                    "on_A": 0, "goals_for": 0,
                                    "goals_against": 0})

    for m in matches:
        goals = [tuple(int(x) for x in g.split("-"))
                 for g in m["games"].split(",") if "-" in g]
        gfa = sum(g[0] for g in goals)  # side A total goals
        gfb = sum(g[1] for g in goals)
        for key, role, side in (("A_def", "def", "A"), ("A_att", "att", "A"),
                                ("B_def", "def", "B"), ("B_att", "att", "B")):
            s = st(m[key])
            s["matches"] += 1
            s["as_" + role] += 1
            won = m["winner"] == side
            s["wins"] += won
            s[role + "_wins"] += won
            s["on_A"] += side == "A"
            s["goals_for"] += gfa if side == "A" else gfb
            s["goals_against"] += gfb if side == "A" else gfa
    return stats


def build_report(matches):
    stats = player_table(matches)
    elo = run_elo(matches)
    elo_def = run_elo(matches, "def")
    elo_att = run_elo(matches, "att")
    players, pr_bias, pr_classic = run_pagerank(matches)

    rows = []
    for p in players:
        s = stats[p]
        rows.append({
            "player": p, "matches": s["matches"],
            "wins": s["wins"],
            "winrate": round(s["wins"] / s["matches"], 3),
            "attacker_share": round(s["as_att"] / s["matches"], 3),
            "att_winrate": (round(s["att_wins"] / s["as_att"], 3)
                            if s["as_att"] else None),
            "def_winrate": (round(s["def_wins"] / s["as_def"], 3)
                            if s["as_def"] else None),
            "side_A_share": round(s["on_A"] / s["matches"], 3),
            "goal_diff": s["goals_for"] - s["goals_against"],
            "elo": round(elo.get(p, ELO_START), 1),
            "elo_def": (round(elo_def[p], 1) if p in elo_def else None),
            "elo_att": (round(elo_att[p], 1) if p in elo_att else None),
            "pagerank_bias": round(pr_bias.get(p, 0.0), 4),
            "pagerank": round(pr_classic.get(p, 0.0), 4),
        })
    rows.sort(key=lambda r: -r["elo"])

    mvd = [r for r in rows if r["elo_def"] is not None
           and stats[r["player"]]["as_def"] >= MIN_GAMES_MVP]
    mva = [r for r in rows if r["elo_att"] is not None
           and stats[r["player"]]["as_att"] >= MIN_GAMES_MVP]
    return {
        "players": rows,
        "side_bias": side_bias(matches),
        "most_valuable_defender": (max(mvd, key=lambda r: r["elo_def"])
                                   ["player"] if mvd else None),
        "most_valuable_attacker": (max(mva, key=lambda r: r["elo_att"])
                                   ["player"] if mva else None),
    }


def pretty(rep):
    hdr = ("player", "mt", "win%", "att%", "attW%", "defW%", "gd",
           "elo", "eloD", "eloA", "PRb")
    print(f"{hdr[0]:<14}" + "".join(f"{h:>7}" for h in hdr[1:]))

    def pc(v):
        return f"{v * 100:.0f}" if v is not None else "-"

    for r in rep["players"]:
        print(f"{r['player']:<14}{r['matches']:>7}{pc(r['winrate']):>7}"
              f"{pc(r['attacker_share']):>7}{pc(r['att_winrate']):>7}"
              f"{pc(r['def_winrate']):>7}{r['goal_diff']:>7}"
              f"{r['elo']:>7.0f}"
              f"{r['elo_def'] if r['elo_def'] is not None else '-':>7}"
              f"{r['elo_att'] if r['elo_att'] is not None else '-':>7}"
              f"{r['pagerank_bias']:>7.3f}")
    sb = rep["side_bias"]
    if sb.get("matches"):
        print(f"\nside bias: A {sb['side_A_wins']} - {sb['side_B_wins']} B "
              f"(A wins {sb['side_A_winrate'] * 100:.0f}%, "
              f"p={sb['p_value']}) -> {sb['verdict']}")
    if rep["most_valuable_defender"]:
        print(f"most valuable defender: {rep['most_valuable_defender']}")
    if rep["most_valuable_attacker"]:
        print(f"most valuable attacker: {rep['most_valuable_attacker']}")


if __name__ == "__main__":
    args = sys.argv[1:]
    path = DEFAULT_LOG
    if "--file" in args:
        path = args[args.index("--file") + 1]
    if not os.path.exists(path):
        sys.exit(f"no singles log at {path} — play a match first")
    ms = parse_log(path)
    if not ms:
        sys.exit(f"{path} contains no matches yet")
    report = build_report(ms)
    if "--json" in args:
        print(json.dumps(report, indent=1))
    else:
        print(f"{len(ms)} singles matches from {path}\n")
        pretty(report)
