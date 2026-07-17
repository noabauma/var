#!/usr/bin/env python3
"""Single-player (2v2 pickup) referee — PROTOTYPE, runs manually and never
touches the team tournament. Each PLAYER lays their own ArUco card in the
corner matching their position:

        top-left:  attacker, side A   |   top-right:  defender, side B
     bottom-left:  defender, side A   |  bottom-right: attacker, side B

(side A = the left score rail as seen by the camera; teammates stand along
the same table edge: defender by their counter, attacker at the free
corner). Exactly FOUR known player cards, one per corner, held stable for
`card_stable_s` seconds = an official singles game.

While the game runs, the live banner shows the four names and the score
(same bead reading as team mode). A finished best-of-three is appended to
singles.tsv:

    match  <iso-time>  <A_def>  <A_att>  <B_def>  <B_att>  <games>  <A|B>

Statistics (role shares, side bias, Elo, PageRank, most valuable defender)
are computed from that log by singles_stats.py.

Run:  python3 referee_singles.py           (stop with Ctrl-C)
      python3 referee_singles.py watch     (just print corner assignments)
"""
import json
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from referee import (load_cfg, detect_cards, mask_cards, count_beads,
                     learn_score_end, Stable, publish, get_frame)

HERE = os.path.dirname(os.path.abspath(__file__))
SINGLES_LOG = os.path.expanduser("~/recordings/singles.tsv")
PLAYERS_MAP = os.path.join(HERE, "cards_players/mapping.json")


def corner_of(cfg, cx, cy):
    """(side, role) from card position; side A = left rail's table edge."""
    mid_x = cfg.get("singles_mid_x", 640)
    split = cfg.get("singles_def_y", 400)
    if cx < mid_x:
        return ("A", "def") if cy >= split else ("A", "att")
    return ("B", "def") if cy < split else ("B", "att")


def assign(cfg, cards, mapping):
    """cards: {id: (center,pts)} of known players -> corner dict or None."""
    if len(cards) != 4:
        return None
    out = {}
    for pid, (ctr, _) in cards.items():
        key = corner_of(cfg, ctr[0], ctr[1])
        if key in out:
            return None  # two cards in one corner: not a valid setup
        out[key] = mapping[pid]
    return out if len(out) == 4 else None


def log_match(lineup, games, winner_side):
    new = not os.path.exists(SINGLES_LOG)
    with open(SINGLES_LOG, "a") as fh:
        if new:
            fh.write("# match\ttime\tA_def\tA_att\tB_def\tB_att\tgames\twinner\n")
        fh.write("match\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            time.strftime("%Y-%m-%dT%H:%M:%S"),
            lineup[("A", "def")], lineup[("A", "att")],
            lineup[("B", "def")], lineup[("B", "att")],
            games, winner_side))


def banner_names(lineup):
    a = f"{lineup[('A','def')]}+{lineup[('A','att')]}"
    b = f"{lineup[('B','def')]}+{lineup[('B','att')]}"
    return a, b


def main_loop(cfg, mapping):
    base = cfg["base_url"]
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, cfg["aruco_dict"]))
    period = 1.0 / cfg["poll_fps"]
    state = "IDLE"
    lineup = None
    since = None
    sa_f = Stable(cfg["confirm_frames"])
    sb_f = Stable(cfg["confirm_frames"])
    game_scores, game_hi = [], [0, 0]
    print("singles referee (PROTOTYPE) — no writes to the team tournament")

    while True:
        t0 = time.monotonic()
        try:
            img = get_frame(base, cfg)
        except Exception as e:
            print("frame:", e)
            time.sleep(2)
            continue
        detected = detect_cards(img, dictionary)
        players = {i: v for i, v in detected.items() if i in mapping}
        clean = mask_cards(img, detected)

        if state == "IDLE":
            cand = assign(cfg, players, mapping)
            if cand:
                if since is None:
                    since = t0
                    print("corners:", {f"{s}-{r}": n
                                       for (s, r), n in cand.items()})
                elif t0 - since >= cfg["card_stable_s"]:
                    lineup = cand
                    state = "MATCH"
                    game_scores, game_hi = [], [0, 0]
                    sa_f = Stable(cfg["confirm_frames"])
                    sb_f = Stable(cfg["confirm_frames"])
                    for rn in ("beads_a", "beads_b"):
                        end = learn_score_end(clean, cfg[rn], cfg)
                        if end:
                            cfg[rn]["_score_end"] = end
                    a, b = banner_names(lineup)
                    print(f"SINGLES MATCH: {a} vs {b}")
                    publish(base, True, a, b, 0, 0, 0, 0, "singles · game 1")
            else:
                since = None

        elif state in ("MATCH", "DONE"):
            if len(players) < 4:
                print("cards removed —",
                      "match recorded" if state == "DONE" else "abandoned")
                publish(base, False)
                state, lineup, since = "IDLE", None, None
                cfg["beads_a"].pop("_score_end", None)
                cfg["beads_b"].pop("_score_end", None)
                continue
            if state == "MATCH":
                ra, occa = count_beads(clean, cfg["beads_a"], cfg)
                rb, occb = count_beads(clean, cfg["beads_b"], cfg)
                if not occa:
                    sa_f.feed(min(ra, cfg["game_to"]))
                if not occb:
                    sb_f.feed(min(rb, cfg["game_to"]))
                sa, sb = sa_f.value or 0, sb_f.value or 0
                game_hi[0] = max(game_hi[0], sa)
                game_hi[1] = max(game_hi[1], sb)
                ga = sum(1 for g in game_scores if g[0] > g[1])
                gb = len(game_scores) - ga
                a, b = banner_names(lineup)
                publish(base, True, a, b, sa, sb, ga, gb,
                        f"singles · game {len(game_scores) + 1}")
                if cfg["game_to"] in (game_hi[0], game_hi[1]) and sa + sb <= 1:
                    game_scores.append(tuple(game_hi))
                    game_hi = [0, 0]
                    sa_f = Stable(cfg["confirm_frames"])
                    sb_f = Stable(cfg["confirm_frames"])
                    ga = sum(1 for g in game_scores if g[0] > g[1])
                    gb = len(game_scores) - ga
                    print(f"game {len(game_scores)}: {game_scores[-1]} "
                          f"— games {ga}-{gb}")
                    if max(ga, gb) >= cfg["games_to_win"]:
                        games = ",".join(f"{x}-{y}" for x, y in game_scores)
                        winner = "A" if ga > gb else "B"
                        log_match(lineup, games, winner)
                        print(f"MATCH OVER: side {winner} wins ({games}) — "
                              f"logged to {SINGLES_LOG}")
                        publish(base, True, a, b, 0, 0, ga, gb,
                                "singles over — remove the cards")
                        state = "DONE"
        dt = time.monotonic() - t0
        time.sleep(max(0.05, period - dt))


def watch(cfg, mapping):
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, cfg["aruco_dict"]))
    while True:
        try:
            img = get_frame(cfg["base_url"], cfg)
            detected = detect_cards(img, dictionary)
            players = {i: v for i, v in detected.items() if i in mapping}
            cand = assign(cfg, players, mapping)
            print("cards:", {mapping[i]: tuple(round(v) for v in c)
                             for i, (c, _) in players.items()},
                  "->", {f"{s}-{r}": n for (s, r), n in cand.items()}
                  if cand else "(no valid 4-corner lineup)")
        except Exception as e:
            print("(", e, ")")
        time.sleep(1.0)


if __name__ == "__main__":
    config = load_cfg()
    if not os.path.exists(PLAYERS_MAP):
        sys.exit("no player cards yet — run gen_player_cards.py first")
    pmap = {int(k): v for k, v in json.load(open(PLAYERS_MAP)).items()}
    print(f"players: {len(pmap)} cards mapped")
    if len(sys.argv) > 1 and sys.argv[1] == "watch":
        watch(config, pmap)
    else:
        main_loop(config, pmap)
