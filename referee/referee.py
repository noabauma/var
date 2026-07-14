#!/usr/bin/env python3
"""CV referee: watches the table through the recorder's /frame endpoint,
detects which teams are playing (ArUco marker cards laid below the score
rails), reads both bead counters, shows the live score on the web page,
and enters the finished best-of-three into the tournament automatically.

Pipeline (runs at ~2 fps, classic CV only — no ML):

  /frame (JPEG) ──> ArUco detect ──> match mode?  ──> bead count per rail
                                              │             │
        /match/state (live banner)  <─────────┴─────────────┤
        /scores/add (auto result)   <── best-of-three state machine

States:
  IDLE        no (or unstable) marker cards on the table
  MATCH       two known cards stable for `card_stable_s` seconds
              → teams identified, optional auto-open of betting
  (in MATCH)  per-game score tracking; a game ends when a rail reaches 10,
              the next game starts when both rails return to ~0
  DONE        a team wins its 2nd game → result POSTed, waits until the
              cards are removed, then back to IDLE

Setup once:
  1. print the cards:            python3 gen_cards.py
  2. grab a calibration frame:   python3 referee.py calibrate
     → writes calibration.jpg; fill the two `beads_*` ROIs in referee.json
       (x, y, w, h in frame pixels around each bead rail — the rail only,
       no handles) and `score_end`: which end of the rail counts as scored
       ("low" = beads slide toward smaller x/y when scoring).
  3. run it:                     python3 referee.py
     (or enable the systemd unit: deploy/slowmo-referee.service)

Everything tunable lives in referee.json. The state machine only trusts a
reading that is stable across `confirm_frames` consecutive frames, scores
within a game only go up, and frames where the rail ROI looks occluded
(hand over the counter) are skipped — so a waving hand can't score a goal.
"""
import json
import os
import sys
import time
import urllib.parse
import urllib.request

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(HERE, "referee.json")

DEFAULT_CFG = {
    "base_url": "http://127.0.0.1:8081",
    "poll_fps": 2.0,
    "aruco_dict": "DICT_4X4_50",
    "cards_map": "cards/mapping.json",  # marker id -> team name (gen_cards.py)
    "card_stable_s": 5.0,               # cards must sit this long to arm
    "confirm_frames": 3,                # consecutive identical readings
    "game_to": 10,                      # a game ends at this score
    "games_to_win": 2,                  # best of three
    "auto_open_betting": True,
    "beads_a": {"x": 265, "y": 300, "w": 60, "h": 220, "axis": "y",
                "score_end": "high"},
    "beads_b": {"x": 265, "y": 470, "w": 60, "h": 220, "axis": "y",
                "score_end": "high"},
    "bead_min_px": 40,                  # min blob area (px) to count as bead
    "bead_max_px": 900,
    "bright_thresh": 150,               # bead brightness threshold (mono)
    "occlusion_frac": 0.5,              # ROI covered more than this = hand
}


def load_cfg():
    cfg = dict(DEFAULT_CFG)
    if os.path.exists(CFG_PATH):
        cfg.update(json.load(open(CFG_PATH)))
    else:
        json.dump(cfg, open(CFG_PATH, "w"), indent=2)
        print(f"wrote default {CFG_PATH} — edit the beads_* ROIs after "
              f"running: python3 referee.py calibrate")
    return cfg


def http(base, path, post=False):
    req = urllib.request.Request(base + path, method="POST" if post else "GET")
    with urllib.request.urlopen(req, timeout=10) as r:
        return r.read()


def get_frame(base):
    data = http(base, "/frame")
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    return img


# --------------------------------------------------------------- detection

def detect_cards(img, dictionary):
    """Returns {marker_id: (cx, cy)} for every detected marker."""
    corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary)
    out = {}
    if ids is not None:
        for c, i in zip(corners, ids.flatten()):
            out[int(i)] = tuple(c[0].mean(axis=0))
    return out


def count_beads(img, roi, cfg):
    """Counts beads on the scoring side of the rail ROI.

    Beads are bright blobs on the dark rail. They are clustered by the
    largest gap along the rail axis; the group at the `score_end` end is
    the score. Returns (score, occluded).
    """
    x, y, w, h = roi["x"], roi["y"], roi["w"], roi["h"]
    crop = img[y:y + h, x:x + w]
    if crop.size == 0:
        return 0, True
    _, bw = cv2.threshold(crop, cfg["bright_thresh"], 255, cv2.THRESH_BINARY)
    frac = float(np.count_nonzero(bw)) / bw.size
    if frac > cfg["occlusion_frac"]:
        return 0, True  # something big and bright covers the rail (a hand)
    n, _, stats, cents = cv2.connectedComponentsWithStats(bw)
    beads = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if cfg["bead_min_px"] <= area <= cfg["bead_max_px"]:
            c = cents[i]
            beads.append(c[1] if roi["axis"] == "y" else c[0])
    if not beads:
        return 0, False
    beads.sort()
    if len(beads) == 1:
        # a single blob: several touching beads can merge — treat as unknown
        # cluster on the non-scoring side = 0
        return 0, False
    # split at the largest gap along the axis
    gaps = [b - a for a, b in zip(beads, beads[1:])]
    gi = int(np.argmax(gaps))
    typical = np.median([g for g in gaps if g > 0]) or 1.0
    if gaps[gi] < 2.5 * typical:
        return 0, False  # no real gap: all beads in one cluster = score 0
    low_group = gi + 1
    high_group = len(beads) - low_group
    return (high_group if roi["score_end"] == "high" else low_group), False


class Stable:
    """A value that must repeat `n` times in a row before it is believed."""

    def __init__(self, n):
        self.n = n
        self.cand = None
        self.count = 0
        self.value = None

    def feed(self, v):
        if v == self.cand:
            self.count += 1
        else:
            self.cand = v
            self.count = 1
        if self.count >= self.n:
            self.value = v
        return self.value


# ------------------------------------------------------------ state machine

def publish(base, on, a="", b="", sa=0, sb=0, ga=0, gb=0, note=""):
    q = urllib.parse.urlencode({"on": 1 if on else 0, "a": a, "b": b,
                                "sa": sa, "sb": sb, "ga": ga, "gb": gb,
                                "note": note})
    try:
        http(base, "/match/state?" + q, post=True)
    except Exception as e:
        print("publish failed:", e)


def main_loop(cfg):
    base = cfg["base_url"]
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, cfg["aruco_dict"]))
    mapping = {}
    mpath = os.path.join(HERE, cfg["cards_map"])
    if os.path.exists(mpath):
        mapping = {int(k): v for k, v in json.load(open(mpath)).items()}
        print(f"cards: {len(mapping)} markers mapped to teams")
    else:
        print(f"warning: {mpath} missing — run gen_cards.py; markers will be ignored")

    period = 1.0 / cfg["poll_fps"]
    state = "IDLE"
    teams = None            # (name_a, name_b) once armed
    cards_since = None
    sa_f = Stable(cfg["confirm_frames"])
    sb_f = Stable(cfg["confirm_frames"])
    game_scores = []        # [(a, b), ...] finished games
    game_hi = [0, 0]        # running max within the current game
    print("referee running — state IDLE")

    while True:
        t0 = time.monotonic()
        try:
            img = get_frame(base)
        except Exception as e:
            print("frame fetch failed:", e)
            time.sleep(2)
            continue

        cards = {i: p for i, p in detect_cards(img, dictionary).items()
                 if i in mapping}

        if state == "IDLE":
            if len(cards) == 2:
                if cards_since is None:
                    cards_since = t0
                    ids = sorted(cards)
                    print("cards seen:", [mapping[i] for i in ids])
                elif t0 - cards_since >= cfg["card_stable_s"]:
                    ids = sorted(cards, key=lambda i: cards[i][1])  # by y:
                    # the card nearer beads_a belongs to side A
                    ya = cfg["beads_a"]["y"]
                    ids = sorted(cards, key=lambda i: abs(cards[i][1] - ya))
                    a_id, b_id = ids[0], ids[1]
                    teams = (mapping[a_id], mapping[b_id])
                    state = "MATCH"
                    game_scores, game_hi = [], [0, 0]
                    sa_f = Stable(cfg["confirm_frames"])
                    sb_f = Stable(cfg["confirm_frames"])
                    print(f"MATCH MODE: {teams[0]} vs {teams[1]}")
                    publish(base, True, teams[0], teams[1], 0, 0, 0, 0,
                            "match detected")
                    if cfg["auto_open_betting"]:
                        try:
                            q = urllib.parse.urlencode({"a": teams[0],
                                                        "b": teams[1]})
                            http(base, "/bets/open?" + q, post=True)
                        except Exception:
                            pass  # pair already played / betting disabled
            else:
                cards_since = None

        elif state in ("MATCH", "DONE"):
            if len(cards) < 2:
                # cards removed: abandon (MATCH) or finish (DONE)
                print("cards removed —", "match done" if state == "DONE"
                      else "match abandoned")
                publish(base, False)
                state = "IDLE"
                teams = None
                cards_since = None
                continue

            if state == "MATCH":
                ra, occa = count_beads(img, cfg["beads_a"], cfg)
                rb, occb = count_beads(img, cfg["beads_b"], cfg)
                if not occa:
                    sa_f.feed(min(ra, cfg["game_to"]))
                if not occb:
                    sb_f.feed(min(rb, cfg["game_to"]))
                sa = sa_f.value or 0
                sb = sb_f.value or 0

                # within a game the score only rises; a drop back toward
                # 0-0 after game point is the bead reset for the next game
                game_hi[0] = max(game_hi[0], sa)
                game_hi[1] = max(game_hi[1], sb)
                ga = sum(1 for g in game_scores if g[0] > g[1])
                gb = len(game_scores) - ga
                publish(base, True, teams[0], teams[1], sa, sb, ga, gb,
                        f"game {len(game_scores) + 1}")

                if cfg["game_to"] in (game_hi[0], game_hi[1]) and \
                        sa + sb <= 1:  # rails were reset -> game concluded
                    game_scores.append(tuple(game_hi))
                    game_hi = [0, 0]
                    sa_f = Stable(cfg["confirm_frames"])
                    sb_f = Stable(cfg["confirm_frames"])
                    ga = sum(1 for g in game_scores if g[0] > g[1])
                    gb = len(game_scores) - ga
                    print(f"game {len(game_scores)} done: "
                          f"{game_scores[-1]} — games {ga}-{gb}")
                    if max(ga, gb) >= cfg["games_to_win"]:
                        games = ",".join(f"{a}-{b}" for a, b in game_scores)
                        q = urllib.parse.urlencode({"a": teams[0],
                                                    "b": teams[1],
                                                    "games": games})
                        try:
                            resp = json.loads(http(base, "/scores/add?" + q,
                                                   post=True))
                            print("result entered:", teams, games, resp)
                        except Exception as e:
                            print("auto-entry failed:", e)
                        publish(base, True, teams[0], teams[1], 0, 0, ga, gb,
                                "match over — remove the cards")
                        state = "DONE"

        dt = time.monotonic() - t0
        time.sleep(max(0.05, period - dt))


def calibrate(cfg):
    img = get_frame(cfg["base_url"])
    out = os.path.join(HERE, "calibration.jpg")
    cv2.imwrite(out, img)
    # draw the current ROIs for reference
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for name, color in (("beads_a", (0, 255, 0)), ("beads_b", (0, 0, 255))):
        r = cfg[name]
        cv2.rectangle(vis, (r["x"], r["y"]),
                      (r["x"] + r["w"], r["y"] + r["h"]), color, 2)
        cv2.putText(vis, name, (r["x"], r["y"] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.imwrite(os.path.join(HERE, "calibration_rois.jpg"), vis)
    print(f"wrote {out} and calibration_rois.jpg — adjust the beads_* ROIs "
          f"in referee.json until each box tightly frames one bead rail, "
          f"then rerun calibrate to check")
    ra, oa = count_beads(img, cfg["beads_a"], cfg)
    rb, ob = count_beads(img, cfg["beads_b"], cfg)
    print(f"current reading: A={ra}{' (occluded)' if oa else ''} "
          f"B={rb}{' (occluded)' if ob else ''}")


if __name__ == "__main__":
    config = load_cfg()
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        calibrate(config)
    else:
        main_loop(config)
