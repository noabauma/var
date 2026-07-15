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
    "auto_open_betting": False,  # side effect on the real board — off while testing
    "auto_enter": False,         # testing phase: log results, never write them
    # measured on the rotated table, 2026-07-15 (mono 1280x720).
    # The boxes are generous search windows (the table shifts a little
    # during play) — the bead band is located inside them every frame.
    # Each rail carries 12 beads of which the outermost one at each end
    # is decoration (deco_ends): the scoring-end group is reduced by one.
    # The table rotation makes rail A read point-symmetrically to B:
    # A scores toward high y, B toward low y.
    "beads_a": {"x": 195, "y": 260, "w": 105, "h": 270, "axis": "y",
                "score_end": "high", "thresh": 80, "total_beads": 12,
                "band_px": 36, "rail_px": 270, "pitch_px": 15.5, "deco_ends": True},
    "beads_b": {"x": 1030, "y": 230, "w": 110, "h": 285, "axis": "y",
                "score_end": "low", "thresh": 95, "total_beads": 12,
                "band_px": 36, "rail_px": 270, "pitch_px": 15, "deco_ends": True},
    "row_frac": 0.25,                   # band row counts as "bead" above this
    "occlusion_frac": 0.6,              # band covered more than this = hand
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
    """Returns {marker_id: (center_xy, corner_pts)} for every marker."""
    corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary)
    out = {}
    if ids is not None:
        for c, i in zip(corners, ids.flatten()):
            pts = c[0]
            out[int(i)] = (tuple(pts.mean(axis=0)), pts)
    return out


def mask_cards(img, detected):
    """Blacks out every detected card using its *oriented* print geometry
    (A5 landscape, marker on the right, digit to the left — gen_cards.py),
    so tilted cards are covered exactly: no white paper leaks into the
    bead count, and no rail pixels beyond the card are eaten."""
    out = img.copy()
    for _, (_, pts) in detected.items():
        c = pts.mean(axis=0)
        # unit-side vectors: edge midpoint minus center is half a side long
        right = ((pts[1] + pts[2]) / 2.0 - c) * 2.0
        up = ((pts[0] + pts[1]) / 2.0 - c) * 2.0
        # in marker-side units the card spans right +0.61 / left -1.65 /
        # up&down 0.80 around the marker center; ~10% print/detect margin
        poly = np.array([c + 0.72 * right + 0.82 * up,
                         c + 0.72 * right - 0.82 * up,
                         c - 1.85 * right - 0.82 * up,
                         c - 1.85 * right + 0.82 * up])
        cv2.fillConvexPoly(out, poly.astype(np.int32), 0)
    return out


def count_beads(img, roi, cfg):
    """Counts beads on the scoring side of the rail ROI.

    Touching beads merge into one blob, so blobs are useless — instead the
    bright mask is projected onto the rail axis into bead/no-bead runs.
    The rail always carries `total_beads`, so the per-frame bead pitch is
    (total run length / total_beads); the run group on the `score_end`
    side of the largest gap, divided by the pitch, is the score.
    Returns (score, occluded).
    """
    x, y, w, h = roi["x"], roi["y"], roi["w"], roi["h"]
    crop = img[y:y + h, x:x + w]
    if crop.size == 0:
        return 0, True
    # per-frame Otsu threshold: the camera's auto-exposure shifts constantly
    # with people moving, so a fixed threshold goes stale within minutes.
    # roi["thresh"] acts as a floor so a beadless dark strip can't split.
    otsu, _ = cv2.threshold(crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    bw = crop >= max(otsu * 0.85, roi.get("thresh", 60) * 0.6)
    # solid bright stripes (the white table edge) fill the whole window
    # height; beads never do — mask such columns out before locating
    if roi["axis"] == "y":
        bw[:, bw.mean(axis=0) > 0.85] = False
    else:
        bw[bw.mean(axis=1) > 0.85, :] = False
    # the ROI is a search window: locate the bead band inside it, so a
    # shifted table doesn't need recalibration. A Gaussian prior centered
    # on the window keeps nearby bright stripes (table edges) from winning.
    band = roi.get("band_px", 36)
    axis_y = roi["axis"] == "y"
    n_cross = bw.shape[1] if axis_y else bw.shape[0]
    # scan every candidate band position and keep the one whose bead-row
    # total is closest to the physical expectation (total_beads * pitch):
    # deterministic, memory-free, and edge stripes/junk can't qualify
    target = roi.get("total_beads", 12) * roi.get("pitch_px", 19)
    cands = []  # (longest_run, -err, offset, on-rows) per candidate band
    for o in range(0, max(1, n_cross - band + 1), 2):
        sub = bw[:, o:o + band] if axis_y else bw[o:o + band, :]
        prof = sub.mean(axis=1 if axis_y else 0)
        on_c = (prof > cfg["row_frac"]) & (prof < 0.92)
        err = abs(int(on_c.sum()) - target)
        if err <= target * 0.30:
            # longest consecutive run = the bead cluster; junk is speckle
            longest = cur = 0
            for v in on_c:
                cur = cur + 1 if v else 0
                longest = max(longest, cur)
            cands.append((longest, -err, o, on_c))
    if not cands:
        return 0, True  # no believable rail in the window (occlusion/move)
    _, nerr, b0, on = max(cands)
    err = -nerr
    sub = bw[:, b0:b0 + band] if axis_y else bw[b0:b0 + band, :]
    frac = float(np.count_nonzero(sub)) / sub.size
    if frac > cfg["occlusion_frac"]:
        return 0, True  # something large covers the rail (a hand/arm)
    runs = []  # (start, length) of consecutive bead rows
    start = None
    for i, v in enumerate(on):
        if v and start is None:
            start = i
        elif not v and start is not None:
            runs.append((start, i - start))
            start = None
    if start is not None:
        runs.append((start, len(on) - start))
    runs = [r for r in runs if r[1] >= 4]  # drop specks
    if not runs:
        return 0, False
    # the physical rail is rail_px long (deco bead to deco bead): keep only
    # the best rail-length span of runs — kills edges/handles at the window
    # borders no matter what they look like
    span = roi.get("rail_px", 0)
    if span and (runs[-1][0] + runs[-1][1] - runs[0][0]) > span:
        conv = np.convolve(on.astype(float), np.ones(span), "valid")
        o = int(np.argmax(conv))
        clipped = []
        for s, l in runs:
            s2, e2 = max(s, o), min(s + l, o + span)
            if e2 - s2 >= 4:
                clipped.append((s2, e2 - s2))
        runs = clipped
        if not runs:
            return 0, False
    total_len = sum(l for _, l in runs)
    pitch = total_len / max(1, roi.get("total_beads", 12))
    if not (13 <= pitch <= 26):
        return 0, True  # implausible bead size: not a clean rail reading
    # the full bead count must be accounted for — if any beads are hidden
    # (mask overlap, hand, bad light) refuse to read rather than misread
    expected = roi.get("total_beads", 12)
    seen = total_len / float(roi.get("pitch_px", 19))
    if abs(seen - expected) > 1.5:
        return 0, True
    deco = 1 if roi.get("deco_ends") else 0
    if len(runs) == 1:
        return 0, False  # single cluster = nothing slid out = score 0
    # split the runs at the largest inter-run gap
    gaps = [runs[i + 1][0] - (runs[i][0] + runs[i][1])
            for i in range(len(runs) - 1)]
    gi = int(np.argmax(gaps))
    low_len = sum(l for _, l in runs[:gi + 1])
    high_len = total_len - low_len
    seg = low_len if roi["score_end"] == "low" else high_len
    # the outermost bead at the scoring end is decoration, not a goal
    return max(0, int(round(seg / pitch)) - deco), False


class Stable:
    """A value that must repeat `n` times in a row before it is believed.
    Implausible jumps (more than 2 goals at once — junk readings from a
    bumped table or partial occlusion) need twice the confirmations."""

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
        need = self.n
        if self.value is not None and abs(v - self.value) > 2:
            need = self.n * 2
        if self.count >= need:
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

        detected = detect_cards(img, dictionary)
        clean = mask_cards(img, detected)  # cards never pollute the rails
        cards = {i: p for i, (p, _) in detected.items() if i in mapping}

        if state == "IDLE":
            if len(cards) == 2:
                if cards_since is None:
                    cards_since = t0
                    ids = sorted(cards)
                    print("cards seen:", [mapping[i] for i in ids])
                elif t0 - cards_since >= cfg["card_stable_s"]:
                    # the card physically closer to rail A belongs to side A
                    ra, rb = cfg["beads_a"], cfg["beads_b"]
                    ca = (ra["x"] + ra["w"] / 2, ra["y"] + ra["h"] / 2)
                    def dist(i, c):
                        return ((cards[i][0] - c[0]) ** 2 +
                                (cards[i][1] - c[1]) ** 2) ** 0.5
                    ids = sorted(cards, key=lambda i: dist(i, ca))
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
                ra, occa = count_beads(clean, cfg["beads_a"], cfg)
                rb, occb = count_beads(clean, cfg["beads_b"], cfg)
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
                        if cfg.get("auto_enter", False):
                            q = urllib.parse.urlencode({"a": teams[0],
                                                        "b": teams[1],
                                                        "games": games})
                            try:
                                resp = json.loads(http(base,
                                                       "/scores/add?" + q,
                                                       post=True))
                                print("result entered:", teams, games, resp)
                            except Exception as e:
                                print("auto-entry failed:", e)
                            note = "match over — remove the cards"
                        else:
                            print(f"TEST MODE — would enter: {teams[0]} vs "
                                  f"{teams[1]} games {games} (auto_enter off)")
                            note = "match over (test mode: not entered)"
                        publish(base, True, teams[0], teams[1], 0, 0, ga, gb,
                                note)
                        state = "DONE"

        dt = time.monotonic() - t0
        time.sleep(max(0.05, period - dt))


def calibrate(cfg):
    img = get_frame(cfg["base_url"])
    out = os.path.join(HERE, "calibration.jpg")
    cv2.imwrite(out, img)
    # exactly the runtime pipeline: detect the cards and mask them out
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, cfg["aruco_dict"]))
    detected = detect_cards(img, dictionary)
    clean = mask_cards(img, detected)
    # draw the current ROIs and detected cards for reference
    vis = cv2.cvtColor(clean, cv2.COLOR_GRAY2BGR)
    for name, color in (("beads_a", (0, 255, 0)), ("beads_b", (0, 0, 255))):
        r = cfg[name]
        cv2.rectangle(vis, (r["x"], r["y"]),
                      (r["x"] + r["w"], r["y"] + r["h"]), color, 2)
        cv2.putText(vis, name, (r["x"], r["y"] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    for mid, (ctr, _) in detected.items():
        cv2.putText(vis, f"card {mid}", (int(ctr[0]) - 30, int(ctr[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 220), 2)
    cv2.imwrite(os.path.join(HERE, "calibration_rois.jpg"), vis)
    print(f"wrote {out} and calibration_rois.jpg — adjust the beads_* ROIs "
          f"in referee.json until each box tightly frames one bead rail, "
          f"then rerun calibrate to check")
    if detected:
        print("cards detected:", sorted(detected))
    ra, oa = count_beads(clean, cfg["beads_a"], cfg)
    rb, ob = count_beads(clean, cfg["beads_b"], cfg)
    print(f"current reading: A={ra}{' (occluded)' if oa else ''} "
          f"B={rb}{' (occluded)' if ob else ''}")


def watch(cfg):
    """Live readings once per second — slide beads at the table and check
    that the numbers follow. '?' marks a rejected (occluded/unclear) read."""
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, cfg["aruco_dict"]))
    print("watching (Ctrl-C to stop) — A | B | cards")
    while True:
        try:
            img = get_frame(cfg["base_url"])
            detected = detect_cards(img, dictionary)
            clean = mask_cards(img, detected)
            a, oa = count_beads(clean, cfg["beads_a"], cfg)
            b, ob = count_beads(clean, cfg["beads_b"], cfg)
            print(f"A={a}{'?' if oa else ' '}  B={b}{'?' if ob else ' '}  "
                  f"cards={sorted(detected)}")
        except Exception as e:
            print("(", e, ")")
        time.sleep(1.0)


if __name__ == "__main__":
    config = load_cfg()
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        calibrate(config)
    elif len(sys.argv) > 1 and sys.argv[1] == "watch":
        watch(config)
    else:
        main_loop(config)
