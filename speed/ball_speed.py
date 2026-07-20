#!/usr/bin/env python3
"""Ball-speed analyzer for saved slow-mo clips (goal detector).

After the recorder finishes transcoding a clip to MP4 it runs
    python3 speed/ball_speed.py analyze <clip>
which decides "was that a goal?" and if so how fast the shot was, and
writes the verdict to <clip minus extension>.speed.json next to the clip.
The web player reads that sidecar and overlays the speed during replay.

How it works (all measured on this table's real clips):
  calibration  A stored quad maps the playfield to table_mm coordinates
               (speed_cal.json + ref_bg.jpg, created by `calibrate`).
               Each clip's median background is aligned to the reference
               with ECC (rotation+shift), so small camera bumps are
               absorbed; if the view moved too much the clip is skipped
               (no speed is better than a wrong speed).
  ball         median background subtraction; the ball is the bright,
               round, ball-sized moving blob inside the field quad.
               Candidates are linked into velocity-gated tracklets.
  goal         a tracklet that dies at a goal mouth and does not
               reappear nearby = the ball left the field there.
  speed        peak 3-frame window speed of that tracklet in mm-space
               (the shot), at capture_fps (120), reported in m/s.

Set table_mm to your tape-measured playfield (length, width between the
inner walls) for exact numbers — the default is the standard 1200x680.

Usage:
  ball_speed.py analyze <clip.mp4> [--force]   write sidecar (skips if fresh)
  ball_speed.py calibrate <clip.mp4|image.jpg> store new reference pose
  ball_speed.py debug <clip.mp4>               verbose, saves overlay jpg
"""
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CAL_PATH = os.path.join(HERE, "speed_cal.json")
REF_BG = os.path.join(HERE, "ref_bg.jpg")

DEFAULT_CAL = {
    # playfield corners TL,TR,BR,BL in the reference image (calibrated on
    # the 2026-07-20 camera pose via warped-marking symmetry)
    "quad": [[290, 205], [930, 185], [945, 580], [277, 603]],
    "table_mm": [1200, 680],   # playfield length x width, inner walls
    "goal_half_mm": 115,       # goal mouth half-width + margin
    "capture_fps": 120.0,
    "min_ecc": 0.5,            # weaker background match = camera moved
    "diff_thresh": 35,         # background-difference threshold
    "ball_bright": 110,        # the ball is white: min pixel brightness
    "ball_area": [50, 900],    # blob area (px^2) accepted as the ball
    "min_speed_ms": 1.0,       # slower "shots" into the goal don't count
}


def load_cal():
    cal = dict(DEFAULT_CAL)
    if os.path.exists(CAL_PATH):
        cal.update(json.load(open(CAL_PATH)))
    return cal


def load_gray(path, step=1):
    cap = cv2.VideoCapture(path)
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
    cap.release()
    return frames


def median_bg(frames, n=25):
    idx = np.linspace(0, len(frames) - 1, min(n, len(frames))).astype(int)
    return np.median(np.stack([frames[i] for i in idx]), axis=0).astype(np.uint8)


def align_quad(cal, bg):
    """ECC-align the stored reference pose to this clip's background and
    move the calibration quad along. -> (quad, ecc) or (None, ecc)."""
    ref = cv2.imread(REF_BG, 0)
    if ref is None:
        return None, 0.0
    if ref.shape != bg.shape:
        return None, 0.0
    sr = cv2.resize(ref, None, fx=0.5, fy=0.5)
    sn = cv2.resize(bg, None, fx=0.5, fy=0.5)
    warp = np.eye(2, 3, dtype=np.float32)
    try:
        ecc, warp = cv2.findTransformECC(
            sr, sn, warp, cv2.MOTION_EUCLIDEAN,
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 200, 1e-5),
            None, 5)
    except cv2.error:
        return None, 0.0
    if ecc < cal["min_ecc"]:
        return None, float(ecc)
    W = warp.copy()
    W[:, 2] *= 2.0
    quad = cv2.transform(np.float32(cal["quad"])[None], W)[0]
    return quad, float(ecc)


class Tracklet:
    def __init__(self, fi, pt):
        self.pts = {fi: pt}
        self.last_f, self.last_p, self.vel = fi, pt, (0.0, 0.0)

    def predict(self, fi):
        dt = fi - self.last_f
        return (self.last_p[0] + self.vel[0] * dt,
                self.last_p[1] + self.vel[1] * dt)

    def add(self, fi, pt):
        dt = fi - self.last_f
        v = ((pt[0] - self.last_p[0]) / dt, (pt[1] - self.last_p[1]) / dt)
        a = 0.5 if len(self.pts) > 2 else 1.0
        self.vel = (a * v[0] + (1 - a) * self.vel[0],
                    a * v[1] + (1 - a) * self.vel[1])
        self.pts[fi] = pt
        self.last_f, self.last_p = fi, pt


def ball_candidates(frames, bg, quad, cal):
    """Per frame: [(x, y, r)] bright round moving blobs inside the field."""
    mask = np.zeros(bg.shape, np.uint8)
    cv2.fillConvexPoly(mask, quad.astype(np.int32), 255)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    a_lo, a_hi = cal["ball_area"]
    out = []
    for g in frames:
        d = cv2.absdiff(g, bg)
        d[mask == 0] = 0
        _, bw = cv2.threshold(d, cal["diff_thresh"], 255, cv2.THRESH_BINARY)
        bw[g < cal["ball_bright"]] = 0
        bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kern)
        cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        cur = []
        for c in cnts:
            a = cv2.contourArea(c)
            if not a_lo <= a <= a_hi:
                continue
            per = cv2.arcLength(c, True)
            circ = 4 * np.pi * a / (per * per) if per > 0 else 0
            (x, y), r = cv2.minEnclosingCircle(c)
            fill = a / (np.pi * r * r + 1e-9)
            if circ > 0.5 and fill > 0.55:
                cur.append((float(x), float(y), float(r)))
        out.append(cur)
    return out


def link(cands, gate=70.0, coast=6):
    open_t, done = [], []
    for fi, cur in enumerate(cands):
        used = set()
        for t in open_t:
            px, py = t.predict(fi)
            best, bd = None, 1e9
            for j, p in enumerate(cur):
                if j not in used:
                    dd = np.hypot(p[0] - px, p[1] - py)
                    if dd < bd:
                        best, bd = j, dd
            if best is not None and bd <= gate:
                t.add(fi, cur[best][:2])
                used.add(best)
        keep = []
        for t in open_t:
            (keep if fi - t.last_f <= coast else done).append(t)
        open_t = keep
        for j, p in enumerate(cur):
            if j not in used:
                open_t.append(Tracklet(fi, p[:2]))
    return [t for t in done + open_t if len(t.pts) >= 6]


def analyse(path, debug=False):
    cal = load_cal()
    frames = load_gray(path)
    if len(frames) < 60:
        return {"ok": False, "why": "clip too short"}
    bg = median_bg(frames)
    quad, ecc = align_quad(cal, bg)
    if quad is None:
        return {"ok": False, "why": "camera view changed since calibration",
                "ecc": round(ecc, 3)}
    L, Wmm = cal["table_mm"]
    H = cv2.getPerspectiveTransform(
        quad, np.float32([[0, 0], [L, 0], [L, Wmm], [0, Wmm]]))

    cands = ball_candidates(frames, bg, quad, cal)
    tracks = link(cands)

    def mm(pt):
        v = cv2.perspectiveTransform(np.float32([[pt]]), H)[0][0]
        return float(v[0]), float(v[1])

    fps = cal["capture_fps"]
    best = None
    infos = []
    for t in tracks:
        fs = sorted(t.pts)
        pos = {f: mm(t.pts[f]) for f in fs}
        speeds = []
        for i in range(len(fs) - 3):
            f0, f1 = fs[i], fs[i + 3]
            if f1 - f0 > 6:
                continue
            d = np.hypot(pos[f1][0] - pos[f0][0], pos[f1][1] - pos[f0][1])
            speeds.append((f1, d / ((f1 - f0) / fps) / 1000.0))
        if not speeds:
            continue
        end = pos[fs[-1]]
        side = "left" if end[0] < 40 else ("right" if end[0] > L - 40 else "")
        at_goal = bool(side) and abs(end[1] - Wmm / 2) < cal["goal_half_mm"]
        # a goal swallows the ball. A shot that BOUNCES off the goal frame
        # reappears near the goal mouth within a few frames — so scan a wide
        # radius for a short window. (Radius must not reach midfield: after
        # a real goal the next ball often gets placed there within the same
        # clip, which must not cancel the verdict.)
        gone = True
        ep = t.pts[fs[-1]]
        for fj in range(fs[-1] + 1, min(fs[-1] + int(fps * 0.4), len(cands))):
            for p in cands[fj]:
                if np.hypot(p[0] - ep[0], p[1] - ep[1]) < 150:
                    gone = False
                    break
        # the shot = the last stretch before the goal; its peak window speed
        tail = [s for f, s in speeds if f >= fs[-1] - 20]
        shot = max(tail) if tail else max(s for _, s in speeds)
        info = {"frames": [fs[0], fs[-1]], "n": len(fs),
                "peak_ms": round(shot, 2), "end_mm": [round(e) for e in end],
                "goal": bool(at_goal and gone and shot >= cal["min_speed_ms"]),
                "side": side}
        infos.append(info)
        if info["goal"] and (best is None or shot > best["speed_ms"]):
            best = {"goal": True, "speed_ms": round(shot, 2),
                    "speed_kmh": round(shot * 3.6, 1),
                    "side": side, "goal_frame": fs[-1]}
    res = {"ok": True, "ecc": round(ecc, 3),
           "goal": bool(best), "n_tracks": len(infos)}
    if best:
        res.update(best)
    if debug:
        res["tracks"] = sorted(infos, key=lambda i: -i["peak_ms"])[:8]
        vis = cv2.cvtColor(bg, cv2.COLOR_GRAY2BGR)
        cv2.polylines(vis, [quad.astype(np.int32)], True, (0, 255, 0), 2)
        for t in tracks:
            fs = sorted(t.pts)
            for i in range(1, len(fs)):
                cv2.line(vis, tuple(map(int, t.pts[fs[i - 1]])),
                         tuple(map(int, t.pts[fs[i]])), (0, 255, 255), 2)
        dbg = os.path.splitext(path)[0] + ".speed_debug.jpg"
        cv2.imwrite(dbg, vis)
        res["debug_jpg"] = dbg
    return res


def sidecar_path(clip):
    return os.path.splitext(clip)[0] + ".speed.json"


def cmd_analyze(clip, force=False):
    sc = sidecar_path(clip)
    if not force and os.path.exists(sc) and \
            os.path.getmtime(sc) >= os.path.getmtime(clip):
        print(f"fresh sidecar exists: {sc}")
        return
    res = analyse(clip)
    tmp = sc + ".tmp"
    json.dump(res, open(tmp, "w"))
    os.replace(tmp, sc)
    print(json.dumps(res))


def cmd_calibrate(src):
    """Store the current pose: a clip (median bg) or a still image."""
    if src.lower().endswith((".jpg", ".jpeg", ".png")):
        bg = cv2.imread(src, 0)
    else:
        bg = median_bg(load_gray(src))
    cv2.imwrite(REF_BG, bg)
    cal = load_cal()
    json.dump(cal, open(CAL_PATH, "w"), indent=1)
    print(f"reference pose stored ({REF_BG}).")
    print("If the camera moved a lot, the quad in speed_cal.json must be")
    print("re-measured for the new pose (corners TL,TR,BR,BL of the field).")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    cmd, target = sys.argv[1], sys.argv[2]
    if cmd == "analyze":
        cmd_analyze(target, force="--force" in sys.argv)
    elif cmd == "debug":
        print(json.dumps(analyse(target, debug=True), indent=1))
    elif cmd == "calibrate":
        cmd_calibrate(target)
    else:
        sys.exit(__doc__)
