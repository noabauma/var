#!/usr/bin/env python3
"""Generates one printable card per team: a huge human-readable number next
to an ArUco marker (DICT_4X4_50) that the referee reads. Print them big
(A5 works), ideally on stiff paper; matte beats glossy (no glare).

Teams come from the live scoreboard (GET /scores). Output:
  cards/card_<nn>_<team>.png   (2480x1748 px = A5 landscape @ 300 dpi)
  cards/mapping.json           marker id -> team name (referee.py reads this)

Rerun after adding teams — existing ids stay stable because the mapping is
merged, so already-printed cards remain valid.
"""
import json
import os
import re
import sys
import urllib.request

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "cards")
BASE = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8081"

W, H = 2480, 1748  # A5 landscape @ 300 dpi
MARKER = 1100      # marker side in px  (~9.3 cm printed)


def main():
    os.makedirs(OUT, exist_ok=True)
    teams = [t["name"] for t in json.loads(
        urllib.request.urlopen(BASE + "/scores", timeout=30).read())["teams"]]
    mpath = os.path.join(OUT, "mapping.json")
    mapping = {}
    if os.path.exists(mpath):
        mapping = json.load(open(mpath))
    name2id = {v: int(k) for k, v in mapping.items()}
    next_id = max([int(k) for k in mapping] + [0]) + 1

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    for team in teams:
        if team in name2id:
            mid = name2id[team]
        else:
            mid = next_id
            next_id += 1
            mapping[str(mid)] = team
        card = np.full((H, W), 255, np.uint8)
        marker = cv2.aruco.drawMarker(dictionary, mid, MARKER)
        my = (H - MARKER) // 2
        card[my:my + MARKER, W - MARKER - 120:W - 120] = marker
        # huge team number on the left half
        num = str(mid)
        scale, thick = (28, 90) if len(num) == 1 else (20, 70)
        (tw, th), _ = cv2.getTextSize(num, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
        cv2.putText(card, num, ((W - MARKER - 240 - tw) // 2, (H + th) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, 0, thick)
        cv2.putText(card, team[:28], (80, H - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.2, 0, 6)
        safe = re.sub(r"[^A-Za-z0-9_-]", "_", team)[:24]
        path = os.path.join(OUT, f"card_{mid:02d}_{safe}.png")
        cv2.imwrite(path, card)
        print(f"id {mid:2d} -> {team:<24} {path}")
    json.dump(mapping, open(mpath, "w"), indent=1)
    print(f"\nmapping: {mpath} ({len(mapping)} teams)")
    print("print each card (A5, matte); teams lay their card below their "
          "score rail to start match mode")


if __name__ == "__main__":
    main()
