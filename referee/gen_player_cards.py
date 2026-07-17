#!/usr/bin/env python3
"""Player cards for single-player (2v2 pickup) mode — one ArUco card per
PLAYER, distinct from the team cards: player ids start at 30 so the two
card sets can never collide (team cards use 1..14).

    python3 gen_player_cards.py Alice Bob Charlie ...
    python3 gen_player_cards.py --file players.txt   (one name per line)

Cards land in cards_players/ with mapping.json (id -> player name).
Ids are stable across reruns: existing names keep their card.
"""
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "cards_players")
FIRST_ID = 30
W, H = 2480, 1748  # A5 landscape @ 300 dpi
MARKER = 1100


def main():
    names = sys.argv[1:]
    if len(names) >= 2 and names[0] == "--file":
        names = [l.strip() for l in open(names[1]) if l.strip()]
    if not names:
        sys.exit("usage: gen_player_cards.py <name> [name ...]  |  --file players.txt")
    os.makedirs(OUT, exist_ok=True)
    mpath = os.path.join(OUT, "mapping.json")
    mapping = json.load(open(mpath)) if os.path.exists(mpath) else {}
    name2id = {v: int(k) for k, v in mapping.items()}
    next_id = max([int(k) for k in mapping] + [FIRST_ID - 1]) + 1

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    for name in names:
        if name in name2id:
            pid = name2id[name]
        else:
            pid = next_id
            next_id += 1
            mapping[str(pid)] = name
        if pid >= 50:
            sys.exit("DICT_4X4_50 has ids 0..49 — too many players")
        card = np.full((H, W), 255, np.uint8)
        marker = cv2.aruco.drawMarker(dictionary, pid, MARKER)
        my = (H - MARKER) // 2
        card[my:my + MARKER, W - MARKER - 120:W - 120] = marker
        num = str(pid)
        scale, thick = (20, 70) if len(num) < 3 else (16, 60)
        (tw, th), _ = cv2.getTextSize(num, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
        cv2.putText(card, num, ((W - MARKER - 240 - tw) // 2, (H + th) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, 0, thick)
        cv2.putText(card, "PLAYER: " + name[:24], (80, H - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.2, 0, 6)
        fn = os.path.join(OUT, f"player_{pid:02d}_" +
                          "".join(c if c.isalnum() or c in "-_" else "_"
                                  for c in name) + ".png")
        cv2.imwrite(fn, card)
        print(f"id {pid} -> {name:<24} {fn}")

    json.dump(mapping, open(mpath, "w"), indent=1)
    print(f"\nmapping: {mpath} ({len(mapping)} players)")
    print("corners: your DEFENDER card goes at your score rail (as in team "
          "mode), your ATTACKER card at the free corner of your side")


if __name__ == "__main__":
    main()
