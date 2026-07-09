# slowmo-cam — instant slow-motion replay for game nights

A Raspberry Pi 4 + an OV9281 USB camera (1280×720 @ 120 fps) continuously
record into RAM. Press **one key** — or **one button on any phone/laptop
browser** — and the last 5 seconds replay immediately in 4× slow motion,
while the clip is saved to disk in the background. Only the newest 20
clips are kept (`--max-clips N`, 0 = keep everything).

## What's in this repo

| File | What it is |
|---|---|
| `slowmo_cam.cpp` | **The recorder** — standalone C++ version (use this one) |
| `slowmo_cam.py` | The original Python version, kept for reference |
| `Makefile` | Builds the C++ version |

## Quick start (on the Raspberry Pi)

```bash
sudo apt install -y g++ make ffmpeg   # ffmpeg only for its `ffplay` replay window
make
./slowmo_cam
```

Clips are saved to `~/recordings/slowmo_YYYYmmdd_HHMMSS.avi`.

## Keys

| Key | Action |
|---|---|
| `SPACE` / `s` | Save the last 5 s **and replay it immediately** (4× slow motion, loops) |
| `r` | Replay the last clip again |
| *any key* | Stop a running replay (or press `q` inside the replay window) |
| `q` / `Ctrl-C` | Quit |

## Live view in the browser (VAR + live cam at the same time)

The recorder itself serves a web page — start `./slowmo_cam` and open:

```
http://<pi-ip>:8080          # any browser on the same network
```

Or, when you're connected over SSH, tunnel it (run on your laptop):

```bash
ssh -L 8080:localhost:8080 noabauma@<pi-ip>   # then open http://localhost:8080
```

(In an already-open SSH session: press `~C` then type `-L 8080:localhost:8080`.)

The page shows the **live camera**, a status bar (fps / buffer / drops), and
three buttons — **Save + replay** (same as pressing `s`), **Replay last**,
**Live** — so any phone on the WiFi can be the VAR trigger. Keyboard works
there too: `space` save, `r` replay, `l`/`esc` back to live.

**Why live view + VAR work simultaneously:** a UVC camera can only be opened
by *one* process — two programs would conflict (`Device or resource busy`).
So the live stream is served *by the recorder itself*: the same compressed
MJPEG frames that fill the ring buffer are forwarded to the browser as
`multipart/x-mixed-replace` (natively rendered by every browser in an
`<img>`). Nothing is decoded or re-encoded, the camera is opened exactly
once, and streaming costs near-zero CPU. Recording never pauses — saves and
terminal replays keep working while any number of browsers watch.

Endpoints (for scripting): `GET /stream` (live MJPEG, `?fps=N` to lower the
rate), `GET /replay` (last clip from RAM, loops), `POST /save` (trigger a
save, returns JSON), `GET /status` (JSON). No authentication — trusted
LAN/tunnel only.

## Tournament scoreboard

The web page shows the **current tournament ranking**, switchable between
the two score functions (both computed on every change, so the toggle is
instant):

- **Bias PageRank** — `page_rank_billiardino_algorithm_bias.py`
  (baseline ∝ games played, compensates uneven participation)
- **Classic PageRank** — `page_rank_billiardino_algorithm.py`

Both Python files stay the single source of truth: the recorder calls them
through `score_function/compute_scores.py`, so tweaking an algorithm
immediately changes the live board. (`recursive_deletion` is deliberately
not applied: it disqualifies the least-connected teams, which is wrong
mid-tournament.)

**Entering matches** — one big group, every match is a **best of three**
to 10, and **each pair of teams plays at most once**:

- Fill in *team A*, *team B* and the game scores **from A's perspective**,
  e.g. `10-9, 10-4` (2-0) or `10-1, 5-10, 10-9` (2-1). The winner is
  derived from the games; incomplete or impossible best-of-threes are
  rejected, as is a rematch of a pair that already played (undo it first).
- New team names create new teams automatically; existing ones autocomplete.
  Teams can also be added explicitly (**Add team**) before they play.
- **Click a team row** to see all its matches (`vs Team X: 10-1, 5-10, 10-9
  → won`) — each match there can be **edited** (✎, fix the game scores;
  the winner is re-derived) or **deleted** (🗑). The panel also offers
  **rename** (results follow the team) and **Delete team** (removes the
  team and all its matches).
- **Undo** removes the last entered match.

State persists in a plain TSV (default `~/recordings/tournament.tsv`).
First run seeds **last year's group A dummy data** so the board shows
something. Start the real tournament fresh with
`: > ~/recordings/tournament.tsv` and a restart (deleting the file instead
would re-seed the demo).

Endpoints: `GET /scores` (both scores + all matches, JSON),
`POST /scores/add?a=X&b=Y&games=10-9,10-4`,
`POST /scores/rename?team=X&name=Y`, `POST /scores/undo`,
`POST /scores/team/add?name=X`, `POST /scores/team/delete?name=X`,
`POST /scores/match/edit?a=X&b=Y&games=…`, `POST /scores/match/delete?a=X&b=Y`.
Options: `--scores-file PATH`, `--scores-script PATH`, `--no-scores`.
Needs `python3` + `numpy` (already on the Pi).

**Headless / kiosk use:** without a terminal the program keeps recording and
serves the web controls (`nohup ./slowmo_cam >/tmp/slowmo.log 2>&1 &`).

## Why this is much faster than the Python version

The slowness you felt on the Pi was almost entirely **SD-card I/O sitting in
the critical path**, not Python itself:

1. **The old save wrote everything to the SD card twice.** It dumped the
   buffer to a temp file, then ran ffmpeg to read it back and write the AVI.
   The new save has its own AVI writer: **one sequential write, no temp
   file, no ffmpeg process**. Still a lossless remux — frames are never
   re-encoded.
2. **The replay no longer waits for the disk at all.** Pressing `s` starts
   playing straight **from RAM** while the file is written in the
   background. The old flow was: save to SD → press `r` → ffplay starts →
   reads the clip back from SD.
3. **Auto-replay.** `s` does save **+** replay in one press (the old
   `s`-then-`r` double step is gone) — the table sees the action right away.
4. **5 s buffer instead of 10 s** — half the data everywhere (RAM, save
   time, file size). Use `--seconds 10` if you ever want the old window.
5. **The camera is read directly via V4L2** instead of through an ffmpeg
   child process whose byte stream had to be re-scanned for JPEG markers in
   Python. Lower CPU, exact frame boundaries, and a `drops` counter in the
   status line so you can *see* if the camera ever skips.

## About your `.avi` → `.mp4` idea

The container format has practically **no effect on speed** — inside both
files sit the exact same MJPEG frames, and both are written without
re-encoding. The time was lost in the double disk writes and the
read-back-for-replay described above, so that is what the rewrite removed.

`.avi` is kept because it's the simplest container to write in one pass and
plays everywhere (`ffplay`, VLC, mpv). If you want an `.mp4` to share (e.g.
phone/WhatsApp), convert a clip afterwards — instant, lossless:

```bash
ffmpeg -i slowmo_x.avi -c copy slowmo_x.mp4        # same MJPEG, mp4 container
```

or re-encode to H.264 for much smaller, universally playable files (slower,
so do it after game night):

```bash
ffmpeg -i slowmo_x.avi -c:v libx264 -crf 20 slowmo_x_small.mp4
```

## Options

```
--device PATH        camera device            (default /dev/video0)
--width N            capture width            (default 1280)
--height N           capture height           (default 720)
--fps N              capture frame rate       (default 120)
--playback-fps N     saved/replay frame rate  (default 30 → 4× slow motion)
--seconds S          seconds kept in RAM      (default 5)
--max-clips N        keep only the newest N clips (default 20, 0 = all)
--out-dir DIR        where clips are saved    (default ~/recordings)
--player CMD         custom replay command (gets raw MJPEG on stdin),
                     e.g. --player "mpv --really-quiet -"
--no-autoreplay      s only saves; use r to replay
--selftest N         capture N s, save, verify, exit (no keyboard needed)
--selftest-replay    selftest also plays one replay pass
--mjpeg-file PATH    test input: raw .mjpeg stream file instead of a camera
--port N             web live view + control port (default 8080, 0 = off)
--http-fps N         live-stream rate served to browsers (default 30)
--scores-script PATH python bridge for the scoreboard
--scores-file PATH   tournament state (default <out-dir>/tournament.tsv)
--no-scores          disable the web scoreboard
```

## Self-test

On the Pi, with the camera plugged in:

```bash
./slowmo_cam --selftest 3                    # checks capture fps + saving
./slowmo_cam --selftest 3 --selftest-replay  # also opens one replay pass
```

It prints the measured capture rate, the number of buffered frames and
drops, saves a clip, and exits non-zero if anything failed.

No camera at hand? Generate any raw MJPEG file and run against it:

```bash
ffmpeg -f lavfi -i testsrc2=duration=4:rate=60:size=1280x720 -c:v mjpeg -f mjpeg test.mjpeg
./slowmo_cam --mjpeg-file test.mjpeg --fps 60 --selftest 3
```

## Troubleshooting

- **Buffer fills but fps is far below 120** — the camera didn't get the
  120 fps mode. Check what it offers:
  `v4l2-ctl -d /dev/video0 --list-formats-ext` (needs `v4l-utils`).
  Ask for a mode it really has, e.g. `--width 640 --height 480 --fps 100`.
- **`drops` climbing in the status line** — frames are being lost between
  camera and USB; try a shorter cable, a powered hub, or a lower fps.
- **Replay window doesn't open** — `ffplay` missing (`sudo apt install
  ffmpeg`) or no display session (`DISPLAY` unset when starting from SSH —
  run `export DISPLAY=:0` first). Saving still works without it; the
  program also falls back to opening the saved file if the RAM replay
  window fails to start.
- **Nothing saved / buffer 0%** — run `./slowmo_cam --selftest 3` and read
  the error; it reports exactly which camera step failed.

## How it works

- A capture thread streams compressed MJPEG frames from the camera straight
  into a RAM ring buffer that always holds the last 5 s. Frames are **never
  decoded** while recording, so CPU load stays minimal.
- `s` snapshots the ring (shared pointers — microseconds, capture never
  pauses), hands it to a background thread that writes the AVI in one
  sequential pass, and at the same time pipes the frames from RAM into an
  `ffplay` window at 30 fps → instant 4× slow-motion replay.
- 120 captured fps played back at 30 fps is the whole slow-motion trick —
  every real second lasts four seconds on screen, at full quality.
