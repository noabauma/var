# slowmo-cam — instant slow-motion replay for game nights

A Raspberry Pi 4 + OV9281 USB camera (1280×720 @ 120 fps) record continuously
into RAM. One key — or one button in any browser — replays the last 5 seconds
in 4× slow motion while the clip saves in the background. Only the newest 20
clips are kept (`--max-clips`, renamed clips are kept forever). Includes a
tournament scoreboard.

| File | What it is |
|---|---|
| `slowmo_cam.cpp` | the recorder (C++, standalone; `slowmo_cam.py` = old reference) |
| `score_function/` | PageRank score algorithms + `compute_scores.py` bridge |
| `deploy/slowmo-cam.service` | systemd unit (source of the installed one) |
| `set-web-password.sh` | change the web login password |

## Running it (systemd)

The recorder runs as a service: starts on boot, retries every 5 s until the
camera appears (hot-plug works — unplugging mid-game is fine), restarts on
crashes. **`kill`/htop won't stop it — systemd revives it in 5 s.** Use:

```bash
sudo systemctl stop  slowmo-cam            # off (until next boot)
sudo systemctl start slowmo-cam            # on
sudo systemctl disable --now slowmo-cam    # off + no autostart
sudo systemctl enable  --now slowmo-cam    # undo that
sudo systemctl restart slowmo-cam          # after `make`
journalctl -u slowmo-cam -f                # live logs
# reinstall after editing the unit:
sudo cp deploy/slowmo-cam.service /etc/systemd/system/ && \
  sudo systemctl daemon-reload && sudo systemctl enable --now slowmo-cam
```

Build with `make` (needs `g++ make ffmpeg`), then restart the service.

## Web page

```
https://<pi-ip>/        user: biliardino   password: ~/slowmo-cam-password.txt
```

nginx terminates TLS on 443 (self-signed cert, accept the one-time warning;
TLSv1.2 + ChaCha20 pinned — the Pi has no AES hardware, ChaCha is ~11×
faster) and proxies to the recorder on `127.0.0.1:8081`. Old
`http://…:8080` links redirect. Change the password with
`./set-web-password.sh`. Over SSH, tunnel instead:
`ssh -L 8081:localhost:8081 noabauma@<pi-ip>` → `http://localhost:8081`.
**Prefer it open (no TLS/login)?** Set `--bind 0.0.0.0` in the service and
browse `http://<pi-ip>:8081` — only do this if 443 isn't internet-exposed.

On the page: live view, status bar (fps / buffer / drops / 👁 viewer count),
**Save + replay**, **Replay last**, **Live**, fullscreen (⛶/`f`), and a
**recordings panel** (click to replay a clip, ✎ rename = keep forever,
🗑 delete). Keys: `space` save, `r` replay, `f` fullscreen, `l`/`esc` live.

**📷 Camera ON/OFF (privacy switch):** anyone can pause recording from the
page. OFF truly stops capture — the V4L2 device is closed, the ring buffer
is wiped (nothing from before the press can be saved), `/save` is refused,
and the video area shows "Camera is OFF". ON brings it back in ~2 s.
Saved clips on disk are not affected. `POST /camera?on=0|1`.

**Slow link (VPN/weak WiFi)?** The full stream is ~24 Mbit/s. Pick a lower
rate in the **fps selector** next to Live (remembered per browser); slow
viewers also self-adapt server-side to fresh-not-stale frames.

The camera is opened exactly once, by the recorder — the browser gets the
same compressed MJPEG frames from the ring buffer (no transcoding), so live
view, saves and replays all run simultaneously without conflict.

Endpoints: `GET /stream` (`?fps=N`), `GET /replay` (`?file=x.avi` for disk
clips), `POST /save`, `GET /status`, `GET /recordings`,
`POST /recordings/rename?file=X&name=Y`, `POST /recordings/delete?file=X`.

## Tournament scoreboard

Rankings switchable between **bias PageRank**
(`page_rank_biliardino_algorithm_bias.py`, baseline ∝ games played) and
**classic PageRank** (`page_rank_biliardino_algorithm.py`). The Python files
are the single source of truth — edit them and the board follows
(`recursive_deletion` is deliberately not applied mid-tournament). The
damping slider (default 0.85) is the only thing not persisted.

One big group; a match is **best of three** to 10; **each pair plays at most
once**. Enter *team A*, *team B*, game scores from A's view
(`10-9, 10-4` or `10-1, 5-10, 10-9`) — the winner is derived, incomplete
best-of-threes and rematches are rejected. New names create teams
(**Add team** also works); click a team row for its match history, per-match
✎ edit / 🗑 delete, **rename**, and **Delete team**. **Undo** removes the
last match. Below the table: head-to-head **matrix + directed graph**.

**Persistence: every change is written immediately (atomic tmp+rename) to
`~/recordings/tournament.tsv` — stopping, killing, crashing or a power cut
loses nothing.** Start a fresh tournament with:

```bash
: > ~/recordings/tournament.tsv && sudo systemctl restart slowmo-cam
```

(truncate, don't delete — a missing file re-seeds last year's demo data).

Endpoints: `GET /scores`, `POST /scores/add?a=X&b=Y&games=10-9,10-4`,
`/scores/rename?team=X&name=Y`, `/scores/undo`, `/scores/team/add?name=X`,
`/scores/team/delete?name=X`, `/scores/match/edit?a=X&b=Y&games=…`,
`/scores/match/delete?a=X&b=Y`, `/scores/d?value=0.85` (all POST).
Options: `--scores-file`, `--scores-script`, `--no-scores`. Needs
`python3` + `numpy`.

## Terminal keys (when run interactively)

`SPACE`/`s` save + instant replay · `r` replay again · any key stops a
replay · `q` quit.

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
--player CMD         custom replay command (raw MJPEG on stdin)
--no-autoreplay      s only saves; use r to replay
--selftest N         capture N s, save, verify, exit (no keyboard needed)
--selftest-replay    selftest also plays one replay pass
--mjpeg-file PATH    test input: raw .mjpeg file instead of a camera
--port N             web port (default 8080, 0 = off)
--bind ADDR          web listener (default 0.0.0.0; 127.0.0.1 = proxy only)
--http-fps N         default live-stream rate (default 30)
--scores-script PATH python bridge for the scoreboard
--scores-file PATH   tournament state (default <out-dir>/tournament.tsv)
--no-scores          disable the web scoreboard
```

## Self-test & troubleshooting

```bash
./slowmo_cam --selftest 3        # capture fps + saving, non-zero exit on failure
# no camera? test against a generated stream:
ffmpeg -f lavfi -i testsrc2=duration=4:rate=60:size=1280x720 -c:v mjpeg -f mjpeg test.mjpeg
./slowmo_cam --mjpeg-file test.mjpeg --fps 60 --selftest 3
```

- **fps far below 120** — mode not granted: check
  `v4l2-ctl -d /dev/video0 --list-formats-ext`, request a real mode.
- **`drops` climbing** — USB losing frames: shorter cable / lower fps.
- **no camera** — page shows "no camera", video resumes on plug-in.
- **TV score display goes black** — X blanking/DPMS is disabled on this Pi
  (`xserver-command=X -s 0 -dpms` in `/etc/lightdm/lightdm.conf`); if the
  TV still sleeps, it's the TV's own eco/sleep timer or HDMI-CEC setting.
- **profiling** — `make tracr` builds an instrumented binary (TraCR);
  `~/src/tracr/build/tracr_process tracr/tracr perfetto` → ui.perfetto.dev.

## Design notes

- Saves are a **lossless one-pass AVI write** (own muxer, no ffmpeg, no
  temp file); replays play **from RAM** while the file writes — that, plus
  reading the camera directly via V4L2, is why the C++ version beat the old
  Python one (the SD card was the bottleneck, not Python).
- `.avi` because it writes in one pass and plays everywhere. For sharing:
  `ffmpeg -i clip.avi -c copy clip.mp4` (instant) or
  `-c:v libx264 -crf 20` (small, slower).
