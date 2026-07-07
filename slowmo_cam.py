#!/usr/bin/env python3
"""
slowmo_cam.py — rolling slow-motion recorder for the OV9281 (USB UVC).

Continuously captures 1280x720 MJPG @ 120 fps into a RAM ring buffer that
always holds the last BUFFER_SECONDS of footage. Compressed JPEG frames are
stored as-is (never decoded), so CPU and memory stay low.

Keys:
  SPACE / s : save the last BUFFER_SECONDS as a slow-motion video
              (120 fps footage remuxed to PLAYBACK_FPS => 4x slow motion)
  r         : replay the most recently saved clip (ffplay window, loops)
  q / Ctrl-C: quit

Saved clips land in ~/recordings/ as slowmo_YYYYmmdd_HHMMSS.avi.
Saving is a lossless remux (no re-encode) and completes in well under a second.

Usage:
  python3 slowmo_cam.py              # interactive
  python3 slowmo_cam.py --selftest 3 # capture 3 s, auto-save, exit (no keys)
"""

import argparse
import collections
import datetime
import os
import select
import shutil
import signal
import subprocess
import sys
import termios
import threading
import time
import tty

# ---------------------------------------------------------------- settings
DEVICE = "/dev/video0"
WIDTH, HEIGHT = 1280, 720
CAPTURE_FPS = 120          # what the camera delivers
PLAYBACK_FPS = 30          # saved-clip frame rate  -> 120/30 = 4x slow motion
BUFFER_SECONDS = 10        # how much history to keep
OUT_DIR = os.path.expanduser("~/recordings")

MAX_FRAMES = CAPTURE_FPS * BUFFER_SECONDS  # 1200 frames

# ---------------------------------------------------------------- capture

def start_camera():
    """Spawn ffmpeg streaming raw MJPG frames (no decode) to stdout."""
    cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "v4l2", "-input_format", "mjpeg",
        "-video_size", f"{WIDTH}x{HEIGHT}",
        "-framerate", str(CAPTURE_FPS),
        "-i", DEVICE,
        "-c", "copy", "-f", "mjpeg", "pipe:1",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, bufsize=0)


class Recorder:
    """Reads the MJPG byte stream, splits it into JPEG frames, fills the ring."""

    def __init__(self):
        self.ring = collections.deque(maxlen=MAX_FRAMES)
        self.frames_seen = 0
        self.window = collections.deque(maxlen=CAPTURE_FPS)  # timestamps for fps
        self.proc = None
        self.alive = True
        self.error = None

    def measured_fps(self):
        if len(self.window) < 2:
            return 0.0
        span = self.window[-1] - self.window[0]
        return (len(self.window) - 1) / span if span > 0 else 0.0

    def run(self):
        try:
            self.proc = start_camera()
            buf = bytearray()
            read = self.proc.stdout.read
            while self.alive:
                chunk = read(65536)
                if not chunk:
                    if self.alive:
                        err = self.proc.stderr.read().decode(errors="replace")
                        self.error = f"camera stream ended: {err.strip()[:300]}"
                    break
                buf += chunk
                # split on JPEG SOI (FFD8) / EOI (FFD9) markers
                while True:
                    soi = buf.find(b"\xff\xd8")
                    if soi < 0:
                        buf.clear()
                        break
                    eoi = buf.find(b"\xff\xd9", soi + 2)
                    if eoi < 0:
                        if soi > 0:
                            del buf[:soi]
                        break
                    self.ring.append(bytes(buf[soi:eoi + 2]))
                    del buf[:eoi + 2]
                    self.frames_seen += 1
                    self.window.append(time.monotonic())
        except Exception as exc:  # surface capture-thread failures to main loop
            self.error = repr(exc)

    def stop(self):
        self.alive = False
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()


# ---------------------------------------------------------------- save / replay

def save_clip(rec):
    """Dump the ring buffer and remux it to a PLAYBACK_FPS AVI (lossless)."""
    frames = list(rec.ring)  # snapshot; capture keeps running
    if not frames:
        return None, "buffer is empty"
    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw = os.path.join(OUT_DIR, f".tmp_{stamp}.mjpg")
    out = os.path.join(OUT_DIR, f"slowmo_{stamp}.avi")
    with open(raw, "wb") as fh:
        for f in frames:
            fh.write(f)
    res = subprocess.run(
        ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
         "-f", "image2pipe", "-framerate", str(PLAYBACK_FPS),
         "-c:v", "mjpeg", "-i", raw,
         "-c", "copy", out],
        capture_output=True, text=True)
    os.unlink(raw)
    if res.returncode != 0:
        return None, f"ffmpeg remux failed: {res.stderr.strip()[:300]}"
    real = len(frames) / CAPTURE_FPS
    play = len(frames) / PLAYBACK_FPS
    return out, f"{len(frames)} frames | {real:.1f}s real -> {play:.1f}s playback ({CAPTURE_FPS // PLAYBACK_FPS}x slow-mo)"


def replay(path):
    """Open the clip in a looping ffplay window (non-blocking)."""
    return subprocess.Popen(
        ["ffplay", "-hide_banner", "-loglevel", "error",
         "-window_title", f"slow-mo replay: {os.path.basename(path)}",
         "-loop", "0", path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ---------------------------------------------------------------- main

def interactive(rec):
    last_saved = None
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print(f"Recording {WIDTH}x{HEIGHT}@{CAPTURE_FPS} from {DEVICE} — "
          f"keeping last {BUFFER_SECONDS}s in RAM")
    print("[SPACE/s] save last 10s   [r] replay last clip   [q] quit")
    try:
        while True:
            if rec.error:
                print(f"\ncapture error: {rec.error}")
                break
            fill = min(100, 100 * len(rec.ring) // MAX_FRAMES)
            sys.stdout.write(f"\r  buffer {fill:3d}% | {rec.measured_fps():6.1f} fps | "
                             f"last clip: {os.path.basename(last_saved) if last_saved else '-':<28}")
            sys.stdout.flush()
            r, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not r:
                continue
            key = sys.stdin.read(1)
            if key in ("q", "\x03"):
                break
            elif key in (" ", "s"):
                path, info = save_clip(rec)
                if path:
                    last_saved = path
                    print(f"\n  saved {path}\n  {info}")
                else:
                    print(f"\n  save failed: {info}")
            elif key == "r":
                if last_saved:
                    print(f"\n  replaying {os.path.basename(last_saved)} (close window to stop)")
                    replay(last_saved)
                else:
                    print("\n  nothing saved yet — press SPACE first")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


def selftest(rec, seconds):
    """Non-interactive check: fill buffer for N seconds, save, report."""
    print(f"selftest: capturing {seconds}s ...")
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if rec.error:
            print(f"capture error: {rec.error}")
            return 1
        time.sleep(0.25)
    print(f"measured capture rate: {rec.measured_fps():.1f} fps, "
          f"{len(rec.ring)} frames buffered")
    path, info = save_clip(rec)
    if not path:
        print(f"save failed: {info}")
        return 1
    print(f"saved {path}\n{info}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--selftest", type=int, metavar="SECONDS",
                    help="capture N seconds, auto-save, exit")
    args = ap.parse_args()

    if not shutil.which("ffmpeg"):
        sys.exit("ffmpeg not found — install with: sudo apt install ffmpeg")
    if not os.path.exists(DEVICE):
        sys.exit(f"{DEVICE} not found — is the camera plugged in?")

    rec = Recorder()
    t = threading.Thread(target=rec.run, daemon=True)
    t.start()

    try:
        if args.selftest:
            sys.exit(selftest(rec, args.selftest))
        interactive(rec)
    finally:
        rec.stop()


if __name__ == "__main__":
    main()
