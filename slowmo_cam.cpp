// slowmo_cam.cpp — standalone rolling slow-motion replay recorder (C++).
//
// Continuously captures MJPEG frames from a V4L2 camera (OV9281 UVC,
// 1280x720 @ 120 fps) into a RAM ring buffer holding the last
// BUFFER_SECONDS of footage. Frames stay compressed (never decoded),
// so CPU and memory stay low.
//
// Keys:
//   SPACE / s : save the last 5 s AND replay it immediately in slow motion
//               (file is written in the background; the replay plays
//               straight from RAM, so it starts instantly)
//   r         : replay the last clip again (loops)
//   any key   : stop a running replay (or press q inside the player window)
//   q / Ctrl-C: quit
//
// Saved clips land in ~/recordings/ as slowmo_YYYYmmdd_HHMMSS.avi
// (MJPEG in AVI, written by this program in one sequential pass —
// no temp file, no re-encode, no ffmpeg process).
//
// Build:  g++ -O2 -std=c++17 -Wall -o slowmo_cam slowmo_cam.cpp -lpthread
// Deps:   ffplay (from the ffmpeg package) is used only as the replay window.
//
// Usage:
//   ./slowmo_cam                    # interactive
//   ./slowmo_cam --selftest 3      # capture 3 s, auto-save, verify, exit
//   ./slowmo_cam --help            # all options

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <linux/videodev2.h>

// ------------------------------------------------------------------ utils

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

static int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

static double now_mono() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static std::string errno_str(const std::string &what) {
    return what + ": " + std::strerror(errno);
}

static bool has_cmd(const std::string &name) {
    const char *path = getenv("PATH");
    if (!path) return false;
    std::string p(path), dir;
    size_t pos = 0;
    while (pos <= p.size()) {
        size_t colon = p.find(':', pos);
        dir = p.substr(pos, colon == std::string::npos ? colon : colon - pos);
        if (!dir.empty() && access((dir + "/" + name).c_str(), X_OK) == 0)
            return true;
        if (colon == std::string::npos) break;
        pos = colon + 1;
    }
    return false;
}

static std::string expand_home(std::string path) {
    if (!path.empty() && path[0] == '~') {
        const char *home = getenv("HOME");
        if (home) path = std::string(home) + path.substr(1);
    }
    return path;
}

static bool mkdirs(const std::string &path) {
    std::string cur;
    size_t pos = 0;
    while (pos <= path.size()) {
        size_t slash = path.find('/', pos);
        cur = slash == std::string::npos ? path : path.substr(0, slash);
        if (!cur.empty() && mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST)
            return false;
        if (slash == std::string::npos) break;
        pos = slash + 1;
    }
    return true;
}

static std::string basename_of(const std::string &p) {
    size_t s = p.rfind('/');
    return s == std::string::npos ? p : p.substr(s + 1);
}

// ------------------------------------------------------------------ config

struct Cfg {
    std::string device = "/dev/video0";
    int width = 1280, height = 720;
    int capture_fps = 120;        // what the camera delivers
    int playback_fps = 30;        // saved/replayed rate -> 120/30 = 4x slow-mo
    double buffer_seconds = 5.0;  // how much history to keep in RAM
    std::string out_dir = "~/recordings";
    std::string player;           // custom replay command (stdin = mjpeg); empty = ffplay
    bool autoreplay = true;       // 's' also starts the replay
    int selftest = 0;             // capture N seconds, save, verify, exit
    bool selftest_replay = false; // selftest also plays one replay pass
    std::string mjpeg_file;       // test input: raw .mjpeg stream instead of camera

    size_t max_frames() const {
        double f = capture_fps * buffer_seconds;
        return f < 1 ? 1 : (size_t)(f + 0.5);
    }
};

static void usage(const char *argv0) {
    std::printf(
        "usage: %s [options]\n"
        "\n"
        "  --device PATH        camera device (default /dev/video0)\n"
        "  --width N            capture width (default 1280)\n"
        "  --height N           capture height (default 720)\n"
        "  --fps N              capture frame rate (default 120)\n"
        "  --playback-fps N     saved/replay frame rate (default 30 => 4x slow-mo)\n"
        "  --seconds S          seconds of history kept in RAM (default 5)\n"
        "  --out-dir DIR        where clips are saved (default ~/recordings)\n"
        "  --player CMD         replay command run via sh -c, mjpeg on its stdin\n"
        "                       (default: built-in ffplay window)\n"
        "  --no-autoreplay      do not start the replay automatically after save\n"
        "  --selftest N         capture N seconds, save, verify, exit (no keys)\n"
        "  --selftest-replay    selftest also plays the clip once through the player\n"
        "  --mjpeg-file PATH    read frames from a raw .mjpeg file instead of a\n"
        "                       camera (testing; frames must match --width/--height)\n"
        "  --help               this text\n",
        argv0);
}

static bool parse_args(int argc, char **argv, Cfg &cfg) {
    auto need = [&](int &i) -> const char * {
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing value for %s\n", argv[i]);
            return nullptr;
        }
        return argv[++i];
    };
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        const char *v;
        if (a == "--device") { if (!(v = need(i))) return false; cfg.device = v; }
        else if (a == "--width") { if (!(v = need(i))) return false; cfg.width = atoi(v); }
        else if (a == "--height") { if (!(v = need(i))) return false; cfg.height = atoi(v); }
        else if (a == "--fps") { if (!(v = need(i))) return false; cfg.capture_fps = atoi(v); }
        else if (a == "--playback-fps") { if (!(v = need(i))) return false; cfg.playback_fps = atoi(v); }
        else if (a == "--seconds") { if (!(v = need(i))) return false; cfg.buffer_seconds = atof(v); }
        else if (a == "--out-dir") { if (!(v = need(i))) return false; cfg.out_dir = v; }
        else if (a == "--player") { if (!(v = need(i))) return false; cfg.player = v; }
        else if (a == "--no-autoreplay") cfg.autoreplay = false;
        else if (a == "--selftest") { if (!(v = need(i))) return false; cfg.selftest = atoi(v); }
        else if (a == "--selftest-replay") cfg.selftest_replay = true;
        else if (a == "--mjpeg-file") { if (!(v = need(i))) return false; cfg.mjpeg_file = v; }
        else if (a == "--help" || a == "-h") { usage(argv[0]); exit(0); }
        else { std::fprintf(stderr, "unknown option: %s\n", a.c_str()); return false; }
    }
    cfg.out_dir = expand_home(cfg.out_dir);
    if (cfg.width <= 0 || cfg.height <= 0 || cfg.capture_fps <= 0 ||
        cfg.playback_fps <= 0 || cfg.buffer_seconds <= 0) {
        std::fprintf(stderr, "width/height/fps/playback-fps/seconds must be > 0\n");
        return false;
    }
    return true;
}

// ------------------------------------------------------------- shared state

using FrameData = std::vector<uint8_t>;
using Frame = std::shared_ptr<const FrameData>;
using Snapshot = std::vector<Frame>;

// Ring buffer of compressed frames. The capture thread pushes; snapshots
// copy only the shared_ptrs (a few microseconds), so capture never stalls.
class Ring {
public:
    Ring(size_t maxlen, size_t fps_window)
        : maxlen_(maxlen), fps_window_(fps_window < 2 ? 2 : fps_window) {}

    void push(Frame f) {
        std::lock_guard<std::mutex> lk(m_);
        q_.push_back(std::move(f));
        if (q_.size() > maxlen_) q_.pop_front();
        stamps_.push_back(now_mono());
        if (stamps_.size() > fps_window_) stamps_.pop_front();
    }
    size_t size() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.size();
    }
    double measured_fps() const {
        std::lock_guard<std::mutex> lk(m_);
        if (stamps_.size() < 2) return 0.0;
        double span = stamps_.back() - stamps_.front();
        return span > 0 ? (stamps_.size() - 1) / span : 0.0;
    }
    std::shared_ptr<Snapshot> snapshot() const {
        std::lock_guard<std::mutex> lk(m_);
        return std::make_shared<Snapshot>(q_.begin(), q_.end());
    }

private:
    mutable std::mutex m_;
    std::deque<Frame> q_;
    std::deque<double> stamps_;
    size_t maxlen_, fps_window_;
};

// Lines produced by worker threads, printed by whichever loop owns the tty.
class MessageQueue {
public:
    void push(std::string s) {
        std::lock_guard<std::mutex> lk(m_);
        v_.push_back(std::move(s));
    }
    std::vector<std::string> drain() {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<std::string> out;
        out.swap(v_);
        return out;
    }

private:
    std::mutex m_;
    std::vector<std::string> v_;
};

struct Shared {
    Ring ring;
    MessageQueue msgs;
    std::atomic<bool> alive{true};
    std::atomic<bool> save_busy{false};
    std::atomic<bool> interactive{false};
    std::atomic<uint64_t> drops{0};
    std::mutex err_m;
    std::string error;
    std::mutex path_m;
    std::string last_path; // last successfully saved file

    Shared(size_t maxlen, size_t fps_window) : ring(maxlen, fps_window) {}

    void set_error(const std::string &e) {
        std::lock_guard<std::mutex> lk(err_m);
        if (error.empty()) error = e;
    }
    std::string get_error() {
        std::lock_guard<std::mutex> lk(err_m);
        return error;
    }
    void set_last_path(const std::string &p) {
        std::lock_guard<std::mutex> lk(path_m);
        last_path = p;
    }
    std::string get_last_path() {
        std::lock_guard<std::mutex> lk(path_m);
        return last_path;
    }
};

// --------------------------------------------------------------- AVI muxer
// Minimal RIFF/AVI writer for one MJPEG video stream. Every value below is
// from the AVI 1.0 spec (little-endian throughout). Because the snapshot is
// fully in RAM, all sizes are known up front and the file is written in a
// single sequential pass — the cheapest possible operation for an SD card.

namespace avi {

struct Bld {
    std::vector<uint8_t> b;
    void u16(uint32_t v) {
        b.push_back(v & 0xff);
        b.push_back((v >> 8) & 0xff);
    }
    void u32(uint32_t v) {
        for (int i = 0; i < 4; i++) b.push_back((v >> (8 * i)) & 0xff);
    }
    void fcc(const char *s) { b.insert(b.end(), s, s + 4); }
};

static bool write_mjpg(const std::string &path, const Snapshot &frames,
                       int w, int h, int fps, std::string &err) {
    if (frames.empty()) { err = "no frames"; return false; }

    const size_t n = frames.size();
    uint64_t payload = 0, maxframe = 0, total_bytes = 0;
    for (const auto &f : frames) {
        uint64_t sz = f->size();
        payload += 8 + sz + (sz & 1); // '00dc' + size + data + even padding
        total_bytes += sz;
        if (sz > maxframe) maxframe = sz;
    }
    const uint64_t movi_size = 4 + payload;            // 'movi' + chunks
    const uint64_t idx_size = 16 * n;                  // idx1 entries
    const uint64_t hdrl_size = 4 + (8 + 56) + (8 + 4 + (8 + 56) + (8 + 40));
    const uint64_t riff_size = 4 + (8 + hdrl_size) + (8 + movi_size) + (8 + idx_size);
    if (riff_size + 8 > 0xF0000000ULL) { err = "clip too large for AVI"; return false; }

    const uint32_t bytes_per_sec = (uint32_t)(total_bytes / n * fps);

    Bld hd;
    hd.fcc("RIFF"); hd.u32((uint32_t)riff_size); hd.fcc("AVI ");
    hd.fcc("LIST"); hd.u32((uint32_t)hdrl_size); hd.fcc("hdrl");
    // ---- avih (MainAVIHeader, 56 bytes)
    hd.fcc("avih"); hd.u32(56);
    hd.u32(1000000u / fps);        // dwMicroSecPerFrame
    hd.u32(bytes_per_sec);         // dwMaxBytesPerSec
    hd.u32(0);                     // dwPaddingGranularity
    hd.u32(0x10 | 0x800);          // dwFlags: HASINDEX | TRUSTCKTYPE
    hd.u32((uint32_t)n);           // dwTotalFrames
    hd.u32(0);                     // dwInitialFrames
    hd.u32(1);                     // dwStreams
    hd.u32((uint32_t)maxframe);    // dwSuggestedBufferSize
    hd.u32((uint32_t)w);
    hd.u32((uint32_t)h);
    for (int i = 0; i < 4; i++) hd.u32(0); // dwReserved
    // ---- strl list
    hd.fcc("LIST"); hd.u32(4 + (8 + 56) + (8 + 40)); hd.fcc("strl");
    // strh (AVISTREAMHEADER, 56 bytes)
    hd.fcc("strh"); hd.u32(56);
    hd.fcc("vids"); hd.fcc("MJPG");
    hd.u32(0);                     // dwFlags
    hd.u16(0); hd.u16(0);          // wPriority, wLanguage
    hd.u32(0);                     // dwInitialFrames
    hd.u32(1);                     // dwScale
    hd.u32((uint32_t)fps);         // dwRate  -> fps = dwRate/dwScale
    hd.u32(0);                     // dwStart
    hd.u32((uint32_t)n);           // dwLength (frames)
    hd.u32((uint32_t)maxframe);    // dwSuggestedBufferSize
    hd.u32(0xFFFFFFFFu);           // dwQuality (default)
    hd.u32(0);                     // dwSampleSize (0 = one sample per chunk)
    hd.u16(0); hd.u16(0); hd.u16((uint16_t)w); hd.u16((uint16_t)h); // rcFrame
    // strf (BITMAPINFOHEADER, 40 bytes)
    hd.fcc("strf"); hd.u32(40);
    hd.u32(40);                    // biSize
    hd.u32((uint32_t)w);
    hd.u32((uint32_t)h);
    hd.u16(1);                     // biPlanes
    hd.u16(24);                    // biBitCount
    hd.fcc("MJPG");                // biCompression
    hd.u32((uint32_t)(w * h * 3)); // biSizeImage (nominal)
    hd.u32(0); hd.u32(0); hd.u32(0); hd.u32(0); // pels/clr fields
    // ---- movi list header
    hd.fcc("LIST"); hd.u32((uint32_t)movi_size); hd.fcc("movi");

    FILE *fh = std::fopen(path.c_str(), "wb");
    if (!fh) { err = errno_str("open " + path); return false; }
    std::vector<char> iobuf(1 << 18);
    setvbuf(fh, iobuf.data(), _IOFBF, iobuf.size());

    bool ok = std::fwrite(hd.b.data(), 1, hd.b.size(), fh) == hd.b.size();

    // frame chunks + index (offsets are relative to the 'movi' fourcc)
    Bld idx;
    idx.fcc("idx1"); idx.u32((uint32_t)idx_size);
    uint32_t off = 4;
    static const uint8_t pad = 0;
    for (size_t i = 0; ok && i < n; i++) {
        const FrameData &f = *frames[i];
        Bld ch;
        ch.fcc("00dc"); ch.u32((uint32_t)f.size());
        ok = ok && std::fwrite(ch.b.data(), 1, 8, fh) == 8;
        ok = ok && std::fwrite(f.data(), 1, f.size(), fh) == f.size();
        if (f.size() & 1) ok = ok && std::fwrite(&pad, 1, 1, fh) == 1;
        idx.fcc("00dc");
        idx.u32(0x10); // AVIIF_KEYFRAME (every MJPEG frame is one)
        idx.u32(off);
        idx.u32((uint32_t)f.size());
        off += 8 + (uint32_t)f.size() + (f.size() & 1);
    }
    ok = ok && std::fwrite(idx.b.data(), 1, idx.b.size(), fh) == idx.b.size();
    ok = std::fflush(fh) == 0 && ok && !ferror(fh);
    std::fclose(fh);
    if (!ok) {
        err = errno_str("write " + path);
        unlink(path.c_str());
        return false;
    }
    return true;
}

} // namespace avi

// ------------------------------------------------------------ child procs

struct Child {
    pid_t pid = -1;
    int fd = -1; // write end of the child's stdin pipe (-1 if none)
};

// Spawn argv with an optional stdin pipe; optionally silence stdout/stderr.
static Child spawn(const std::vector<std::string> &argv, bool stdin_pipe,
                   bool silence_output) {
    Child c;
    int p[2] = {-1, -1};
    if (stdin_pipe && pipe2(p, O_CLOEXEC) != 0) return c;

    std::vector<char *> args;
    for (const auto &a : argv) args.push_back(const_cast<char *>(a.c_str()));
    args.push_back(nullptr);

    pid_t pid = fork();
    if (pid == 0) {
        if (stdin_pipe) dup2(p[0], 0); // dup2 clears O_CLOEXEC on fd 0
        else {
            int dn = open("/dev/null", O_RDONLY);
            if (dn >= 0) dup2(dn, 0);
        }
        if (silence_output) {
            int dn = open("/dev/null", O_WRONLY);
            if (dn >= 0) { dup2(dn, 1); dup2(dn, 2); }
        }
        execvp(args[0], args.data());
        _exit(127);
    }
    if (stdin_pipe) close(p[0]);
    if (pid < 0) {
        if (stdin_pipe) close(p[1]);
        return c;
    }
    c.pid = pid;
    c.fd = stdin_pipe ? p[1] : -1;
    return c;
}

// Wait for a child: give it grace_ms to exit, then SIGTERM, then SIGKILL.
static void reap(pid_t pid, int grace_ms) {
    if (pid <= 0) return;
    for (int waited = 0;; waited += 50) {
        int st;
        pid_t r = waitpid(pid, &st, WNOHANG);
        if (r == pid || (r == -1 && errno == ECHILD)) return;
        if (waited >= grace_ms) break;
        usleep(50 * 1000);
    }
    kill(pid, SIGTERM);
    for (int waited = 0; waited < 1000; waited += 50) {
        int st;
        if (waitpid(pid, &st, WNOHANG) != 0) return;
        usleep(50 * 1000);
    }
    kill(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
}

// ------------------------------------------------------------------ replay

enum class ReplayEnd { Done, Key, WindowClosed, PlayerFail, Stop };

static void drain_stdin() {
    char b[64];
    ssize_t r = read(0, b, sizeof b);
    (void)r;
}

// Play a snapshot from RAM: pipe raw MJPEG into a player window at
// playback_fps. Loops until a terminal key, the window is closed, or
// (max_cycles > 0) that many passes completed. Never touches the disk.
static ReplayEnd replay_ram(const Snapshot &frames, const Cfg &cfg, Shared &sh,
                            int max_cycles) {
    if (frames.empty()) return ReplayEnd::Done;

    Child c;
    if (!cfg.player.empty()) {
        c = spawn({"/bin/sh", "-c", cfg.player}, true, false);
    } else {
        char rate[16];
        std::snprintf(rate, sizeof rate, "%d", cfg.playback_fps);
        c = spawn({"ffplay", "-hide_banner", "-loglevel", "error",
                   "-fflags", "nobuffer", "-flags", "low_delay",
                   "-probesize", "32", "-analyzeduration", "0",
                   "-window_title", "SLOW-MO REPLAY  (q here or any key in terminal = stop)",
                   "-autoexit", "-f", "mjpeg", "-framerate", rate, "-i", "pipe:0"},
                  true, true);
    }
    if (c.pid < 0) return ReplayEnd::PlayerFail;
    fcntl(c.fd, F_SETFL, O_NONBLOCK);

    const bool watch_keys = sh.interactive.load();
    const double period = 1.0 / cfg.playback_fps;
    const double start = now_mono();
    double next = start;
    size_t idx = 0;
    int cycles = 0;
    ReplayEnd end = ReplayEnd::Done;
    bool done = false;

    auto broken_pipe = [&]() {
        return (cycles == 0 && now_mono() - start < 1.5) ? ReplayEnd::PlayerFail
                                                         : ReplayEnd::WindowClosed;
    };

    while (!done) {
        if (g_stop) { end = ReplayEnd::Stop; break; }

        // wait for this frame's deadline, watching the terminal meanwhile
        double t = now_mono();
        if (t < next) {
            int ms = (int)((next - t) * 1000.0);
            if (ms > 100) ms = 100;
            if (ms < 1) ms = 1;
            pollfd p{0, POLLIN, 0};
            int r = poll(watch_keys ? &p : nullptr, watch_keys ? 1 : 0, ms);
            if (r > 0 && (p.revents & POLLIN)) { drain_stdin(); end = ReplayEnd::Key; break; }
            continue;
        }

        // write one whole frame (non-blocking, so a paused player can't
        // freeze us — terminal keys keep working while the pipe is full)
        const FrameData &f = *frames[idx];
        size_t off = 0;
        while (off < f.size()) {
            if (g_stop) { end = ReplayEnd::Stop; done = true; break; }
            pollfd ps[2] = {{0, POLLIN, 0}, {c.fd, POLLOUT, 0}};
            pollfd *pp = watch_keys ? ps : ps + 1;
            int r = poll(pp, watch_keys ? 2 : 1, 100);
            if (r <= 0) continue;
            if (watch_keys && (ps[0].revents & POLLIN)) {
                drain_stdin(); end = ReplayEnd::Key; done = true; break;
            }
            pollfd &wp = ps[1]; // pipe entry, whether 1 or 2 fds were polled
            if (wp.revents & (POLLERR | POLLHUP)) { end = broken_pipe(); done = true; break; }
            if (wp.revents & POLLOUT) {
                ssize_t w = write(c.fd, f.data() + off, f.size() - off);
                if (w > 0) off += (size_t)w;
                else if (w < 0 && errno != EAGAIN && errno != EINTR) {
                    end = broken_pipe(); done = true; break;
                }
            }
        }
        if (done) break;

        if (++idx == frames.size()) {
            idx = 0;
            cycles++;
            if (max_cycles > 0 && cycles >= max_cycles) { end = ReplayEnd::Done; break; }
        }
        next += period;
        double t2 = now_mono();
        if (next < t2 - 0.25) next = t2; // player was paused/stalled: no burst catch-up
    }

    close(c.fd); // EOF -> ffplay -autoexit closes after playing what's queued
    reap(c.pid, (end == ReplayEnd::Done || end == ReplayEnd::Key) ? 3000 : 300);
    return end;
}

// Fallback: open the saved file in a detached looping ffplay window
// (the mechanism the original Python script used for 'r').
static pid_t replay_file(const std::string &path) {
    Child c = spawn({"ffplay", "-hide_banner", "-loglevel", "error",
                     "-window_title", "slow-mo replay: " + basename_of(path),
                     "-loop", "0", path},
                    false, true);
    return c.pid;
}

// ------------------------------------------------------------------- saver

static std::string clip_path(const Cfg &cfg) {
    char stamp[32];
    time_t t = time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", &tmv);
    std::string base = cfg.out_dir + "/slowmo_" + stamp;
    std::string path = base + ".avi";
    for (int i = 2; access(path.c_str(), F_OK) == 0 && i < 100; i++)
        path = base + "_" + std::to_string(i) + ".avi";
    return path;
}

static void saver_thread(std::shared_ptr<const Snapshot> snap, Cfg cfg,
                         Shared *sh, std::string path) {
    double t0 = now_mono();
    std::string err;
    uint64_t bytes = 0;
    for (const auto &f : *snap) bytes += f->size();
    bool ok = avi::write_mjpg(path, *snap, cfg.width, cfg.height,
                              cfg.playback_fps, err);
    double dt = now_mono() - t0;
    char msg[512];
    if (ok) {
        double real = (double)snap->size() / cfg.capture_fps;
        double play = (double)snap->size() / cfg.playback_fps;
        std::snprintf(msg, sizeof msg,
                      "saved %s  (%zu frames, %.1f MB in %.2fs | %.1fs real -> %.1fs playback, %.0fx slow-mo)",
                      path.c_str(), snap->size(), bytes / 1e6, dt, real, play,
                      (double)cfg.capture_fps / cfg.playback_fps);
        sh->set_last_path(path);
    } else {
        std::snprintf(msg, sizeof msg, "save FAILED: %s", err.c_str());
    }
    sh->msgs.push(msg);
    sh->save_busy = false;
}

// ---------------------------------------------------------- V4L2 capture

static void v4l2_capture_thread(Cfg cfg, Shared *sh) {
    int fd = open(cfg.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) { sh->set_error(errno_str("open " + cfg.device)); return; }

    v4l2_capability cap{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) != 0) {
        sh->set_error(errno_str("VIDIOC_QUERYCAP")); close(fd); return;
    }
    uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps
                                                              : cap.capabilities;
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE) || !(caps & V4L2_CAP_STREAMING)) {
        sh->set_error(cfg.device + " does not support streaming video capture");
        close(fd); return;
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = cfg.width;
    fmt.fmt.pix.height = cfg.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        sh->set_error(errno_str("VIDIOC_S_FMT")); close(fd); return;
    }
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
        sh->set_error(cfg.device + " does not offer MJPG — check "
                      "`v4l2-ctl -d " + cfg.device + " --list-formats-ext`");
        close(fd); return;
    }
    int aw = fmt.fmt.pix.width, ah = fmt.fmt.pix.height;

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe = {1, (uint32_t)cfg.capture_fps};
    double afps = cfg.capture_fps;
    if (xioctl(fd, VIDIOC_S_PARM, &parm) == 0 &&
        parm.parm.capture.timeperframe.numerator > 0) {
        afps = (double)parm.parm.capture.timeperframe.denominator /
               parm.parm.capture.timeperframe.numerator;
    }
    {
        char m[256];
        std::snprintf(m, sizeof m, "camera: %dx%d MJPG @ %.0f fps%s", aw, ah, afps,
                      (aw != cfg.width || ah != cfg.height ||
                       (int)(afps + 0.5) != cfg.capture_fps)
                          ? "  (driver adjusted the requested mode!)" : "");
        sh->msgs.push(m);
    }

    v4l2_requestbuffers req{};
    req.count = 6;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) != 0 || req.count < 2) {
        sh->set_error(errno_str("VIDIOC_REQBUFS")); close(fd); return;
    }
    struct Buf { void *ptr; size_t len; };
    std::vector<Buf> bufs(req.count);
    for (uint32_t i = 0; i < req.count; i++) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (xioctl(fd, VIDIOC_QUERYBUF, &b) != 0) {
            sh->set_error(errno_str("VIDIOC_QUERYBUF")); close(fd); return;
        }
        bufs[i].len = b.length;
        bufs[i].ptr = mmap(nullptr, b.length, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd, b.m.offset);
        if (bufs[i].ptr == MAP_FAILED) {
            sh->set_error(errno_str("mmap")); close(fd); return;
        }
        if (xioctl(fd, VIDIOC_QBUF, &b) != 0) {
            sh->set_error(errno_str("VIDIOC_QBUF")); close(fd); return;
        }
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        sh->set_error(errno_str("VIDIOC_STREAMON")); close(fd); return;
    }

    uint32_t expected_seq = 0;
    bool have_seq = false;
    while (sh->alive && !g_stop) {
        pollfd p{fd, POLLIN, 0};
        int r = poll(&p, 1, 200);
        if (r < 0) {
            if (errno == EINTR) continue;
            sh->set_error(errno_str("poll(camera)"));
            break;
        }
        if (r == 0) continue;

        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            if (errno == EAGAIN) continue;
            sh->set_error(errno_str("VIDIOC_DQBUF"));
            break;
        }
        const uint8_t *d = (const uint8_t *)bufs[b.index].ptr;
        // skip broken frames some UVC cams emit right after stream start
        if (b.bytesused > 4 && d[0] == 0xFF && d[1] == 0xD8) {
            sh->ring.push(std::make_shared<FrameData>(d, d + b.bytesused));
            if (have_seq && b.sequence != expected_seq)
                sh->drops += b.sequence - expected_seq;
            expected_seq = b.sequence + 1;
            have_seq = true;
        }
        if (xioctl(fd, VIDIOC_QBUF, &b) != 0) {
            sh->set_error(errno_str("VIDIOC_QBUF"));
            break;
        }
    }

    xioctl(fd, VIDIOC_STREAMOFF, &type);
    for (auto &bf : bufs)
        if (bf.ptr && bf.ptr != MAP_FAILED) munmap(bf.ptr, bf.len);
    close(fd);
}

// -------------------------------------------- test capture from .mjpeg file

static void file_capture_thread(Cfg cfg, Shared *sh) {
    FILE *fh = std::fopen(cfg.mjpeg_file.c_str(), "rb");
    if (!fh) { sh->set_error(errno_str("open " + cfg.mjpeg_file)); return; }
    std::vector<uint8_t> data;
    {
        uint8_t chunk[1 << 16];
        size_t r;
        while ((r = std::fread(chunk, 1, sizeof chunk, fh)) > 0)
            data.insert(data.end(), chunk, chunk + r);
        std::fclose(fh);
    }
    // split on JPEG SOI/EOI markers
    std::vector<std::pair<size_t, size_t>> spans;
    size_t pos = 0;
    while (pos + 4 <= data.size()) {
        if (data[pos] == 0xFF && data[pos + 1] == 0xD8) {
            size_t e = pos + 2;
            while (e + 2 <= data.size() &&
                   !(data[e] == 0xFF && data[e + 1] == 0xD9)) e++;
            if (e + 2 > data.size()) break;
            spans.emplace_back(pos, e + 2 - pos);
            pos = e + 2;
        } else pos++;
    }
    if (spans.empty()) { sh->set_error("no JPEG frames in " + cfg.mjpeg_file); return; }
    {
        char m[160];
        std::snprintf(m, sizeof m, "test input: %zu frames from %s (looping @ %d fps)",
                      spans.size(), cfg.mjpeg_file.c_str(), cfg.capture_fps);
        sh->msgs.push(m);
    }
    const double period = 1.0 / cfg.capture_fps;
    double next = now_mono();
    size_t i = 0;
    while (sh->alive && !g_stop) {
        double t = now_mono();
        if (t < next) {
            int ms = (int)((next - t) * 1000.0);
            poll(nullptr, 0, ms > 50 ? 50 : (ms < 1 ? 1 : ms));
            continue;
        }
        const auto &s = spans[i];
        sh->ring.push(std::make_shared<FrameData>(data.begin() + s.first,
                                                  data.begin() + s.first + s.second));
        i = (i + 1) % spans.size();
        next += period;
        if (next < t - 0.5) next = t;
    }
}

// ------------------------------------------------------------- interactive

struct RawTerm {
    termios old{};
    bool active = false;
    RawTerm() {
        if (isatty(0) && tcgetattr(0, &old) == 0) {
            termios t = old;
            t.c_lflag &= ~(ICANON | ECHO); // cbreak; keep ISIG so Ctrl-C works
            t.c_cc[VMIN] = 1;
            t.c_cc[VTIME] = 0;
            active = tcsetattr(0, TCSADRAIN, &t) == 0;
        }
    }
    ~RawTerm() {
        if (active) tcsetattr(0, TCSADRAIN, &old);
    }
};

static void print_msgs(Shared &sh) {
    for (const auto &m : sh.msgs.drain())
        std::printf("\r\033[K  %s\n", m.c_str());
}

// Start the background save; returns the snapshot (or nullptr if empty).
static std::shared_ptr<const Snapshot> start_save(const Cfg &cfg, Shared &sh,
                                                  std::thread &saver) {
    auto snap = sh.ring.snapshot();
    if (snap->empty()) {
        sh.msgs.push("buffer is empty — is the camera running?");
        return nullptr;
    }
    if (saver.joinable()) saver.join(); // previous save (finishes in ~a second)
    std::string path = clip_path(cfg);
    sh.save_busy = true;
    saver = std::thread(saver_thread, snap, cfg, &sh, path);
    char m[256];
    std::snprintf(m, sizeof m, "saving %zu frames -> %s (in background)",
                  snap->size(), basename_of(path).c_str());
    sh.msgs.push(m);
    return snap;
}

static void run_replay(const Snapshot &snap, const Cfg &cfg, Shared &sh,
                       std::thread &saver, pid_t &file_player) {
    double real = (double)snap.size() / cfg.capture_fps;
    double play = (double)snap.size() / cfg.playback_fps;
    std::printf("\r\033[K  replay: %.1fs real -> %.1fs @ %d fps (%.0fx slow-mo), "
                "looping — press any key to stop\n",
                real, play, cfg.playback_fps,
                (double)cfg.capture_fps / cfg.playback_fps);
    std::fflush(stdout);
    ReplayEnd e = replay_ram(snap, cfg, sh, 0);
    if (e == ReplayEnd::PlayerFail) {
        sh.msgs.push("replay window failed to start (is ffplay installed and "
                     "DISPLAY set?) — falling back to playing the saved file");
        if (saver.joinable()) saver.join(); // make sure the file exists
        std::string p = sh.get_last_path();
        if (!p.empty()) {
            if (file_player > 0) { kill(file_player, SIGTERM); reap(file_player, 200); }
            file_player = replay_file(p);
            if (file_player < 0)
                sh.msgs.push("could not start ffplay — install with: sudo apt install ffmpeg");
        }
    }
}

static int interactive(const Cfg &cfg, Shared &sh, std::thread &saver) {
    if (!isatty(0)) {
        std::fprintf(stderr, "stdin is not a terminal — use --selftest N for "
                             "non-interactive runs\n");
        return 1;
    }
    sh.interactive = true;
    RawTerm raw;
    std::shared_ptr<const Snapshot> last_snap;
    pid_t file_player = -1;

    std::printf("Recording %dx%d @ %d fps from %s — keeping the last %.0f s in RAM\n",
                cfg.width, cfg.height, cfg.capture_fps,
                cfg.mjpeg_file.empty() ? cfg.device.c_str() : cfg.mjpeg_file.c_str(),
                cfg.buffer_seconds);
    std::printf("[SPACE/s] save last %.0f s + instant replay   [r] replay again   [q] quit\n",
                cfg.buffer_seconds);

    int rc = 0;
    while (!g_stop) {
        std::string err = sh.get_error();
        if (!err.empty()) {
            std::printf("\r\033[K\ncapture error: %s\n", err.c_str());
            rc = 1;
            break;
        }
        print_msgs(sh);

        size_t fill = sh.ring.size() * 100 / cfg.max_frames();
        if (fill > 100) fill = 100;
        std::string last = sh.get_last_path();
        std::printf("\r\033[K  buffer %3zu%% | %6.1f fps | drops %llu%s | last: %s",
                    fill, sh.ring.measured_fps(),
                    (unsigned long long)sh.drops.load(),
                    sh.save_busy ? " | saving..." : "",
                    last.empty() ? "-" : basename_of(last).c_str());
        std::fflush(stdout);

        pollfd p{0, POLLIN, 0};
        int r = poll(&p, 1, 200);
        if (r <= 0) continue;
        char key = 0;
        if (read(0, &key, 1) != 1) continue;

        if (key == 'q' || key == 'Q' || key == 3) break;
        if (key == ' ' || key == 's' || key == 'S') {
            auto snap = start_save(cfg, sh, saver);
            if (snap) {
                last_snap = snap;
                print_msgs(sh);
                if (cfg.autoreplay)
                    run_replay(*snap, cfg, sh, saver, file_player);
            }
        } else if (key == 'r' || key == 'R') {
            if (last_snap)
                run_replay(*last_snap, cfg, sh, saver, file_player);
            else
                sh.msgs.push("nothing saved yet — press SPACE/s first");
        }
    }

    std::printf("\r\033[K\n");
    if (file_player > 0) { kill(file_player, SIGTERM); reap(file_player, 200); }
    return rc;
}

// ---------------------------------------------------------------- selftest

static int selftest(const Cfg &cfg, Shared &sh) {
    std::printf("selftest: capturing %d s ...\n", cfg.selftest);
    double deadline = now_mono() + cfg.selftest;
    while (now_mono() < deadline && !g_stop) {
        std::string err = sh.get_error();
        if (!err.empty()) { std::printf("capture error: %s\n", err.c_str()); return 1; }
        poll(nullptr, 0, 100);
        for (const auto &m : sh.msgs.drain()) std::printf("  %s\n", m.c_str());
    }
    std::printf("measured capture rate: %.1f fps, %zu frames buffered, %llu drops\n",
                sh.ring.measured_fps(), sh.ring.size(),
                (unsigned long long)sh.drops.load());

    auto snap = sh.ring.snapshot();
    if (snap->empty()) { std::printf("selftest FAILED: no frames captured\n"); return 1; }

    std::string path = clip_path(cfg);
    saver_thread(snap, cfg, &sh, path); // synchronous here
    for (const auto &m : sh.msgs.drain()) std::printf("%s\n", m.c_str());
    if (sh.get_last_path() != path) return 1;

    if (cfg.selftest_replay) {
        std::printf("replaying one pass through the player ...\n");
        ReplayEnd e = replay_ram(*snap, cfg, sh, 1);
        if (e == ReplayEnd::PlayerFail || e == ReplayEnd::WindowClosed) {
            std::printf("selftest replay FAILED (player exited early)\n");
            return 1;
        }
        std::printf("replay pass done\n");
    }
    std::printf("selftest OK\n");
    return 0;
}

// -------------------------------------------------------------------- main

int main(int argc, char **argv) {
    Cfg cfg;
    if (!parse_args(argc, argv, cfg)) return 2;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    if (cfg.mjpeg_file.empty() && access(cfg.device.c_str(), F_OK) != 0) {
        std::fprintf(stderr, "%s not found — is the camera plugged in?\n",
                     cfg.device.c_str());
        return 1;
    }
    if (!mkdirs(cfg.out_dir)) {
        std::fprintf(stderr, "%s\n", errno_str("cannot create " + cfg.out_dir).c_str());
        return 1;
    }
    if (cfg.player.empty() && !has_cmd("ffplay") && !(cfg.selftest && !cfg.selftest_replay))
        std::fprintf(stderr, "warning: ffplay not found — saving will work but "
                             "replay will not (sudo apt install ffmpeg)\n");

    Shared sh(cfg.max_frames(), (size_t)cfg.capture_fps);
    std::thread cap(cfg.mjpeg_file.empty() ? v4l2_capture_thread
                                           : file_capture_thread,
                    cfg, &sh);
    std::thread saver;

    int rc;
    if (cfg.selftest > 0)
        rc = selftest(cfg, sh);
    else
        rc = interactive(cfg, sh, saver);

    sh.alive = false;
    g_stop = 1;
    cap.join();
    if (saver.joinable()) {
        if (sh.save_busy) std::printf("finishing save ...\n");
        saver.join();
    }
    for (const auto &m : sh.msgs.drain()) std::printf("  %s\n", m.c_str());
    return rc;
}
