// slowmo_cam.cpp — standalone rolling slow-motion replay recorder (C++).
//
// Continuously captures MJPEG frames from a V4L2 camera (OV9281 UVC,
// 1280x720 @ 120 fps) into a RAM ring buffer holding the last
// BUFFER_SECONDS of footage. Frames stay compressed (never decoded),
// so CPU and memory stay low.
//
// Keys:
//   SPACE / s : save the last 8 s AND replay it immediately in slow motion
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
// Web (same process — the camera is opened exactly once, so the live view
// and the VAR recorder run simultaneously by construction):
//   GET  /        control page: live view + save/replay buttons + status
//   GET  /stream  live camera as multipart MJPEG (?fps=N to limit rate)
//   GET  /replay  the last saved clip, streamed from RAM at playback fps
//   POST /save    same as pressing 's': snapshot + save + make replayable
//   GET  /status  JSON: fps, buffer %, drops, last clip, ...
// Default port 8080 (--port N, --port 0 disables). Frames are passed
// through compressed — streaming adds no measurable CPU load.
//
// Build:  g++ -O2 -std=c++17 -Wall -o slowmo_cam slowmo_cam.cpp -lpthread
// Deps:   ffplay (from the ffmpeg package) is used only as the replay window.
//
// Usage:
//   ./slowmo_cam                    # interactive
//   ./slowmo_cam --selftest 3      # capture 3 s, auto-save, verify, exit
//   ./slowmo_cam --help            # all options

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <csignal>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <dirent.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
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
    double buffer_seconds = 8.0;  // how much history to keep in RAM
    std::string out_dir = "~/recordings";
    std::string player;           // custom replay command (stdin = mjpeg); empty = ffplay
    bool autoreplay = true;       // 's' also starts the replay
    int selftest = 0;             // capture N seconds, save, verify, exit
    bool selftest_replay = false; // selftest also plays one replay pass
    std::string mjpeg_file;       // test input: raw .mjpeg stream instead of camera
    int max_clips = 20;           // keep at most N saved clips (0 = unlimited)
    int http_port = 8080;         // web live view / control (0 = disabled)
    int http_fps = 30;            // default live-stream rate served to browsers
    std::string scores_script = "~/src/var/score_function/compute_scores.py";
    std::string scores_file;      // tournament state; default <out_dir>/tournament.tsv
    bool no_scores = false;       // disable the web scoreboard

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
        "  --seconds S          seconds of history kept in RAM (default 8)\n"
        "  --max-clips N        keep only the newest N saved clips in out-dir,\n"
        "                       older slowmo_*.avi are deleted (default 20, 0 = keep all)\n"
        "  --out-dir DIR        where clips are saved (default ~/recordings)\n"
        "  --player CMD         replay command run via sh -c, mjpeg on its stdin\n"
        "                       (default: built-in ffplay window)\n"
        "  --no-autoreplay      do not start the replay automatically after save\n"
        "  --selftest N         capture N seconds, save, verify, exit (no keys)\n"
        "  --selftest-replay    selftest also plays the clip once through the player\n"
        "  --mjpeg-file PATH    read frames from a raw .mjpeg file instead of a\n"
        "                       camera (testing; frames must match --width/--height)\n"
        "  --port N             web live view + control port (default 8080, 0 = off)\n"
        "  --http-fps N         live-stream rate served to browsers (default 30;\n"
        "                       clients may lower it per-connection via /stream?fps=N)\n"
        "  --scores-script PATH python bridge computing the tournament scores\n"
        "                       (default ~/src/var/score_function/compute_scores.py)\n"
        "  --scores-file PATH   tournament state (default <out-dir>/tournament.tsv)\n"
        "  --no-scores          disable the web scoreboard\n"
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
        else if (a == "--max-clips") { if (!(v = need(i))) return false; cfg.max_clips = atoi(v); }
        else if (a == "--port") { if (!(v = need(i))) return false; cfg.http_port = atoi(v); }
        else if (a == "--http-fps") { if (!(v = need(i))) return false; cfg.http_fps = atoi(v); }
        else if (a == "--scores-script") { if (!(v = need(i))) return false; cfg.scores_script = v; }
        else if (a == "--scores-file") { if (!(v = need(i))) return false; cfg.scores_file = v; }
        else if (a == "--no-scores") cfg.no_scores = true;
        else if (a == "--help" || a == "-h") { usage(argv[0]); exit(0); }
        else { std::fprintf(stderr, "unknown option: %s\n", a.c_str()); return false; }
    }
    cfg.out_dir = expand_home(cfg.out_dir);
    cfg.scores_script = expand_home(cfg.scores_script);
    cfg.scores_file = expand_home(cfg.scores_file);
    if (cfg.scores_file.empty()) cfg.scores_file = cfg.out_dir + "/tournament.tsv";
    if (cfg.width <= 0 || cfg.height <= 0 || cfg.capture_fps <= 0 ||
        cfg.playback_fps <= 0 || cfg.buffer_seconds <= 0) {
        std::fprintf(stderr, "width/height/fps/playback-fps/seconds must be > 0\n");
        return false;
    }
    if (cfg.http_port < 0 || cfg.http_port > 65535) {
        std::fprintf(stderr, "--port must be 0..65535\n");
        return false;
    }
    if (cfg.http_fps <= 0) cfg.http_fps = 30;
    if (cfg.http_fps > cfg.capture_fps) cfg.http_fps = cfg.capture_fps;
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
    Frame latest() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.empty() ? nullptr : q_.back();
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

struct ScoreBoard; // tournament rankings, defined after the capture threads

struct Shared {
    Ring ring;
    MessageQueue msgs;
    ScoreBoard *scores = nullptr; // web scoreboard (null = disabled)
    std::atomic<bool> alive{true};
    std::atomic<bool> save_busy{false};
    std::atomic<bool> interactive{false};
    std::atomic<uint64_t> drops{0};
    std::mutex err_m;
    std::string error;
    std::mutex path_m;
    std::string last_path; // last successfully saved file
    std::mutex snap_m;
    std::shared_ptr<const Snapshot> last_snap; // last saved clip, for replays
    std::mutex saver_m;
    std::thread saver; // background AVI writer (at most one at a time)

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
    void set_last_snap(std::shared_ptr<const Snapshot> s) {
        std::lock_guard<std::mutex> lk(snap_m);
        last_snap = std::move(s);
    }
    std::shared_ptr<const Snapshot> get_last_snap() {
        std::lock_guard<std::mutex> lk(snap_m);
        return last_snap;
    }
    void join_saver() {
        std::lock_guard<std::mutex> lk(saver_m);
        if (saver.joinable()) saver.join();
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

// Delete the oldest slowmo_*.avi in out_dir beyond cfg.max_clips. Clip names
// start with a timestamp, so lexicographic order == chronological order.
static void prune_clips(const Cfg &cfg, Shared *sh) {
    if (cfg.max_clips <= 0) return;
    DIR *d = opendir(cfg.out_dir.c_str());
    if (!d) return;
    std::vector<std::string> clips;
    dirent *e;
    while ((e = readdir(d)) != nullptr) {
        std::string n = e->d_name;
        if (n.size() > 11 && n.compare(0, 7, "slowmo_") == 0 &&
            n.compare(n.size() - 4, 4, ".avi") == 0)
            clips.push_back(n);
    }
    closedir(d);
    if ((int)clips.size() <= cfg.max_clips) return;
    std::sort(clips.begin(), clips.end());
    size_t doomed = clips.size() - (size_t)cfg.max_clips;
    for (size_t i = 0; i < doomed; i++)
        unlink((cfg.out_dir + "/" + clips[i]).c_str());
    char msg[96];
    std::snprintf(msg, sizeof msg, "pruned %zu old clip%s (keeping newest %d)",
                  doomed, doomed == 1 ? "" : "s", cfg.max_clips);
    sh->msgs.push(msg);
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
    if (ok) prune_clips(cfg, sh);
    sh->save_busy = false;
}

// Snapshot the ring and start the background save. Serialized by saver_m so
// the terminal 's' key and the web POST /save can fire concurrently. Returns
// the snapshot (nullptr if the buffer is empty); out_path gets the clip path.
static std::shared_ptr<const Snapshot> save_now(const Cfg &cfg, Shared &sh,
                                                std::string *out_path = nullptr) {
    auto snap = sh.ring.snapshot();
    if (snap->empty()) {
        sh.msgs.push("buffer is empty — is the camera running?");
        return nullptr;
    }
    std::string path;
    {
        std::lock_guard<std::mutex> lk(sh.saver_m);
        if (sh.saver.joinable()) sh.saver.join(); // previous save (~a second)
        path = clip_path(cfg); // after join, so the previous file is visible
        sh.save_busy = true;
        sh.saver = std::thread(saver_thread, snap, cfg, &sh, path);
    }
    sh.set_last_snap(snap);
    if (out_path) *out_path = path;
    char m[256];
    std::snprintf(m, sizeof m, "saving %zu frames -> %s (in background)",
                  snap->size(), basename_of(path).c_str());
    sh.msgs.push(m);
    return snap;
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

// ------------------------------------------------------------- score board
// Tournament rankings for the web page. Match results (who beat whom) are a
// simple TSV file; the scores come from the *bias* PageRank in
// score_function/page_rank_billiardino_algorithm_bias.py, called through
// score_function/compute_scores.py — the Python file stays the single
// source of truth for the algorithm, so edits there flow to the live board.

// Run `python3 script` with `input` on stdin, collect stdout. Bounded by
// timeout_ms (numpy's import on a Pi can take a couple of seconds cold).
static bool run_script(const std::string &script, const std::string &input,
                       std::string &output, std::string &err, int timeout_ms) {
    int in_p[2], out_p[2];
    if (pipe2(in_p, O_CLOEXEC) != 0) { err = errno_str("pipe"); return false; }
    if (pipe2(out_p, O_CLOEXEC) != 0) {
        err = errno_str("pipe");
        close(in_p[0]); close(in_p[1]);
        return false;
    }
    pid_t pid = fork();
    if (pid < 0) {
        err = errno_str("fork");
        close(in_p[0]); close(in_p[1]); close(out_p[0]); close(out_p[1]);
        return false;
    }
    if (pid == 0) {
        dup2(in_p[0], 0);
        dup2(out_p[1], 1); // stderr stays: script errors land in our log
        execlp("python3", "python3", script.c_str(), (char *)nullptr);
        _exit(127);
    }
    close(in_p[0]);
    close(out_p[1]);
    bool wok = true;
    size_t off = 0;
    while (off < input.size()) { // matrix is a few KB — fits the pipe buffer
        ssize_t w = write(in_p[1], input.data() + off, input.size() - off);
        if (w < 0) { if (errno == EINTR) continue; wok = false; break; }
        off += (size_t)w;
    }
    close(in_p[1]);
    const double deadline = now_mono() + timeout_ms / 1000.0;
    char buf[4096];
    bool timed_out = false;
    for (;;) {
        if (g_stop) { timed_out = true; break; } // shutting down: abort
        double left = deadline - now_mono();
        if (left <= 0) { timed_out = true; break; }
        pollfd p{out_p[0], POLLIN, 0};
        int r = poll(&p, 1, 200);
        if (r < 0 && errno != EINTR) break;
        if (r <= 0) continue;
        ssize_t n = read(out_p[0], buf, sizeof buf);
        if (n < 0) { if (errno == EINTR) continue; break; }
        if (n == 0) break; // EOF
        output.append(buf, (size_t)n);
    }
    close(out_p[0]);
    if (timed_out) {
        kill(pid, SIGKILL);
        waitpid(pid, nullptr, 0);
        err = "score script timed out";
        return false;
    }
    int st = 0;
    waitpid(pid, &st, 0);
    if (!wok) { err = "write to score script failed"; return false; }
    if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) {
        err = "score script failed (exit " +
              std::to_string(WIFEXITED(st) ? WEXITSTATUS(st) : -1) +
              ", see terminal for its stderr)";
        return false;
    }
    return true;
}

static void json_escape(std::string &dst, const std::string &s) {
    for (char c : s) {
        if (c == '"' || c == '\\') dst += '\\';
        dst += c; // names are sanitized: no control characters left
    }
}

struct ScoreBoard {
    struct Game { int a, b; }; // one game's points, from team A's perspective
    struct Match {
        int a, b;                // team indices; each pair plays at most once
        std::vector<Game> games; // may be empty (legacy entries)
        int winner;              // == a or b, derived from games won
    };

    std::mutex m;
    std::string file;   // TSV persistence
    std::string script; // compute_scores.py
    std::vector<std::string> teams;
    std::vector<Match> matches;
    std::vector<double> cache_bias, cache_plain; // per-team; empty = stale
    std::string cache_err;

    static constexpr size_t kMaxTeams = 64;
    static constexpr size_t kMaxNameLen = 40;

    // printable, no tabs/newlines (TSV + JSON safety), trimmed, length-capped
    static std::string sanitize_name(const std::string &s) {
        std::string out;
        for (char c : s)
            if ((unsigned char)c >= 0x20 && c != 0x7f) out += c;
        size_t a = out.find_first_not_of(' ');
        if (a == std::string::npos) return "";
        size_t b = out.find_last_not_of(' ');
        out = out.substr(a, b - a + 1);
        if (out.size() > kMaxNameLen) out.resize(kMaxNameLen);
        return out;
    }

    int find_or_add_locked(const std::string &name) {
        for (size_t i = 0; i < teams.size(); i++)
            if (teams[i] == name) return (int)i;
        if (teams.size() >= kMaxTeams) return -1;
        teams.push_back(name);
        return (int)teams.size() - 1;
    }

    int find_match_locked(int a, int b) const {
        for (size_t i = 0; i < matches.size(); i++)
            if ((matches[i].a == a && matches[i].b == b) ||
                (matches[i].a == b && matches[i].b == a)) return (int)i;
        return -1;
    }

    // -1 = tied overall (no winner), else the winning team's index
    static int winner_of(int a, int b, const std::vector<Game> &games) {
        int wa = 0, wb = 0;
        for (const auto &g : games) (g.a > g.b ? wa : wb)++;
        return wa == wb ? -1 : (wa > wb ? a : b);
    }

    // best-of-three: a complete match is exactly 2-0 or 2-1 in games
    static bool bo3_ok(const std::vector<Game> &gs, int &wa, int &wb,
                       std::string &err) {
        wa = wb = 0;
        for (const auto &g : gs) (g.a > g.b ? wa : wb)++;
        int hi = wa > wb ? wa : wb, lo = wa + wb - hi;
        if (hi != 2 || lo > 1) {
            err = "best-of-three: a match ends when a team wins its 2nd game "
                  "— enter 2 or 3 games, e.g. 10-9, 10-4  or  10-1, 5-10, 10-9";
            return false;
        }
        return true;
    }

    // resolve two team names to an existing head-to-head match; -1 + err if
    // either team or the pair's match does not exist
    int find_pair_locked(const std::string &an, const std::string &bn,
                         int &ai, int &bi, std::string &err) {
        ai = bi = -1;
        for (size_t i = 0; i < teams.size(); i++) {
            if (teams[i] == an) ai = (int)i;
            if (teams[i] == bn) bi = (int)i;
        }
        if (ai < 0) { err = "no team named \"" + an + "\""; return -1; }
        if (bi < 0) { err = "no team named \"" + bn + "\""; return -1; }
        int idx = find_match_locked(ai, bi);
        if (idx < 0) err = an + " vs " + bn + " has not been played yet";
        return idx;
    }

    static std::string games_str(const Match &mt) {
        std::string s;
        for (size_t i = 0; i < mt.games.size(); i++) {
            if (i) s += ",";
            s += std::to_string(mt.games[i].a) + "-" + std::to_string(mt.games[i].b);
        }
        return s;
    }

    // "10-1, 5-10, 10-9" -> games; rejects ties, junk, and empty input
    static bool parse_games(const std::string &s, std::vector<Game> &out,
                            std::string &err) {
        out.clear();
        const char *fmt_hint = "scores must look like: 10-1, 5-10, 10-9";
        size_t pos = 0;
        while (pos < s.size()) {
            while (pos < s.size() && (s[pos] == ' ' || s[pos] == ',')) pos++;
            if (pos >= s.size()) break;
            char *end;
            long x = strtol(s.c_str() + pos, &end, 10);
            if (end == s.c_str() + pos) { err = fmt_hint; return false; }
            const char *p = end;
            while (*p == ' ') p++;
            if (*p != '-') { err = fmt_hint; return false; }
            p++;
            while (*p == ' ') p++;
            char *end2;
            long y = strtol(p, &end2, 10);
            if (end2 == p) { err = fmt_hint; return false; }
            if (x < 0 || y < 0 || x > 999 || y > 999) {
                err = "game points out of range";
                return false;
            }
            if (x == y) {
                err = "a game cannot end tied (" + std::to_string(x) + "-" +
                      std::to_string(y) + ")";
                return false;
            }
            out.push_back({(int)x, (int)y});
            if (out.size() > 15) { err = "too many games in one match"; return false; }
            pos = (size_t)(end2 - s.c_str());
        }
        if (out.empty()) {
            err = "enter the game scores, e.g. 10-1, 5-10, 10-9";
            return false;
        }
        return true;
    }

    bool save_locked(std::string &err) {
        std::string tmp = file + ".tmp";
        FILE *fh = std::fopen(tmp.c_str(), "w");
        if (!fh) { err = errno_str("open " + tmp); return false; }
        bool ok = true;
        for (const auto &t : teams)
            ok = ok && std::fprintf(fh, "team\t%s\n", t.c_str()) > 0;
        for (const auto &mt : matches) {
            std::string g = games_str(mt);
            ok = ok && std::fprintf(fh, "match\t%d\t%d\t%s\t%d\n", mt.a, mt.b,
                                    g.empty() ? "-" : g.c_str(), mt.winner) > 0;
        }
        ok = std::fflush(fh) == 0 && ok && !ferror(fh);
        std::fclose(fh);
        if (!ok || rename(tmp.c_str(), file.c_str()) != 0) {
            err = errno_str("write " + file);
            unlink(tmp.c_str());
            return false;
        }
        return true;
    }

    // Load the TSV; on first run seed with last year's group A (dummy values
    // from the score_function scripts) so the board shows something real.
    void load_or_seed(MessageQueue &msgs) {
        std::lock_guard<std::mutex> lk(m);
        FILE *fh = std::fopen(file.c_str(), "r");
        if (fh) {
            char line[256];
            while (std::fgets(line, sizeof line, fh)) {
                std::string s(line);
                while (!s.empty() && (s.back() == '\n' || s.back() == '\r'))
                    s.pop_back();
                if (s.compare(0, 5, "team\t") == 0) {
                    std::string nm = sanitize_name(s.substr(5));
                    if (teams.size() < kMaxTeams) // placeholder keeps indices stable
                        teams.push_back(nm.empty()
                                            ? "Team " + std::to_string(teams.size() + 1)
                                            : nm);
                } else if (s.compare(0, 6, "match\t") == 0) {
                    int a = -1, b = -1, w = -1;
                    char gbuf[192] = {0};
                    int got = std::sscanf(s.c_str() + 6, "%d\t%d\t%191[^\t]\t%d",
                                          &a, &b, gbuf, &w);
                    if (got == 2) w = a; // legacy "match\twinner\tloser"
                    else if (got != 4) continue;
                    if (a < 0 || b < 0 || a == b || (w != a && w != b) ||
                        a >= (int)teams.size() || b >= (int)teams.size())
                        continue;
                    if (find_match_locked(a, b) >= 0) continue; // one per pair
                    Match mt{a, b, {}, w};
                    if (got == 4 && std::strcmp(gbuf, "-") != 0) {
                        std::vector<Game> gs;
                        std::string gerr;
                        if (parse_games(gbuf, gs, gerr) &&
                            winner_of(a, b, gs) >= 0) {
                            mt.games = gs;
                            mt.winner = winner_of(a, b, gs);
                        }
                    }
                    matches.push_back(mt);
                }
            }
            std::fclose(fh);
            msgs.push("scoreboard: " + std::to_string(teams.size()) + " teams, " +
                      std::to_string(matches.size()) + " matches from " + file);
            return;
        }
        static const int demo[9][9] = {
            {0, 0, 1, 1, 1, 1, 1, 1, 1}, {1, 0, 1, 1, 1, 1, 0, 0, 1},
            {0, 0, 0, 1, 1, 0, 1, 1, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 1},
            {0, 0, 0, 1, 0, 0, 0, 0, 0}, {0, 0, 1, 1, 0, 0, 1, 0, 1},
            {0, 0, 0, 1, 0, 0, 0, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0}};
        for (int i = 0; i < 9; i++)
            teams.push_back("Team " + std::to_string(i + 1));
        for (int i = 0; i < 9; i++)
            for (int j = 0; j < 9; j++)
                if (demo[i][j]) // plausible fake best-of-three (2-0) for the demo
                    matches.push_back({i, j,
                                       {{10, (3 * i + 5 * j + 7) % 10},
                                        {10, (7 * i + 2 * j + 3) % 10}},
                                       i});
        std::string err;
        if (!save_locked(err)) msgs.push("scoreboard: " + err);
        msgs.push("scoreboard: seeded demo tournament (last year's group A) -> " + file);
    }

    void recompute_locked() {
        cache_bias.clear();
        cache_plain.clear();
        cache_err.clear();
        const size_t n = teams.size();
        if (n == 0) return;
        std::vector<double> M(n * n, 0.0); // M[i][j] = 1 if i won the match vs j
        for (const auto &mt : matches) {
            int l = mt.winner == mt.a ? mt.b : mt.a;
            M[(size_t)mt.winner * n + l] += 1.0;
        }
        std::string in = std::to_string(n);
        char num[32];
        for (size_t i = 0; i < n * n; i++) {
            std::snprintf(num, sizeof num, " %g", M[i]);
            in += num;
        }
        in += "\n";
        std::string out, err;
        if (!run_script(script, in, out, err, 30000)) { cache_err = err; return; }
        const char *p = out.c_str();
        char *end;
        for (size_t i = 0; i < 2 * n; i++) { // line 1 bias, line 2 classic
            double v = strtod(p, &end);
            if (end == p) {
                cache_err = "unexpected score script output";
                cache_bias.clear();
                cache_plain.clear();
                return;
            }
            (i < n ? cache_bias : cache_plain).push_back(v);
            p = end;
        }
    }

    std::string scores_json() {
        std::lock_guard<std::mutex> lk(m);
        const size_t n = teams.size();
        if (cache_bias.size() != n || cache_plain.size() != n) recompute_locked();
        if (!cache_err.empty()) {
            std::string j = "{\"ok\":false,\"error\":\"";
            json_escape(j, cache_err);
            return j + "\"}";
        }
        std::vector<int> wins(n, 0), losses(n, 0);
        for (const auto &mt : matches) {
            wins[mt.winner]++;
            losses[mt.winner == mt.a ? mt.b : mt.a]++;
        }
        // teams in index order with both scores; the page sorts client-side,
        // so switching algorithms needs no server round-trip
        std::string j = "{\"ok\":true,\"teams\":[";
        char buf[160];
        for (size_t i = 0; i < n; i++) {
            if (i) j += ',';
            j += "{\"name\":\"";
            json_escape(j, teams[i]);
            std::snprintf(buf, sizeof buf,
                          "\",\"bias\":%.6f,\"plain\":%.6f,\"wins\":%d,\"losses\":%d}",
                          cache_bias[i], cache_plain[i], wins[i], losses[i]);
            j += buf;
        }
        j += "],\"matches\":[";
        for (size_t k = 0; k < matches.size(); k++) {
            const Match &mt = matches[k];
            if (k) j += ',';
            std::snprintf(buf, sizeof buf,
                          "{\"a\":%d,\"b\":%d,\"winner\":%d,\"games\":[",
                          mt.a, mt.b, mt.winner);
            j += buf;
            for (size_t g = 0; g < mt.games.size(); g++) {
                if (g) j += ',';
                std::snprintf(buf, sizeof buf, "[%d,%d]", mt.games[g].a,
                              mt.games[g].b);
                j += buf;
            }
            j += "]}";
        }
        return j + "]}";
    }

    bool add_match(const std::string &a_raw, const std::string &b_raw,
                   const std::string &games_raw, std::string &err) {
        std::string an = sanitize_name(a_raw), bn = sanitize_name(b_raw);
        if (an.empty() || bn.empty()) { err = "both team names are required"; return false; }
        if (an == bn) { err = "a team cannot play itself"; return false; }
        std::vector<Game> gs;
        if (!parse_games(games_raw, gs, err)) return false;
        int wa, wb;
        if (!bo3_ok(gs, wa, wb, err)) return false;
        std::lock_guard<std::mutex> lk(m);
        // all validation happens before find_or_add: a rejected add must not
        // leave new teams on the roster
        int ai = -1, bi = -1;
        for (size_t i = 0; i < teams.size(); i++) {
            if (teams[i] == an) ai = (int)i;
            if (teams[i] == bn) bi = (int)i;
        }
        if (ai >= 0 && bi >= 0) {
            int prev = find_match_locked(ai, bi);
            if (prev >= 0) {
                const Match &p = matches[prev];
                err = teams[p.a] + " vs " + teams[p.b] + " was already played (" +
                      (p.games.empty() ? "no game scores" : games_str(p)) +
                      ", " + teams[p.winner] + " won) — each pair plays at " +
                      "most once; undo it first if it is wrong";
                return false;
            }
        }
        if (ai < 0) ai = find_or_add_locked(an);
        if (bi < 0) bi = find_or_add_locked(bn);
        if (ai < 0 || bi < 0) {
            err = "too many teams (max " + std::to_string(kMaxTeams) + ")";
            return false;
        }
        matches.push_back({ai, bi, gs, wa > wb ? ai : bi}); // hi==2 => no tie
        cache_bias.clear();
        cache_plain.clear();
        return save_locked(err);
    }

    bool rename_team(const std::string &old_raw, const std::string &new_raw,
                     std::string &err) {
        std::string oldn = sanitize_name(old_raw), newn = sanitize_name(new_raw);
        if (oldn.empty() || newn.empty()) {
            err = "team and its new name are both required";
            return false;
        }
        std::lock_guard<std::mutex> lk(m);
        int oi = -1;
        for (size_t i = 0; i < teams.size(); i++)
            if (teams[i] == oldn) { oi = (int)i; break; }
        if (oi < 0) { err = "no team named \"" + oldn + "\""; return false; }
        if (newn == oldn) return true;
        for (const auto &t : teams)
            if (t == newn) {
                err = "a team named \"" + newn + "\" already exists";
                return false;
            }
        teams[oi] = newn; // matches reference indices: scores are unaffected
        return save_locked(err);
    }

    bool undo(std::string &err) {
        std::lock_guard<std::mutex> lk(m);
        if (matches.empty()) { err = "no matches to undo"; return false; }
        matches.pop_back(); // teams stay on the roster
        cache_bias.clear();
        cache_plain.clear();
        return save_locked(err);
    }

    bool add_team(const std::string &raw, std::string &err) {
        std::string nm = sanitize_name(raw);
        if (nm.empty()) { err = "team name required"; return false; }
        std::lock_guard<std::mutex> lk(m);
        for (const auto &t : teams)
            if (t == nm) {
                err = "a team named \"" + nm + "\" already exists";
                return false;
            }
        if (teams.size() >= kMaxTeams) {
            err = "too many teams (max " + std::to_string(kMaxTeams) + ")";
            return false;
        }
        teams.push_back(nm);
        cache_bias.clear(); // roster size changes both rankings
        cache_plain.clear();
        return save_locked(err);
    }

    // Removes the team and every match it played; remaining matches are
    // reindexed to the shrunken roster.
    bool delete_team(const std::string &raw, std::string &err) {
        std::string nm = sanitize_name(raw);
        if (nm.empty()) { err = "team name required"; return false; }
        std::lock_guard<std::mutex> lk(m);
        int ti = -1;
        for (size_t i = 0; i < teams.size(); i++)
            if (teams[i] == nm) { ti = (int)i; break; }
        if (ti < 0) { err = "no team named \"" + nm + "\""; return false; }
        matches.erase(std::remove_if(matches.begin(), matches.end(),
                                     [ti](const Match &mt) {
                                         return mt.a == ti || mt.b == ti;
                                     }),
                      matches.end());
        for (auto &mt : matches) {
            if (mt.a > ti) mt.a--;
            if (mt.b > ti) mt.b--;
            if (mt.winner > ti) mt.winner--;
        }
        teams.erase(teams.begin() + ti);
        cache_bias.clear();
        cache_plain.clear();
        return save_locked(err);
    }

    // Replace the games of an existing head-to-head match (scores are read
    // from team a's perspective, like add_match).
    bool edit_match(const std::string &a_raw, const std::string &b_raw,
                    const std::string &games_raw, std::string &err) {
        std::string an = sanitize_name(a_raw), bn = sanitize_name(b_raw);
        if (an.empty() || bn.empty()) { err = "both team names are required"; return false; }
        if (an == bn) { err = "a team cannot play itself"; return false; }
        std::vector<Game> gs;
        if (!parse_games(games_raw, gs, err)) return false;
        int wa, wb;
        if (!bo3_ok(gs, wa, wb, err)) return false;
        std::lock_guard<std::mutex> lk(m);
        int ai, bi;
        int idx = find_pair_locked(an, bn, ai, bi, err);
        if (idx < 0) return false;
        matches[idx] = {ai, bi, gs, wa > wb ? ai : bi};
        cache_bias.clear();
        cache_plain.clear();
        return save_locked(err);
    }

    bool delete_match(const std::string &a_raw, const std::string &b_raw,
                      std::string &err) {
        std::string an = sanitize_name(a_raw), bn = sanitize_name(b_raw);
        if (an.empty() || bn.empty()) { err = "both team names are required"; return false; }
        std::lock_guard<std::mutex> lk(m);
        int ai, bi;
        int idx = find_pair_locked(an, bn, ai, bi, err);
        if (idx < 0) return false;
        matches.erase(matches.begin() + idx);
        cache_bias.clear();
        cache_plain.clear();
        return save_locked(err);
    }
};

// -------------------------------------------------------------- web server
// Live view + remote VAR control. One listener thread accepts connections;
// each client gets a detached handler thread. Live frames go out as
// multipart/x-mixed-replace MJPEG — the camera's own bytes, no transcoding —
// which every browser renders natively in an <img> tag.

static const char *kHtmlPage = R"HTML(<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>slowmo-cam</title>
<style>
:root{color-scheme:dark}
body{margin:0;background:#0b0e12;color:#dfe6ee;font:15px/1.45 system-ui,sans-serif;display:flex;flex-direction:column;min-height:100vh}
header{display:flex;align-items:baseline;gap:12px;padding:10px 16px}
h1{font-size:16px;margin:0;font-weight:600}
#st{margin-left:auto;font-variant-numeric:tabular-nums;color:#8b96a5;font-size:13px}
main{flex:1;display:flex;align-items:center;justify-content:center;padding:0 10px}
#wrap{position:relative;width:100%;max-width:1280px}
#cam{width:100%;display:block;border-radius:10px;background:#000;aspect-ratio:16/9;object-fit:contain}
#badge{position:absolute;top:12px;left:12px;background:#c0231d;color:#fff;font-weight:700;padding:4px 12px;border-radius:6px;letter-spacing:1px;font-size:13px;animation:p 1s infinite}
@keyframes p{50%{opacity:.5}}
footer{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;padding:14px}
button{font:inherit;font-weight:600;border:0;border-radius:10px;padding:12px 20px;cursor:pointer;background:#1c2530;color:#dfe6ee}
button:disabled{opacity:.4;cursor:default}
#save{background:#b3261e;color:#fff}
#board{max-width:720px;margin:0 auto;padding:0 14px 8px;width:100%;box-sizing:border-box}
#board h2{font-size:13px;color:#8b96a5;font-weight:600;letter-spacing:.6px;text-transform:uppercase}
#board table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
#board th,#board td{text-align:left;padding:6px 8px;border-bottom:1px solid #1c2530}
#board th{color:#5c6672;font-size:12px}
#board td:first-child,#board th:first-child{width:2em;color:#5c6672}
#board td:nth-child(n+3),#board th:nth-child(n+3){text-align:right}
#af{display:flex;gap:8px;align-items:center;margin-top:12px;flex-wrap:wrap}
#af input{font:inherit;background:#141a21;border:1px solid #1c2530;color:#dfe6ee;border-radius:8px;padding:9px 12px;min-width:8em;flex:1}
#af button{padding:9px 14px}
#serr{color:#e57373;font-size:13px;min-height:1.2em;margin-top:6px}
#algsw{display:flex;gap:8px;margin-bottom:10px}
#algsw button{padding:7px 12px;font-size:13px;background:#0b0e12;border:1px solid #1c2530}
#algsw button.on{background:#1c2530;border-color:#3b82f6;color:#fff}
#board tbody tr{cursor:pointer}
#board tbody tr:hover td,#board tr.sel td{background:#141a21}
#detail{background:#141a21;border:1px solid #1c2530;border-radius:10px;padding:12px;margin-top:10px;font-size:14px}
.dh{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}
.xbtn{background:none;padding:2px 8px}
.mrow{padding:3px 0;color:#8b96a5;display:flex;align-items:center;gap:2px}
.mrow span{flex:1}
.mrow.w{color:#81c784}
.mrow.l{color:#e57373}
.mbtn{background:none;padding:0 6px;font-size:13px;opacity:.7}
.mbtn:hover{opacity:1}
.rn{display:flex;gap:8px;margin-top:10px;flex-wrap:wrap}
.rn input{font:inherit;background:#0b0e12;border:1px solid #1c2530;color:#dfe6ee;border-radius:8px;padding:7px 10px;flex:1;min-width:8em}
.danger{background:#5c1f1c;color:#fbb}
#tf{display:flex;gap:8px;margin-top:8px}
#tf input{font:inherit;background:#141a21;border:1px solid #1c2530;color:#dfe6ee;border-radius:8px;padding:9px 12px;flex:1;min-width:8em}
#tf button{padding:9px 14px}
#board h3{font-size:13px;color:#8b96a5;font-weight:600;letter-spacing:.6px;text-transform:uppercase;margin:20px 0 6px}
#vswrap{overflow-x:auto}
#board table.vs{width:auto;font-size:13px}
#board table.vs th,#board table.vs td{text-align:center;padding:4px 8px;border:1px solid #1c2530;min-width:26px;width:auto;color:#dfe6ee}
#board table.vs thead th{color:#8b96a5;font-size:12px}
#board table.vs tbody th{color:#aeb9c7;font-size:12px;text-align:right;white-space:nowrap}
#board table.vs td.win{color:#81c784;font-weight:700}
#board table.vs td.z{color:#3b4652}
#board table.vs td.np{color:#2c3540}
#board table.vs td.diag{background:#0f151c;color:#39434f}
#board table.vs .tot{color:#fff;font-weight:700}
#board table.vs tbody tr{cursor:default}
#board table.vs tbody tr:hover td{background:none}
kbd{background:#1c2530;border-radius:4px;padding:1px 5px;font-size:12px}
#help{text-align:center;color:#5c6672;font-size:12px;padding-bottom:12px}
</style>
<header><h1>slowmo-cam</h1><span id="st">connecting&hellip;</span></header>
<main><div id="wrap"><img id="cam" src="/stream" alt="live"><div id="badge" hidden>REPLAY</div></div></main>
<footer>
<button id="save">&#128308; Save + replay</button>
<button id="again" disabled>&#8635; Replay last</button>
<button id="live">&#9679; Live</button>
</footer>
<section id="board">
<h2>Tournament</h2>
<div id="algsw">
<button id="algb" class="on">Bias PageRank</button>
<button id="algp">Classic PageRank</button>
</div>
<table><thead><tr><th>#</th><th>team</th><th>score</th><th>W</th><th>L</th></tr></thead><tbody id="tb"></tbody></table>
<div id="detail" hidden></div>
<form id="af">
<input id="ta" list="tlist" placeholder="team A" maxlength="40" required autocomplete="off">
<span>vs</span>
<input id="tb2" list="tlist" placeholder="team B" maxlength="40" required autocomplete="off">
<input id="tg" placeholder="10-1, 5-10, 10-9" required autocomplete="off">
<datalist id="tlist"></datalist>
<button type="submit">Add match</button>
<button type="button" id="undo">Undo</button>
</form>
<form id="tf">
<input id="tn" placeholder="new team name" maxlength="40" required autocomplete="off">
<button type="submit">Add team</button>
</form>
<div id="serr"></div>
<h3>Head-to-head</h3>
<div id="vswrap"></div>
</section>
<div id="help"><kbd>space</kbd> save&nbsp; <kbd>r</kbd> replay&nbsp; <kbd>l</kbd>/<kbd>esc</kbd>/click live</div>
<script>
const $=id=>document.getElementById(id),cam=$('cam'),badge=$('badge'),st=$('st');
let timer=0,replayFrames=0,playFps=30,slow=4;
function live(){clearTimeout(timer);badge.hidden=true;cam.src='/stream?t='+Date.now();}
function replay(sec){clearTimeout(timer);badge.textContent='REPLAY '+slow+'× SLOW-MO';badge.hidden=false;
  cam.src='/replay?t='+Date.now();if(sec>0)timer=setTimeout(live,sec*1000+400);}
$('save').onclick=async()=>{$('save').disabled=true;
  try{const j=await(await fetch('/save',{method:'POST'})).json();if(j.ok)replay(j.playback_seconds);}
  catch(e){}finally{$('save').disabled=false;}};
$('again').onclick=()=>{if(replayFrames)replay(replayFrames/playFps);};
$('live').onclick=live;cam.onclick=live;
addEventListener('keydown',e=>{if(e.target&&e.target.tagName==='INPUT')return;
  if(e.code==='Space'){e.preventDefault();$('save').click();}
  else if(e.key==='r')$('again').click();else if(e.key==='l'||e.key==='Escape')live();});
cam.onerror=()=>setTimeout(()=>{if(badge.hidden)live();},1500);
(async function poll(){try{const s=await(await fetch('/status')).json();
  replayFrames=s.replay_frames;playFps=s.playback_fps;slow=Math.round(s.capture_fps/s.playback_fps);
  $('again').disabled=!replayFrames;
  st.textContent=s.fps.toFixed(0)+' fps · buffer '+s.buffer_pct+'% · drops '+s.drops+(s.save_busy?' · saving…':'');
}catch(e){st.textContent='⚠ connection lost';}setTimeout(poll,1000);})();
let S=null,alg='bias',selTeam=-1;
const enc=encodeURIComponent;
async function post(u){try{const r=await(await fetch(u,{method:'POST'})).json();
 if(!r.ok){$('serr').textContent=r.error;return false;}
 $('serr').textContent='';scores();return true;}catch(e){return false;}}
function fmtGames(m,fromA){return m.games.map(g=>fromA?g[0]+'-'+g[1]:g[1]+'-'+g[0]).join(', ');}
function renderDetail(){const d=$('detail');if(selTeam<0||!S){d.hidden=true;return;}
 d.hidden=false;d.innerHTML='';
 const me=S.teams[selTeam].name;
 const h=document.createElement('div');h.className='dh';
 const nm=document.createElement('b');nm.textContent=me;h.appendChild(nm);
 const x=document.createElement('button');x.textContent='✕';x.className='xbtn';
 x.onclick=()=>{selTeam=-1;renderDetail();};h.appendChild(x);d.appendChild(h);
 const ms=S.matches.filter(m=>m.a===selTeam||m.b===selTeam);
 if(!ms.length){const p=document.createElement('div');p.className='mrow';
  p.textContent='no matches yet';d.appendChild(p);}
 ms.forEach(m=>{const isA=m.a===selTeam,opp=S.teams[isA?m.b:m.a].name,won=m.winner===selTeam;
  const row=document.createElement('div');row.className='mrow '+(won?'w':'l');
  const txt=document.createElement('span');
  txt.textContent='vs '+opp+': '
   +(m.games.length?fmtGames(m,isA):'(no game scores)')
   +' → '+(won?'won':'lost');
  row.appendChild(txt);
  const eb=document.createElement('button');eb.textContent='✎';eb.className='mbtn';
  eb.title='edit game scores';
  eb.onclick=()=>{const nv=prompt('Game scores '+me+' vs '+opp
   +" (from "+me+"'s view):",m.games.length?fmtGames(m,isA):'');
   if(nv!=null)post('/scores/match/edit?a='+enc(me)+'&b='+enc(opp)+'&games='+enc(nv));};
  const db=document.createElement('button');db.textContent='🗑';db.className='mbtn';
  db.title='delete this match';
  db.onclick=()=>{if(confirm('Delete the match '+me+' vs '+opp+'?'))
   post('/scores/match/delete?a='+enc(me)+'&b='+enc(opp));};
  row.appendChild(eb);row.appendChild(db);
  d.appendChild(row);});
 const rf=document.createElement('form');rf.className='rn';
 const ri=document.createElement('input');ri.maxLength=40;ri.placeholder='rename to…';
 rf.appendChild(ri);
 const rb=document.createElement('button');rb.textContent='Rename';rf.appendChild(rb);
 rf.onsubmit=e=>{e.preventDefault();
  post('/scores/rename?team='+enc(me)+'&name='+enc(ri.value));};
 const td=document.createElement('button');td.type='button';td.textContent='Delete team';
 td.className='danger';
 td.onclick=()=>{if(confirm('Delete '+me
   +(ms.length?' and its '+ms.length+' match(es)?':'?')))
  {selTeam=-1;post('/scores/team/delete?name='+enc(me));}};
 rf.appendChild(td);
 d.appendChild(rf);}
function vcell(cls,txt){const td=document.createElement('td');if(cls)td.className=cls;td.textContent=txt;return td;}
function renderVs(idx){const wrap=$('vswrap');wrap.innerHTML='';
 if(!S||!S.teams.length)return;
 const beat={},played={};
 S.matches.forEach(m=>{const l=m.winner===m.a?m.b:m.a;beat[m.winner+'_'+l]=1;
  played[m.a+'_'+m.b]=1;played[m.b+'_'+m.a]=1;});
 const tbl=document.createElement('table');tbl.className='vs';
 const thr=document.createElement('tr');
 const c0=document.createElement('th');c0.textContent='beat \\ lost';thr.appendChild(c0);
 idx.forEach(j=>{const th=document.createElement('th');th.textContent=S.teams[j].name;thr.appendChild(th);});
 const wc=document.createElement('th');wc.className='tot';wc.textContent='W';thr.appendChild(wc);
 const thead=document.createElement('thead');thead.appendChild(thr);tbl.appendChild(thead);
 const tbody=document.createElement('tbody');
 idx.forEach(i=>{const tr=document.createElement('tr');
  const rh=document.createElement('th');rh.textContent=S.teams[i].name;tr.appendChild(rh);
  let w=0;
  idx.forEach(j=>{
   if(i===j){tr.appendChild(vcell('diag','·'));}
   else if(beat[i+'_'+j]){tr.appendChild(vcell('win','1'));w++;}
   else if(played[i+'_'+j]){tr.appendChild(vcell('z','0'));}
   else{tr.appendChild(vcell('np','–'));}});
  tr.appendChild(vcell('tot',w));tbody.appendChild(tr);});
 tbl.appendChild(tbody);wrap.appendChild(tbl);}
function renderTable(){if(!S)return;
 const tb=$('tb'),dl=$('tlist');tb.innerHTML='';dl.innerHTML='';
 const idx=S.teams.map((t,i)=>i).sort((x,y)=>S.teams[y][alg]-S.teams[x][alg]
  ||S.teams[x].name.localeCompare(S.teams[y].name));
 idx.forEach((ti,r)=>{const t=S.teams[ti],tr=document.createElement('tr');
  [r+1,t.name,(t[alg]*100).toFixed(2),t.wins,t.losses].forEach(v=>{
   const td=document.createElement('td');td.textContent=v;tr.appendChild(td);});
  tr.onclick=()=>{selTeam=ti;renderDetail();renderTable();};
  if(ti===selTeam)tr.className='sel';
  tb.appendChild(tr);
  const o=document.createElement('option');o.value=t.name;dl.appendChild(o);});
 $('algb').className=alg==='bias'?'on':'';$('algp').className=alg==='plain'?'on':'';renderVs(idx);}
async function scores(){try{const s=await(await fetch('/scores')).json();
 if(!s.ok){$('serr').textContent=s.error;return;}
 $('serr').textContent='';S=s;if(selTeam>=S.teams.length)selTeam=-1;
 renderTable();renderDetail();}catch(e){}}
$('algb').onclick=()=>{alg='bias';renderTable();};
$('algp').onclick=()=>{alg='plain';renderTable();};
$('af').addEventListener('submit',async e=>{e.preventDefault();
 try{const r=await(await fetch('/scores/add?a='+encodeURIComponent($('ta').value)
  +'&b='+encodeURIComponent($('tb2').value)
  +'&games='+encodeURIComponent($('tg').value),{method:'POST'})).json();
 if(!r.ok){$('serr').textContent=r.error;return;}
 $('ta').value='';$('tb2').value='';$('tg').value='';scores();}catch(e){}});
$('undo').onclick=()=>post('/scores/undo');
$('tf').addEventListener('submit',async e=>{e.preventDefault();
 if(await post('/scores/team/add?name='+enc($('tn').value)))$('tn').value='';});
scores();setInterval(scores,10000);
</script>
)HTML";

// Registry of open client sockets so shutdown can unblock and wait for them.
struct HttpState {
    int listen_fd = -1;
    std::mutex m;
    std::vector<int> fds;
    std::atomic<int> active{0};

    void client_add(int fd) {
        std::lock_guard<std::mutex> lk(m);
        fds.push_back(fd);
        active++;
    }
    void client_done(int fd) { // remove from registry BEFORE close(fd)
        {
            std::lock_guard<std::mutex> lk(m);
            for (size_t i = 0; i < fds.size(); i++)
                if (fds[i] == fd) { fds[i] = fds.back(); fds.pop_back(); break; }
        }
        active--;
    }
    void shutdown_clients() {
        {
            std::lock_guard<std::mutex> lk(m);
            for (int fd : fds) shutdown(fd, SHUT_RDWR);
        }
        for (int waited = 0; active.load() > 0 && waited < 3000; waited += 20)
            usleep(20 * 1000);
    }
};

static bool send_all(int fd, const void *data, size_t len, Shared *sh) {
    const uint8_t *p = (const uint8_t *)data;
    while (len) {
        if (!sh->alive || g_stop) return false;
        ssize_t w = send(fd, p, len, MSG_NOSIGNAL);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false; // client gone, or SO_SNDTIMEO hit (stalled client)
        }
        p += w;
        len -= (size_t)w;
    }
    return true;
}

static bool send_str(int fd, const std::string &s, Shared *sh) {
    return send_all(fd, s.data(), s.size(), sh);
}

static void send_simple(int fd, Shared *sh, const char *status,
                        const char *ctype, const std::string &body) {
    char h[256];
    std::snprintf(h, sizeof h,
                  "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
                  "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                  status, ctype, body.size());
    if (send_str(fd, h, sh)) send_str(fd, body, sh);
}

// Read the request head; returns "METHOD /path" split. Body (none expected
// beyond an empty POST) is ignored.
static bool read_request(int fd, std::string &method, std::string &path) {
    char buf[4096];
    size_t used = 0;
    while (used < sizeof buf - 1) {
        ssize_t r = recv(fd, buf + used, sizeof buf - 1 - used, 0);
        if (r <= 0) return false; // closed or SO_RCVTIMEO
        used += (size_t)r;
        buf[used] = 0;
        if (strstr(buf, "\r\n\r\n")) break;
    }
    char m[8] = {0}, p[2048] = {0};
    if (std::sscanf(buf, "%7s %2047s", m, p) != 2) return false;
    method = m;
    path = p;
    return true;
}

static int query_int(const std::string &path, const char *key, int fallback) {
    size_t q = path.find('?');
    if (q == std::string::npos) return fallback;
    std::string qs = path.substr(q + 1);
    size_t pos = 0;
    std::string want = std::string(key) + "=";
    while (pos < qs.size()) {
        size_t amp = qs.find('&', pos);
        std::string kv = qs.substr(pos, amp == std::string::npos ? amp : amp - pos);
        if (kv.compare(0, want.size(), want) == 0) {
            int v = atoi(kv.c_str() + want.size());
            return v > 0 ? v : fallback;
        }
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    return fallback;
}

static std::string percent_decode(const std::string &s) {
    std::string out;
    for (size_t i = 0; i < s.size(); i++) {
        if (s[i] == '+') out += ' ';
        else if (s[i] == '%' && i + 2 < s.size() && isxdigit((unsigned char)s[i + 1]) &&
                 isxdigit((unsigned char)s[i + 2])) {
            char hex[3] = {s[i + 1], s[i + 2], 0};
            out += (char)strtol(hex, nullptr, 16);
            i += 2;
        } else out += s[i];
    }
    return out;
}

// value of ?key=... (percent-decoded), or "" if absent
static std::string query_str(const std::string &path, const char *key) {
    size_t q = path.find('?');
    if (q == std::string::npos) return "";
    std::string qs = path.substr(q + 1);
    std::string want = std::string(key) + "=";
    size_t pos = 0;
    while (pos < qs.size()) {
        size_t amp = qs.find('&', pos);
        std::string kv = qs.substr(pos, amp == std::string::npos ? std::string::npos
                                                                 : amp - pos);
        if (kv.compare(0, want.size(), want) == 0)
            return percent_decode(kv.substr(want.size()));
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    return "";
}

static const char *kMjpegHeader =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Cache-Control: no-store\r\nPragma: no-cache\r\nConnection: close\r\n\r\n";

static bool send_part(int fd, const FrameData &f, Shared *sh) {
    char h[96];
    int n = std::snprintf(h, sizeof h,
                          "--frame\r\nContent-Type: image/jpeg\r\n"
                          "Content-Length: %zu\r\n\r\n", f.size());
    return send_all(fd, h, (size_t)n, sh) &&
           send_all(fd, f.data(), f.size(), sh) &&
           send_all(fd, "\r\n", 2, sh);
}

// Live camera -> browser, paced at `fps` (the ring keeps filling at 120).
static void handle_stream(int fd, const Cfg &cfg, Shared *sh, int fps) {
    if (fps > cfg.capture_fps) fps = cfg.capture_fps;
    if (!send_str(fd, kMjpegHeader, sh)) return;
    const double period = 1.0 / fps;
    double next = now_mono();
    while (sh->alive && !g_stop) {
        double t = now_mono();
        if (t < next) {
            int ms = (int)((next - t) * 1000.0);
            poll(nullptr, 0, ms > 100 ? 100 : (ms < 1 ? 1 : ms));
            continue;
        }
        Frame f = sh->ring.latest();
        if (!f) { next = t + 0.1; continue; }
        if (!send_part(fd, *f, sh)) return;
        next += period;
        if (next < t - 1.0) next = t; // stalled client came back: no burst
    }
}

// Last saved clip from RAM at playback fps, looping until the client leaves
// (the control page switches itself back to /stream after one pass).
static void handle_replay(int fd, const Cfg &cfg, Shared *sh) {
    auto snap = sh->get_last_snap();
    if (!snap || snap->empty()) {
        send_simple(fd, sh, "404 Not Found", "application/json",
                    "{\"ok\":false,\"error\":\"nothing saved yet\"}");
        return;
    }
    if (!send_str(fd, kMjpegHeader, sh)) return;
    const double period = 1.0 / cfg.playback_fps;
    double next = now_mono();
    size_t i = 0;
    while (sh->alive && !g_stop) {
        double t = now_mono();
        if (t < next) {
            int ms = (int)((next - t) * 1000.0);
            poll(nullptr, 0, ms > 100 ? 100 : (ms < 1 ? 1 : ms));
            continue;
        }
        if (!send_part(fd, *(*snap)[i], sh)) return;
        i = (i + 1) % snap->size();
        next += period;
        if (next < t - 1.0) next = t;
    }
}

static void handle_save(int fd, const Cfg &cfg, Shared *sh) {
    std::string path;
    auto snap = save_now(cfg, *sh, &path);
    if (!snap) {
        send_simple(fd, sh, "503 Service Unavailable", "application/json",
                    "{\"ok\":false,\"error\":\"buffer empty\"}");
        return;
    }
    char body[512];
    std::snprintf(body, sizeof body,
                  "{\"ok\":true,\"frames\":%zu,\"playback_seconds\":%.2f,"
                  "\"file\":\"%s\"}",
                  snap->size(), (double)snap->size() / cfg.playback_fps,
                  basename_of(path).c_str());
    send_simple(fd, sh, "200 OK", "application/json", body);
}

static void handle_status(int fd, const Cfg &cfg, Shared *sh) {
    size_t fill = sh->ring.size() * 100 / cfg.max_frames();
    if (fill > 100) fill = 100;
    auto snap = sh->get_last_snap();
    char body[512];
    std::snprintf(body, sizeof body,
                  "{\"fps\":%.1f,\"buffer_pct\":%zu,\"frames\":%zu,"
                  "\"drops\":%llu,\"save_busy\":%s,\"last_clip\":\"%s\","
                  "\"replay_frames\":%zu,\"capture_fps\":%d,"
                  "\"playback_fps\":%d,\"buffer_seconds\":%.1f}",
                  sh->ring.measured_fps(), fill, sh->ring.size(),
                  (unsigned long long)sh->drops.load(),
                  sh->save_busy ? "true" : "false",
                  basename_of(sh->get_last_path()).c_str(),
                  snap ? snap->size() : 0, cfg.capture_fps, cfg.playback_fps,
                  cfg.buffer_seconds);
    send_simple(fd, sh, "200 OK", "application/json", body);
}

static void scores_error(int fd, Shared *sh, const char *status,
                         const std::string &err) {
    std::string body = "{\"ok\":false,\"error\":\"";
    json_escape(body, err);
    send_simple(fd, sh, status, "application/json", body + "\"}");
}

static void handle_scores(int fd, Shared *sh) {
    if (!sh->scores) {
        scores_error(fd, sh, "503 Service Unavailable", "scoreboard disabled");
        return;
    }
    send_simple(fd, sh, "200 OK", "application/json", sh->scores->scores_json());
}

static void handle_scores_add(int fd, Shared *sh, const std::string &path) {
    if (!sh->scores) {
        scores_error(fd, sh, "503 Service Unavailable", "scoreboard disabled");
        return;
    }
    std::string err;
    if (!sh->scores->add_match(query_str(path, "a"), query_str(path, "b"),
                               query_str(path, "games"), err)) {
        scores_error(fd, sh, "400 Bad Request", err);
        return;
    }
    send_simple(fd, sh, "200 OK", "application/json", "{\"ok\":true}");
}

static void handle_scores_rename(int fd, Shared *sh, const std::string &path) {
    if (!sh->scores) {
        scores_error(fd, sh, "503 Service Unavailable", "scoreboard disabled");
        return;
    }
    std::string err;
    if (!sh->scores->rename_team(query_str(path, "team"), query_str(path, "name"), err)) {
        scores_error(fd, sh, "400 Bad Request", err);
        return;
    }
    send_simple(fd, sh, "200 OK", "application/json", "{\"ok\":true}");
}

static void handle_scores_undo(int fd, Shared *sh) {
    if (!sh->scores) {
        scores_error(fd, sh, "503 Service Unavailable", "scoreboard disabled");
        return;
    }
    std::string err;
    if (!sh->scores->undo(err)) {
        scores_error(fd, sh, "400 Bad Request", err);
        return;
    }
    send_simple(fd, sh, "200 OK", "application/json", "{\"ok\":true}");
}

// shared tail for the team/match management endpoints
static void scores_mutation(int fd, Shared *sh, bool ok, const std::string &err) {
    if (!ok) {
        scores_error(fd, sh, "400 Bad Request", err);
        return;
    }
    send_simple(fd, sh, "200 OK", "application/json", "{\"ok\":true}");
}

static void http_client_thread(int fd, Cfg cfg, Shared *sh, HttpState *st) {
    timeval tv{5, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);

    std::string method, path;
    if (read_request(fd, method, path)) {
        std::string route = path.substr(0, path.find('?'));
        if (method == "GET" && (route == "/" || route == "/index.html"))
            send_simple(fd, sh, "200 OK", "text/html; charset=utf-8", kHtmlPage);
        else if (method == "GET" && route == "/stream")
            handle_stream(fd, cfg, sh, query_int(path, "fps", cfg.http_fps));
        else if (method == "GET" && route == "/replay")
            handle_replay(fd, cfg, sh);
        else if (method == "POST" && route == "/save")
            handle_save(fd, cfg, sh);
        else if (method == "GET" && route == "/status")
            handle_status(fd, cfg, sh);
        else if (method == "GET" && route == "/scores")
            handle_scores(fd, sh);
        else if (method == "POST" && route == "/scores/add")
            handle_scores_add(fd, sh, path);
        else if (method == "POST" && route == "/scores/rename")
            handle_scores_rename(fd, sh, path);
        else if (method == "POST" && route == "/scores/undo")
            handle_scores_undo(fd, sh);
        else if (method == "POST" && route == "/scores/team/add" && sh->scores) {
            std::string err;
            scores_mutation(fd, sh, sh->scores->add_team(query_str(path, "name"), err), err);
        } else if (method == "POST" && route == "/scores/team/delete" && sh->scores) {
            std::string err;
            scores_mutation(fd, sh, sh->scores->delete_team(query_str(path, "name"), err), err);
        } else if (method == "POST" && route == "/scores/match/edit" && sh->scores) {
            std::string err;
            scores_mutation(fd, sh,
                            sh->scores->edit_match(query_str(path, "a"), query_str(path, "b"),
                                                   query_str(path, "games"), err),
                            err);
        } else if (method == "POST" && route == "/scores/match/delete" && sh->scores) {
            std::string err;
            scores_mutation(fd, sh,
                            sh->scores->delete_match(query_str(path, "a"),
                                                     query_str(path, "b"), err),
                            err);
        }
        else if (method == "GET" && route == "/favicon.ico")
            send_str(fd, "HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n", sh);
        else
            send_simple(fd, sh, "404 Not Found", "text/plain", "not found\n");
    }
    st->client_done(fd); // deregister first, then close (avoids fd-reuse race)
    close(fd);
}

// First non-loopback IPv4, for the startup hint. Best effort.
static std::string lan_ip() {
    ifaddrs *ifa = nullptr;
    std::string ip;
    if (getifaddrs(&ifa) == 0) {
        for (ifaddrs *a = ifa; a; a = a->ifa_next) {
            if (!a->ifa_addr || a->ifa_addr->sa_family != AF_INET) continue;
            char buf[INET_ADDRSTRLEN];
            auto *sin = (sockaddr_in *)a->ifa_addr;
            if (!inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof buf)) continue;
            if (std::strcmp(buf, "127.0.0.1") == 0) continue;
            ip = buf;
            break;
        }
        freeifaddrs(ifa);
    }
    return ip;
}

static void http_server_thread(Cfg cfg, Shared *sh, HttpState *st) {
    int lfd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (lfd < 0) { sh->msgs.push(errno_str("web: socket")); return; }
    int one = 1;
    setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)cfg.http_port);
    if (bind(lfd, (sockaddr *)&addr, sizeof addr) != 0 || listen(lfd, 16) != 0) {
        char m[160];
        std::snprintf(m, sizeof m, "web: cannot listen on port %d (%s) — "
                      "live view disabled", cfg.http_port, std::strerror(errno));
        sh->msgs.push(m);
        close(lfd);
        return;
    }
    st->listen_fd = lfd;
    {
        std::string ip = lan_ip();
        char m[256];
        std::snprintf(m, sizeof m, "live view: http://%s:%d  (via ssh: "
                      "ssh -L %d:localhost:%d <pi>, then http://localhost:%d)",
                      ip.empty() ? "<pi-ip>" : ip.c_str(), cfg.http_port,
                      cfg.http_port, cfg.http_port, cfg.http_port);
        sh->msgs.push(m);
    }

    while (sh->alive && !g_stop) {
        pollfd p{lfd, POLLIN, 0};
        int r = poll(&p, 1, 200);
        if (r < 0 && errno != EINTR) break;
        if (r <= 0 || !(p.revents & POLLIN)) continue;
        int cfd = accept4(lfd, nullptr, nullptr, SOCK_CLOEXEC);
        if (cfd < 0) continue;
        st->client_add(cfd);
        std::thread(http_client_thread, cfd, cfg, sh, st).detach();
    }
    st->shutdown_clients();
    close(lfd);
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

static void run_replay(const Snapshot &snap, const Cfg &cfg, Shared &sh,
                       pid_t &file_player) {
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
        sh.join_saver(); // make sure the file exists
        std::string p = sh.get_last_path();
        if (!p.empty()) {
            if (file_player > 0) { kill(file_player, SIGTERM); reap(file_player, 200); }
            file_player = replay_file(p);
            if (file_player < 0)
                sh.msgs.push("could not start ffplay — install with: sudo apt install ffmpeg");
        }
    }
}

static int interactive(const Cfg &cfg, Shared &sh) {
    sh.interactive = true;
    RawTerm raw;
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
            auto snap = save_now(cfg, sh);
            if (snap) {
                print_msgs(sh);
                if (cfg.autoreplay)
                    run_replay(*snap, cfg, sh, file_player);
            }
        } else if (key == 'r' || key == 'R') {
            auto snap = sh.get_last_snap();
            if (snap)
                run_replay(*snap, cfg, sh, file_player);
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

// ---------------------------------------------------------------- headless
// No tty on stdin (nohup, systemd, ssh without -t): keep recording and let
// the web page do all the control. Runs until SIGINT/SIGTERM.

static int headless(const Cfg &cfg, Shared &sh) {
    std::printf("no terminal — headless mode: recording %dx%d @ %d fps, "
                "control via the web page (Ctrl-C / SIGTERM to quit)\n",
                cfg.width, cfg.height, cfg.capture_fps);
    while (!g_stop) {
        std::string err = sh.get_error();
        if (!err.empty()) {
            std::fprintf(stderr, "capture error: %s\n", err.c_str());
            return 1;
        }
        for (const auto &m : sh.msgs.drain()) {
            std::printf("  %s\n", m.c_str());
            std::fflush(stdout);
        }
        poll(nullptr, 0, 200);
    }
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

    ScoreBoard scores; // declared before sh: outlives the web client threads
    Shared sh(cfg.max_frames(), (size_t)cfg.capture_fps);
    if (!cfg.no_scores && cfg.http_port > 0 && cfg.selftest == 0) {
        if (access(cfg.scores_script.c_str(), F_OK) != 0) {
            std::fprintf(stderr, "warning: score script not found (%s) — "
                                 "scoreboard disabled (--scores-script PATH)\n",
                         cfg.scores_script.c_str());
        } else {
            scores.script = cfg.scores_script;
            scores.file = cfg.scores_file;
            scores.load_or_seed(sh.msgs);
            sh.scores = &scores;
        }
    }
    std::thread cap(cfg.mjpeg_file.empty() ? v4l2_capture_thread
                                           : file_capture_thread,
                    cfg, &sh);
    HttpState http_state;
    std::thread http;
    if (cfg.http_port > 0 && cfg.selftest == 0)
        http = std::thread(http_server_thread, cfg, &sh, &http_state);

    int rc;
    if (cfg.selftest > 0)
        rc = selftest(cfg, sh);
    else if (isatty(0))
        rc = interactive(cfg, sh);
    else if (http.joinable())
        rc = headless(cfg, sh);
    else {
        std::fprintf(stderr, "stdin is not a terminal and the web port is "
                             "disabled — use --selftest N or --port N\n");
        rc = 1;
    }

    sh.alive = false;
    g_stop = 1;
    cap.join();
    if (http.joinable()) http.join(); // unblocks + waits for web clients
    if (sh.save_busy) std::printf("finishing save ...\n");
    sh.join_saver();
    for (const auto &m : sh.msgs.drain()) std::printf("  %s\n", m.c_str());
    return rc;
}
