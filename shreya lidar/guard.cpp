#include "config.h"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

enum Level { LEVEL_OK = 0, LEVEL_L1 = 1, LEVEL_L2 = 2, LEVEL_FAULT = 3 };

static const char* level_name(Level level) {
    switch (level) {
    case LEVEL_L1: return "L1";
    case LEVEL_L2: return "L2";
    case LEVEL_FAULT: return "FAULT";
    default: return "OK";
    }
}

struct Point {
    float angle_deg;
    int dist_mm;
    int quality;
    bool sync;
};

struct Parser {
    int pos = 0;
    uint8_t raw[5]{};

    bool feed(uint8_t b, Point* out) {
        for (;;) {
            if (pos == 0) {
                if ((((b >> 1) ^ b) & 0x01) == 0) return false;
                raw[0] = b;
                pos = 1;
                return false;
            }
            if (pos == 1 && (b & 0x01) == 0) {
                pos = 0;
                continue;
            }
            raw[pos++] = b;
            if (pos < 5) return false;
            pos = 0;
            uint16_t angle_q = (uint16_t)(raw[1] | (raw[2] << 8));
            uint16_t dist_q = (uint16_t)(raw[3] | (raw[4] << 8));
            out->sync = (raw[0] & 0x01) != 0;
            out->quality = raw[0] >> 2;
            out->angle_deg = (float)(angle_q >> 1) / 64.f;
            out->dist_mm = (int)(dist_q / 4);
            return true;
        }
    }
};

static int bin_of(float angle_deg, int offset) {
    int d = (int)std::lround(angle_deg) + offset;
    d %= LIDAR_BINS;
    if (d < 0) d += LIDAR_BINS;
    return d;
}

static bool bin_watched(int bin, char dir) {
    if (dir != 'F' && dir != 'R') return true;
    bool front = bin <= 90 || bin >= 270;
    return dir == 'F' ? front : !front;
}

static Level evaluate(const Config* cfg, const uint16_t dist[LIDAR_BINS], char dir) {
    uint8_t hit[LIDAR_BINS];
    for (int i = 0; i < LIDAR_BINS; i++) {
        hit[i] = 0;
        if (cfg->blank[i] || dist[i] == 0 || !bin_watched(i, dir)) continue;
        if (cfg->ignore_inner_mm > 0 && (int)dist[i] <= cfg->ignore_inner_mm) continue;
        uint16_t body = cfg->chassis_mm[i];
        if (body && (int)dist[i] + cfg->chassis_margin_mm >= (int)body) continue;
        if ((int)dist[i] <= cfg->l2_mm) hit[i] = 2;
        else if ((int)dist[i] <= cfg->l1_mm) hit[i] = 1;
    }

    int run2 = 0;
    int run1 = 0;
    bool saw2 = false;
    bool saw1 = false;
    auto feed = [&](uint8_t h) {
        if (h == 2) {
            run2++;
            run1++;
        } else if (h == 1) {
            run2 = 0;
            run1++;
        } else {
            run2 = 0;
            run1 = 0;
        }
        if (run2 >= cfg->min_adjacent) saw2 = true;
        if (run1 >= cfg->min_adjacent) saw1 = true;
    };
    for (int i = 0; i < LIDAR_BINS; i++) feed(hit[i]);
    int extra = cfg->min_adjacent - 1;
    if (extra > LIDAR_BINS) extra = LIDAR_BINS;
    for (int i = 0; i < extra; i++) feed(hit[i]);
    if (saw2) return LEVEL_L2;
    if (saw1) return LEVEL_L1;
    return LEVEL_OK;
}

struct Debounce {
    int l2 = 0;
    int l1 = 0;
    int ok = 0;
    Level published = LEVEL_OK;

    void push(Level raw, const Config* cfg) {
        if (raw == LEVEL_L2) {
            l2++;
            l1++;
            ok = 0;
        } else if (raw == LEVEL_L1) {
            l2 = 0;
            l1++;
            ok = 0;
        } else {
            l2 = 0;
            l1 = 0;
            ok++;
        }
        if (l2 >= cfg->confirm_scans) published = LEVEL_L2;
        else if (l1 >= cfg->confirm_scans) published = LEVEL_L1;
        else if (ok >= cfg->clear_scans) published = LEVEL_OK;
    }
};

static long long mono_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

static long long wall_ms() {
    timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

static volatile sig_atomic_t g_run = 1;
static void on_signal(int) { g_run = 0; }

static bool write_all(int fd, const uint8_t* data, size_t n) {
    size_t off = 0;
    while (off < n) {
        ssize_t w = ::write(fd, data + off, n - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        off += (size_t)w;
    }
    return true;
}

static void send_simple(int fd, uint8_t cmd) {
    uint8_t b[2] = {0xA5, cmd};
    write_all(fd, b, 2);
}

static void send_payload(int fd, uint8_t cmd, const uint8_t* payload, uint8_t n) {
    cmd = (uint8_t)(cmd | 0x80);
    uint8_t cs = (uint8_t)(0xA5 ^ cmd ^ n);
    for (uint8_t i = 0; i < n; i++) cs = (uint8_t)(cs ^ payload[i]);
    uint8_t buf[16];
    buf[0] = 0xA5;
    buf[1] = cmd;
    buf[2] = n;
    std::memcpy(buf + 3, payload, n);
    buf[3 + n] = cs;
    write_all(fd, buf, (size_t)(4 + n));
}

static bool read_bytes(int fd, uint8_t* dst, int n, int timeout_ms) {
    int got = 0;
    long long deadline = mono_ms() + timeout_ms;
    while (got < n && g_run) {
        long long now = mono_ms();
        if (now >= deadline) return false;
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int pr = poll(&pfd, 1, (int)(deadline - now));
        if (pr < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (pr == 0) return false;
        ssize_t r = ::read(fd, dst + got, (size_t)(n - got));
        if (r < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            return false;
        }
        if (r == 0) return false;
        got += (int)r;
    }
    return got == n;
}

static bool read_header(int fd, uint8_t* type, uint32_t* size, int timeout_ms) {
    uint8_t raw[7];
    int state = 0;
    long long deadline = mono_ms() + timeout_ms;
    while (g_run && mono_ms() < deadline) {
        uint8_t b = 0;
        if (!read_bytes(fd, &b, 1, (int)(deadline - mono_ms()))) return false;
        if (state == 0) {
            if (b != 0xA5) continue;
            raw[0] = b;
            state = 1;
            continue;
        }
        if (state == 1) {
            if (b != 0x5A) {
                state = (b == 0xA5) ? 1 : 0;
                raw[0] = b;
                continue;
            }
            raw[1] = b;
            state = 2;
            continue;
        }
        raw[state++] = b;
        if (state == 7) {
            uint32_t word = (uint32_t)raw[2] | ((uint32_t)raw[3] << 8) |
                            ((uint32_t)raw[4] << 16) | ((uint32_t)raw[5] << 24);
            *size = word & 0x3FFFFFFF;
            *type = raw[6];
            return true;
        }
    }
    return false;
}

static speed_t baud_speed(int baud) {
    switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B460800;
    }
}

static void relax_latency(const char* port) {
    const char* base = std::strrchr(port, '/');
    base = base ? base + 1 : port;
    char path[256];
    std::snprintf(path, sizeof(path), "/sys/bus/usb-serial/devices/%s/latency_timer", base);
    FILE* f = std::fopen(path, "w");
    if (!f) return;
    std::fputs("1\n", f);
    std::fclose(f);
}

static int open_port(const char* port, int baud) {
    int fd = ::open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::fprintf(stderr, "cannot open %s: %s\n", port, std::strerror(errno));
        return -1;
    }
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        std::fprintf(stderr, "%s is already in use\n", port);
        ::close(fd);
        return -1;
    }
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::fprintf(stderr, "tcgetattr failed: %s\n", std::strerror(errno));
        ::close(fd);
        return -1;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, baud_speed(baud));
    cfsetospeed(&tty, baud_speed(baud));
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::fprintf(stderr, "tcsetattr failed: %s\n", std::strerror(errno));
        ::close(fd);
        return -1;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    tcflush(fd, TCIOFLUSH);
    relax_latency(port);
    return fd;
}

static char normalize_dir(int c) {
    if (c == 'F' || c == 'f') return 'F';
    if (c == 'R' || c == 'r') return 'R';
    return 'S';
}

static char read_dir() {
    FILE* f = std::fopen(LIDAR_DIR_PATH, "r");
    if (!f) return 'S';
    int c = std::fgetc(f);
    std::fclose(f);
    return normalize_dir(c);
}

static void write_dir(char dir) {
    char tmp[] = "/dev/shm/lidar-dir.tmp";
    FILE* f = std::fopen(tmp, "w");
    if (!f) return;
    std::fputc(dir, f);
    std::fputc('\n', f);
    std::fclose(f);
    std::rename(tmp, LIDAR_DIR_PATH);
}

static void write_status(Level level, const uint16_t dist[LIDAR_BINS], int hz_x10, char dir) {
    char tmp[] = "/dev/shm/lidar-status.json.tmp";
    FILE* f = std::fopen(tmp, "wb");
    if (!f) return;
    std::fprintf(f, "{\"level\":\"%s\",\"dir\":\"%c\",\"ms\":%lld,\"hz\":%d.%d,\"dist_mm\":[",
                 level_name(level), dir, wall_ms(), hz_x10 / 10, hz_x10 % 10);
    for (int i = 0; i < LIDAR_BINS; i++) {
        if (i) std::fputc(',', f);
        std::fprintf(f, "%u", dist[i]);
    }
    std::fprintf(f, "]}");
    std::fclose(f);
    std::rename(tmp, LIDAR_STATUS_PATH);
}

static int self_test() {
    int failed = 0;
    auto expect = [&](bool ok, const char* msg) {
        if (!ok) {
            std::fprintf(stderr, "FAIL %s\n", msg);
            failed++;
        }
    };

    Parser parser;
    Point pt{};
    uint8_t junk = 0x00;
    expect(!parser.feed(junk, &pt), "junk byte ignored");
    uint8_t node[] = {0x3D, 0x01, 0x2D, 0xA0, 0x0F};
    bool got = false;
    for (uint8_t b : node) got = parser.feed(b, &pt);
    expect(got, "node decoded");
    expect(pt.sync && pt.quality == 15, "sync and quality");
    expect(std::fabs(pt.angle_deg - 90.f) < 0.01f, "angle 90");
    expect(pt.dist_mm == 1000, "distance 1000");

    uint8_t node2[] = {0x2A, 0x01, 0x00, 0xE8, 0x03};
    got = false;
    for (uint8_t b : node2) got = parser.feed(b, &pt);
    expect(got && !pt.sync && pt.quality == 10, "second node flags");
    expect(std::fabs(pt.angle_deg) < 0.01f && pt.dist_mm == 250, "second node values");

    Config cfg;
    config_defaults(&cfg);
    uint16_t dist[LIDAR_BINS] = {};
    expect(evaluate(&cfg, dist, 'S') == LEVEL_OK, "empty scan is OK");
    dist[10] = dist[11] = dist[12] = 400;
    expect(evaluate(&cfg, dist, 'S') == LEVEL_L2, "three near bins are L2");
    dist[10] = dist[11] = dist[12] = 1000;
    expect(evaluate(&cfg, dist, 'S') == LEVEL_L1, "three mid bins are L1");
    dist[12] = 0;
    expect(evaluate(&cfg, dist, 'S') == LEVEL_OK, "short run is ignored");

    std::memset(dist, 0, sizeof(dist));
    dist[359] = dist[0] = dist[1] = 400;
    expect(evaluate(&cfg, dist, 'S') == LEVEL_L2, "wrap-around run is L2");

    std::memset(dist, 0, sizeof(dist));
    dist[10] = dist[11] = dist[12] = 100;
    cfg.blank[10] = cfg.blank[11] = cfg.blank[12] = true;
    expect(evaluate(&cfg, dist, 'S') == LEVEL_OK, "blank rays are ignored");

    config_defaults(&cfg);
    std::memset(dist, 0, sizeof(dist));
    cfg.chassis_mm[20] = cfg.chassis_mm[21] = cfg.chassis_mm[22] = 200;
    dist[20] = dist[21] = dist[22] = 200;
    expect(evaluate(&cfg, dist, 'S') == LEVEL_OK, "chassis contour is ignored");
    dist[20] = dist[21] = dist[22] = 100;
    expect(evaluate(&cfg, dist, 'S') == LEVEL_L2, "closer than chassis is L2");

    config_defaults(&cfg);
    std::memset(dist, 0, sizeof(dist));
    dist[0] = dist[1] = dist[2] = 400;
    expect(evaluate(&cfg, dist, 'F') == LEVEL_L2, "front obstacle warns while forward");
    expect(evaluate(&cfg, dist, 'R') == LEVEL_OK, "front obstacle is ignored in reverse");
    std::memset(dist, 0, sizeof(dist));
    dist[180] = dist[181] = dist[182] = 400;
    expect(evaluate(&cfg, dist, 'R') == LEVEL_L2, "rear obstacle warns while reversing");
    expect(evaluate(&cfg, dist, 'F') == LEVEL_OK, "rear obstacle is ignored while forward");
    expect(evaluate(&cfg, dist, 'S') == LEVEL_L2, "stopped watches both halves");

    Debounce deb;
    config_defaults(&cfg);
    deb.push(LEVEL_L2, &cfg);
    expect(deb.published == LEVEL_OK, "one scan does not trip");
    deb.push(LEVEL_L2, &cfg);
    expect(deb.published == LEVEL_L2, "two scans trip L2");
    deb.push(LEVEL_OK, &cfg);
    deb.push(LEVEL_OK, &cfg);
    expect(deb.published == LEVEL_L2, "hold L2 while clearing");
    deb.push(LEVEL_OK, &cfg);
    expect(deb.published == LEVEL_OK, "clear after three scans");

    config_defaults(&cfg);
    cfg.blank[350] = cfg.blank[351] = true;
    for (int d = 352; d <= 359; d++) cfg.blank[d] = true;
    for (int d = 0; d <= 10; d++) cfg.blank[d] = true;
    cfg.chassis_mm[4] = 180;
    cfg.angle_offset_deg = 15;
    const char* path = "/tmp/.selftest.json";
    expect(config_save(path, &cfg), "save config");
    Config loaded;
    expect(config_load(path, &loaded), "load config");
    expect(loaded.l1_mm == 1000 && loaded.l2_mm == 550, "ranges round-trip");
    expect(loaded.blank[0] && loaded.blank[359] && loaded.blank[355], "wrapped blank kept");
    expect(!loaded.blank[180], "open angle stays open");
    expect(loaded.chassis_mm[4] == 180, "chassis round-trip");
    expect(loaded.angle_offset_deg == 15, "offset round-trip");
    std::remove(path);

    if (failed) {
        std::fprintf(stderr, "%d failed\n", failed);
        return 1;
    }
    std::printf("self-test ok\n");
    return 0;
}

static Config g_cfg;
static const char* g_cfg_path = "config.json";
static time_t g_mtime_sec = -1;
static long g_mtime_nsec = -1;
static bool g_teaching = false;
static int g_teach_left = 0;
static uint16_t g_teach[20][LIDAR_BINS];
static uint8_t g_teach_n[LIDAR_BINS];

static void remember_mtime() {
    struct stat st{};
    if (stat(g_cfg_path, &st) != 0) return;
    g_mtime_sec = st.st_mtim.tv_sec;
    g_mtime_nsec = st.st_mtim.tv_nsec;
}

static void start_teach() {
    std::memset(g_teach, 0, sizeof(g_teach));
    std::memset(g_teach_n, 0, sizeof(g_teach_n));
    g_teach_left = 20;
    g_teaching = true;
    std::fprintf(stderr, "teaching chassis for 20 scans\n");
}

static void finish_teach() {
    for (int i = 0; i < LIDAR_BINS; i++) {
        int n = g_teach_n[i];
        if (n <= 0) {
            g_cfg.chassis_mm[i] = 0;
            continue;
        }
        uint16_t v[20];
        for (int k = 0; k < n; k++) v[k] = g_teach[k][i];
        for (int a = 1; a < n; a++) {
            uint16_t x = v[a];
            int b = a;
            while (b > 0 && v[b - 1] > x) {
                v[b] = v[b - 1];
                b--;
            }
            v[b] = x;
        }
        uint16_t med = v[n / 2];
        g_cfg.chassis_mm[i] = (med > 0 && med <= g_cfg.chassis_max_mm) ? med : 0;
    }
    g_cfg.teach_chassis = false;
    g_teaching = false;
    g_teach_left = 0;
    if (!config_save(g_cfg_path, &g_cfg)) std::fprintf(stderr, "could not save taught chassis\n");
    else std::fprintf(stderr, "chassis taught\n");
    remember_mtime();
}

static void maybe_reload() {
    struct stat st{};
    if (stat(g_cfg_path, &st) != 0) return;
    if (st.st_mtim.tv_sec == g_mtime_sec && st.st_mtim.tv_nsec == g_mtime_nsec) return;
    Config next;
    if (!config_load(g_cfg_path, &next)) return;
    g_cfg = next;
    g_mtime_sec = st.st_mtim.tv_sec;
    g_mtime_nsec = st.st_mtim.tv_nsec;
    std::fprintf(stderr, "config reloaded  L1 %d mm  L2 %d mm\n", g_cfg.l1_mm, g_cfg.l2_mm);
    if (g_cfg.teach_chassis) start_teach();
    else {
        g_teaching = false;
        g_teach_left = 0;
    }
}

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--self-test") == 0) return self_test();
    g_cfg_path = (argc > 1) ? argv[1] : "config.json";

    if (!config_load(g_cfg_path, &g_cfg)) {
        config_defaults(&g_cfg);
        if (!config_save(g_cfg_path, &g_cfg)) {
            std::fprintf(stderr, "cannot write %s\n", g_cfg_path);
            return 1;
        }
        std::fprintf(stderr, "wrote default %s\n", g_cfg_path);
    }
    remember_mtime();
    if (g_cfg.teach_chassis) start_teach();

    int fd = open_port(g_cfg.port, g_cfg.baud);
    if (fd < 0) return 1;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    send_simple(fd, 0x25);
    usleep(150 * 1000);
    tcflush(fd, TCIFLUSH);

    send_simple(fd, 0x52);
    uint8_t type = 0;
    uint32_t size = 0;
    if (!read_header(fd, &type, &size, 800) || type != 0x06) {
        std::fprintf(stderr, "health request failed (type 0x%02x)\n", type);
        ::close(fd);
        return 1;
    }
    uint8_t health[8] = {};
    int health_n = (int)size;
    if (health_n < 3) health_n = 3;
    if (health_n > 8) health_n = 8;
    if (!read_bytes(fd, health, health_n, 400)) {
        std::fprintf(stderr, "health payload missing\n");
        ::close(fd);
        return 1;
    }
    if (health[0] != 0) {
        unsigned err = (unsigned)health[1] | ((unsigned)health[2] << 8);
        std::fprintf(stderr, "lidar health status %u error 0x%04x\n", health[0], err);
        ::close(fd);
        return 1;
    }

    uint8_t pwm[2] = {0x94, 0x02};
    send_payload(fd, 0xF0, pwm, 2);
    usleep(300 * 1000);
    tcflush(fd, TCIFLUSH);

    send_simple(fd, 0x20);
    if (!read_header(fd, &type, &size, 1000) || type != 0x81) {
        std::fprintf(stderr, "scan start failed (type 0x%02x size %u)\n", type, size);
        send_simple(fd, 0x25);
        ::close(fd);
        return 1;
    }

    write_dir('S');
    char g_dir = 'S';
    std::fprintf(stderr, "scanning %s at %d  direction S\n", g_cfg.port, g_cfg.baud);

    Parser parser;
    uint16_t dist[LIDAR_BINS] = {};
    uint16_t published[LIDAR_BINS] = {};
    int scan_points = 0;
    int last_points = 0;
    Debounce deb;
    Level shown = LEVEL_OK;
    bool ever = false;
    bool fault = false;
    long long started = mono_ms();
    long long last_scan = started;
    long long last_log = started;
    long long last_write = 0;
    long long prev_scan = 0;
    int hz_x10 = 0;
    uint8_t buf[512];

    auto publish = [&](bool force) {
        Level level = fault ? LEVEL_FAULT : deb.published;
        long long now = mono_ms();
        if (force || now - last_write >= 300) {
            write_status(level, published, hz_x10, g_dir);
            last_write = now;
        }
        if (level != shown) {
            shown = level;
            std::printf("%s\n", level_name(level));
            std::fflush(stdout);
        }
    };

    while (g_run) {
        maybe_reload();
        char next_dir = read_dir();
        if (next_dir != g_dir) {
            g_dir = next_dir;
            deb = Debounce{};
            std::fprintf(stderr, "direction %c\n", g_dir);
        }
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int pr = poll(&pfd, 1, 100);
        if (pr < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (pr > 0) {
            ssize_t n = ::read(fd, buf, sizeof(buf));
            if (n < 0) {
                if (errno != EAGAIN && errno != EINTR) break;
            } else {
                for (ssize_t i = 0; i < n; i++) {
                    Point pt;
                    if (!parser.feed(buf[i], &pt)) continue;
                    if (pt.sync && scan_points > 0) {
                        if (scan_points >= 150) {
                            long long now = mono_ms();
                            if (prev_scan) {
                                long long dt = now - prev_scan;
                                if (dt > 0) hz_x10 = (int)(10000LL / dt);
                            }
                            prev_scan = now;
                            last_scan = now;
                            ever = true;
                            fault = false;
                            std::memcpy(published, dist, sizeof(published));
                            Level raw = evaluate(&g_cfg, dist, g_dir);
                            deb.push(raw, &g_cfg);
                            if (g_teaching) {
                                for (int b = 0; b < LIDAR_BINS; b++) {
                                    if (dist[b] == 0 || g_teach_n[b] >= 20) continue;
                                    g_teach[g_teach_n[b]++][b] = dist[b];
                                }
                                g_teach_left--;
                                if (g_teach_left <= 0) finish_teach();
                            }
                            last_points = scan_points;
                            publish(true);
                        }
                        std::memset(dist, 0, sizeof(dist));
                        scan_points = 0;
                    }
                    if (pt.quality > 0 && pt.dist_mm > 0) {
                        int b = bin_of(pt.angle_deg, g_cfg.angle_offset_deg);
                        if (dist[b] == 0 || pt.dist_mm < (int)dist[b]) dist[b] = (uint16_t)pt.dist_mm;
                        scan_points++;
                    }
                }
            }
        }

        long long now = mono_ms();
        if (ever && now - last_scan > 300) fault = true;
        else if (!ever && now - started > 3000) fault = true;
        publish(false);

        if (now - last_log >= 2000) {
            last_log = now;
            std::fprintf(stderr, "%s  %d.%d Hz  last scan %d points\n",
                         level_name(fault ? LEVEL_FAULT : deb.published),
                         hz_x10 / 10, hz_x10 % 10, last_points);
        }
    }

    send_simple(fd, 0x25);
    uint8_t stop_pwm[2] = {0, 0};
    send_payload(fd, 0xF0, stop_pwm, 2);
    ::close(fd);
    std::fprintf(stderr, "stopped\n");
    return 0;
}
