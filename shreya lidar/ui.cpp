#include "config.h"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

static volatile sig_atomic_t g_run = 1;
static void on_signal(int) { g_run = 0; }

static char g_html_path[640];
static const char* g_cfg_path = "config.json";

static void find_html() {
    char exe[480];
    ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (n < 0) {
        std::snprintf(g_html_path, sizeof(g_html_path), "index.html");
        return;
    }
    exe[n] = 0;
    char* slash = std::strrchr(exe, '/');
    if (!slash) {
        std::snprintf(g_html_path, sizeof(g_html_path), "index.html");
        return;
    }
    *slash = 0;
    std::snprintf(g_html_path, sizeof(g_html_path), "%s/index.html", exe);
}

static char* read_file(const char* path, size_t* out_n) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return nullptr;
    if (std::fseek(f, 0, SEEK_END) != 0) {
        std::fclose(f);
        return nullptr;
    }
    long sz = std::ftell(f);
    if (sz < 0 || sz > 200000) {
        std::fclose(f);
        return nullptr;
    }
    std::rewind(f);
    char* buf = (char*)std::malloc((size_t)sz + 1);
    if (!buf) {
        std::fclose(f);
        return nullptr;
    }
    size_t n = std::fread(buf, 1, (size_t)sz, f);
    std::fclose(f);
    buf[n] = 0;
    *out_n = n;
    return buf;
}

static void respond(int fd, int code, const char* reason, const char* type, const char* body, size_t n) {
    char hdr[512];
    int h = std::snprintf(hdr, sizeof(hdr),
                          "HTTP/1.1 %d %s\r\n"
                          "Content-Type: %s\r\n"
                          "Content-Length: %zu\r\n"
                          "Access-Control-Allow-Origin: *\r\n"
                          "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
                          "Access-Control-Allow-Headers: *\r\n"
                          "Cache-Control: no-store\r\n"
                          "Connection: close\r\n\r\n",
                          code, reason, type, n);
    if (h > 0) {
        ssize_t w = ::write(fd, hdr, (size_t)h);
        (void)w;
    }
    size_t off = 0;
    while (off < n) {
        ssize_t w = ::write(fd, body + off, n - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            break;
        }
        off += (size_t)w;
    }
}

static bool read_request(int fd, char* buf, size_t cap, int* out_len) {
    size_t n = 0;
    while (n + 1 < cap && g_run) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int pr = poll(&pfd, 1, 2000);
        if (pr <= 0) break;
        ssize_t r = ::read(fd, buf + n, cap - 1 - n);
        if (r < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (r == 0) break;
        n += (size_t)r;
        buf[n] = 0;
        char* hdr_end = std::strstr(buf, "\r\n\r\n");
        if (!hdr_end) continue;
        int header_len = (int)(hdr_end - buf) + 4;
        int content_len = 0;
        char* cl = std::strstr(buf, "Content-Length:");
        if (cl && cl < hdr_end) content_len = std::atoi(cl + 15);
        if (content_len < 0 || header_len + content_len >= (int)cap) return false;
        if ((int)n >= header_len + content_len) {
            *out_len = (int)n;
            return true;
        }
    }
    *out_len = (int)n;
    return false;
}

static void handle(int fd) {
    char req[128 * 1024];
    int n = 0;
    if (!read_request(fd, req, sizeof(req), &n)) {
        respond(fd, 400, "Bad Request", "text/plain", "bad request", 11);
        return;
    }
    char method[8] = {};
    char url[256] = {};
    if (std::sscanf(req, "%7s %255s", method, url) != 2) {
        respond(fd, 400, "Bad Request", "text/plain", "bad request", 11);
        return;
    }
    char* q = std::strchr(url, '?');
    if (q) *q = 0;

    if (std::strcmp(method, "OPTIONS") == 0) {
        respond(fd, 204, "No Content", "text/plain", "", 0);
        return;
    }

    if (std::strcmp(method, "GET") == 0 && (std::strcmp(url, "/") == 0 || std::strcmp(url, "/index.html") == 0)) {
        size_t hn = 0;
        char* html = read_file(g_html_path, &hn);
        if (!html) {
            respond(fd, 500, "Error", "text/plain", "index.html missing", 18);
            return;
        }
        respond(fd, 200, "OK", "text/html; charset=utf-8", html, hn);
        std::free(html);
        return;
    }

    if (std::strcmp(method, "GET") == 0 && std::strcmp(url, "/api/status") == 0) {
        size_t sn = 0;
        char* body = read_file(LIDAR_STATUS_PATH, &sn);
        if (!body) {
            const char* empty = "{\"level\":\"FAULT\",\"dir\":\"S\",\"ms\":0,\"hz\":0,\"dist_mm\":[]}";
            respond(fd, 200, "OK", "application/json", empty, std::strlen(empty));
            return;
        }
        respond(fd, 200, "OK", "application/json", body, sn);
        std::free(body);
        return;
    }

    if (std::strcmp(method, "GET") == 0 && std::strcmp(url, "/api/config") == 0) {
        size_t cn = 0;
        char* body = read_file(g_cfg_path, &cn);
        if (!body) {
            Config cfg;
            config_defaults(&cfg);
            config_save(g_cfg_path, &cfg);
            body = read_file(g_cfg_path, &cn);
        }
        if (!body) {
            respond(fd, 500, "Error", "text/plain", "no config", 9);
            return;
        }
        respond(fd, 200, "OK", "application/json", body, cn);
        std::free(body);
        return;
    }

    if (std::strcmp(method, "POST") == 0 && std::strcmp(url, "/api/dir") == 0) {
        char* hdr_end = std::strstr(req, "\r\n\r\n");
        const char* body = hdr_end ? hdr_end + 4 : "";
        char dir = 'S';
        for (const char* p = body; *p; ++p) {
            if (*p == 'F' || *p == 'f') { dir = 'F'; break; }
            if (*p == 'R' || *p == 'r') { dir = 'R'; break; }
            if (*p == 'S' || *p == 's') { dir = 'S'; break; }
        }
        char tmp[] = "/dev/shm/lidar-dir.tmp";
        FILE* out = std::fopen(tmp, "w");
        if (!out) {
            const char* err = "{\"ok\":false}";
            respond(fd, 500, "Error", "application/json", err, std::strlen(err));
            return;
        }
        std::fputc(dir, out);
        std::fputc('\n', out);
        std::fclose(out);
        if (std::rename(tmp, LIDAR_DIR_PATH) != 0) {
            const char* err = "{\"ok\":false}";
            respond(fd, 500, "Error", "application/json", err, std::strlen(err));
            return;
        }
        char ok[32];
        int n = std::snprintf(ok, sizeof(ok), "{\"dir\":\"%c\"}", dir);
        respond(fd, 200, "OK", "application/json", ok, (size_t)n);
        return;
    }

    if (std::strcmp(method, "POST") == 0 && std::strcmp(url, "/api/config") == 0) {
        char* hdr_end = std::strstr(req, "\r\n\r\n");
        const char* body = hdr_end ? hdr_end + 4 : "";
        Config cfg;
        if (!config_parse(body, &cfg)) {
            const char* err = "{\"ok\":false}";
            respond(fd, 400, "Bad Request", "application/json", err, std::strlen(err));
            return;
        }
        if (!config_save(g_cfg_path, &cfg)) {
            const char* err = "{\"ok\":false}";
            respond(fd, 500, "Error", "application/json", err, std::strlen(err));
            return;
        }
        size_t cn = 0;
        char* saved = read_file(g_cfg_path, &cn);
        if (!saved) {
            const char* ok = "{\"ok\":true}";
            respond(fd, 200, "OK", "application/json", ok, std::strlen(ok));
            return;
        }
        respond(fd, 200, "OK", "application/json", saved, cn);
        std::free(saved);
        return;
    }

    respond(fd, 404, "Not Found", "text/plain", "not found", 9);
}

int main(int argc, char** argv) {
    g_cfg_path = (argc > 1) ? argv[1] : "config.json";
    int port = (argc > 2) ? std::atoi(argv[2]) : 8080;
    if (port <= 0 || port > 65535) port = 8080;
    find_html();

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    int srv = ::socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) {
        std::perror("socket");
        return 1;
    }
    int yes = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(srv, (sockaddr*)&addr, sizeof(addr)) != 0) {
        std::fprintf(stderr, "bind %d failed: %s\n", port, std::strerror(errno));
        ::close(srv);
        return 1;
    }
    if (listen(srv, 4) != 0) {
        std::perror("listen");
        ::close(srv);
        return 1;
    }
    std::fprintf(stderr, "lidar-ui http://0.0.0.0:%d/  config %s\n", port, g_cfg_path);
    std::fprintf(stderr, "page %s\n", g_html_path);

    while (g_run) {
        pollfd pfd{};
        pfd.fd = srv;
        pfd.events = POLLIN;
        int pr = poll(&pfd, 1, 200);
        if (pr < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (pr == 0) continue;
        int fd = accept(srv, nullptr, nullptr);
        if (fd < 0) {
            if (errno == EINTR) continue;
            break;
        }
        handle(fd);
        ::close(fd);
    }
    ::close(srv);
    return 0;
}
