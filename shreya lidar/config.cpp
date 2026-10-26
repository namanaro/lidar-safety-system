#include "config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

void config_defaults(Config* c) {
    std::memset(c, 0, sizeof(*c));
    std::snprintf(c->port, sizeof(c->port), "/dev/ttyUSB0");
    c->baud = 460800;
    c->angle_offset_deg = 0;
    c->l1_mm = 1000;
    c->l2_mm = 550;
    c->ignore_inner_mm = 0;
    c->min_adjacent = 3;
    c->confirm_scans = 2;
    c->clear_scans = 3;
    c->chassis_margin_mm = 40;
    c->chassis_max_mm = 400;
}

static const char* find_key(const char* json, const char* key) {
    char pat[80];
    std::snprintf(pat, sizeof(pat), "\"%s\"", key);
    return std::strstr(json, pat);
}

static const char* skip_sep(const char* p) {
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ':') p++;
    return p;
}

static int json_int(const char* json, const char* key, int fallback) {
    const char* p = find_key(json, key);
    if (!p) return fallback;
    p = skip_sep(p + std::strlen(key) + 2);
    if (*p == '-' || (*p >= '0' && *p <= '9')) return std::atoi(p);
    return fallback;
}

static bool json_bool(const char* json, const char* key, bool fallback) {
    const char* p = find_key(json, key);
    if (!p) return fallback;
    p = skip_sep(p + std::strlen(key) + 2);
    if (std::strncmp(p, "true", 4) == 0) return true;
    if (std::strncmp(p, "false", 5) == 0) return false;
    return fallback;
}

static void json_string(const char* json, const char* key, char* out, size_t out_len) {
    const char* p = find_key(json, key);
    if (!p) return;
    p = skip_sep(p + std::strlen(key) + 2);
    if (*p != '"') return;
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < out_len) out[n++] = *p++;
    out[n] = 0;
}

static void clamp_config(Config* c) {
    if (c->port[0] == 0) std::snprintf(c->port, sizeof(c->port), "/dev/ttyUSB0");
    if (c->baud < 9600) c->baud = 460800;
    if (c->l1_mm < 100) c->l1_mm = 100;
    if (c->l1_mm > 12000) c->l1_mm = 12000;
    if (c->l2_mm < 50) c->l2_mm = 50;
    if (c->l2_mm >= c->l1_mm) c->l2_mm = c->l1_mm - 1;
    if (c->ignore_inner_mm < 0) c->ignore_inner_mm = 0;
    if (c->ignore_inner_mm > 5000) c->ignore_inner_mm = 5000;
    if (c->min_adjacent < 1) c->min_adjacent = 1;
    if (c->min_adjacent > 30) c->min_adjacent = 30;
    if (c->confirm_scans < 1) c->confirm_scans = 1;
    if (c->confirm_scans > 20) c->confirm_scans = 20;
    if (c->clear_scans < 1) c->clear_scans = 1;
    if (c->clear_scans > 20) c->clear_scans = 20;
    if (c->chassis_margin_mm < 0) c->chassis_margin_mm = 0;
    if (c->chassis_margin_mm > 500) c->chassis_margin_mm = 500;
    if (c->chassis_max_mm < 50) c->chassis_max_mm = 50;
    if (c->chassis_max_mm > 2000) c->chassis_max_mm = 2000;
    c->angle_offset_deg %= 360;
    if (c->angle_offset_deg < 0) c->angle_offset_deg += 360;
}

bool sector_contains(int start_deg, int end_deg, int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    start_deg %= 360;
    if (start_deg < 0) start_deg += 360;
    end_deg %= 360;
    if (end_deg < 0) end_deg += 360;
    if (start_deg <= end_deg) return deg >= start_deg && deg <= end_deg;
    return deg >= start_deg || deg <= end_deg;
}

static void mark_sector(Config* c, int start_deg, int end_deg) {
    for (int d = 0; d < LIDAR_BINS; d++) {
        if (sector_contains(start_deg, end_deg, d)) c->blank[d] = true;
    }
}

static const char* parse_sectors(const char* json, Config* c) {
    const char* key = find_key(json, "blank_sectors");
    if (!key) return json;
    const char* p = std::strchr(key, '[');
    if (!p) return json;
    p++;
    int depth = 1;
    const char* end = p;
    while (*end && depth) {
        if (*end == '[') depth++;
        else if (*end == ']') depth--;
        if (depth) end++;
    }
    while (p < end) {
        const char* s = std::strstr(p, "\"start_deg\"");
        if (!s || s >= end) break;
        const char* e = std::strstr(s, "\"end_deg\"");
        if (!e || e >= end) break;
        int start = std::atoi(skip_sep(s + 11));
        int stop = std::atoi(skip_sep(e + 9));
        mark_sector(c, start, stop);
        p = e + 9;
    }
    return end;
}

static void parse_chassis(const char* json, Config* c) {
    const char* key = find_key(json, "chassis_mm");
    if (!key) return;
    const char* p = std::strchr(key, '[');
    if (!p) return;
    p++;
    for (int i = 0; i < LIDAR_BINS && *p && *p != ']'; i++) {
        while (*p && (*p < '0' || *p > '9') && *p != '-') {
            if (*p == ']') return;
            p++;
        }
        if (!*p || *p == ']') return;
        c->chassis_mm[i] = (uint16_t)std::atoi(p);
        while (*p && *p != ',' && *p != ']') p++;
    }
}

bool config_parse(const char* text, Config* c) {
    if (!text || !find_key(text, "l1_mm") || !find_key(text, "l2_mm")) return false;
    config_defaults(c);
    json_string(text, "port", c->port, sizeof(c->port));
    c->baud = json_int(text, "baud", c->baud);
    c->angle_offset_deg = json_int(text, "angle_offset_deg", c->angle_offset_deg);
    c->l1_mm = json_int(text, "l1_mm", c->l1_mm);
    c->l2_mm = json_int(text, "l2_mm", c->l2_mm);
    c->ignore_inner_mm = json_int(text, "ignore_inner_mm", c->ignore_inner_mm);
    c->min_adjacent = json_int(text, "min_adjacent", c->min_adjacent);
    c->confirm_scans = json_int(text, "confirm_scans", c->confirm_scans);
    c->clear_scans = json_int(text, "clear_scans", c->clear_scans);
    c->chassis_margin_mm = json_int(text, "chassis_margin_mm", c->chassis_margin_mm);
    c->chassis_max_mm = json_int(text, "chassis_max_mm", c->chassis_max_mm);
    c->teach_chassis = json_bool(text, "teach_chassis", false);
    parse_sectors(text, c);
    parse_chassis(text, c);
    clamp_config(c);
    return true;
}

bool config_load(const char* path, Config* c) {
    FILE* f = std::fopen(path, "rb");
    if (!f) {
        config_defaults(c);
        return false;
    }
    char buf[96 * 1024];
    size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
    std::fclose(f);
    buf[n] = 0;
    if (n == 0) {
        config_defaults(c);
        return false;
    }
    return config_parse(buf, c);
}

int sector_count(const Config* c, Sector* out, int max_out) {
    int n = 0;
    int run = -1;
    for (int i = 0; i < LIDAR_BINS; i++) {
        if (c->blank[i]) {
            if (run < 0) run = i;
        } else if (run >= 0) {
            if (n < max_out) {
                out[n].start_deg = run;
                out[n].end_deg = i - 1;
                n++;
            }
            run = -1;
        }
    }
    if (run >= 0 && n < max_out) {
        if (n > 0 && out[0].start_deg == 0) {
            out[0].start_deg = run;
        } else {
            out[n].start_deg = run;
            out[n].end_deg = LIDAR_BINS - 1;
            n++;
        }
    }
    return n;
}

bool config_save(const char* path, const Config* c) {
    char tmp[512];
    std::snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE* f = std::fopen(tmp, "wb");
    if (!f) return false;

    Sector sectors[LIDAR_MAX_SECTORS];
    int ns = sector_count(c, sectors, LIDAR_MAX_SECTORS);

    std::fprintf(f, "{\n");
    std::fprintf(f, "  \"port\": \"%s\",\n", c->port);
    std::fprintf(f, "  \"baud\": %d,\n", c->baud);
    std::fprintf(f, "  \"angle_offset_deg\": %d,\n", c->angle_offset_deg);
    std::fprintf(f, "  \"l1_mm\": %d,\n", c->l1_mm);
    std::fprintf(f, "  \"l2_mm\": %d,\n", c->l2_mm);
    std::fprintf(f, "  \"ignore_inner_mm\": %d,\n", c->ignore_inner_mm);
    std::fprintf(f, "  \"min_adjacent\": %d,\n", c->min_adjacent);
    std::fprintf(f, "  \"confirm_scans\": %d,\n", c->confirm_scans);
    std::fprintf(f, "  \"clear_scans\": %d,\n", c->clear_scans);
    std::fprintf(f, "  \"chassis_margin_mm\": %d,\n", c->chassis_margin_mm);
    std::fprintf(f, "  \"chassis_max_mm\": %d,\n", c->chassis_max_mm);
    std::fprintf(f, "  \"teach_chassis\": %s,\n", c->teach_chassis ? "true" : "false");
    std::fprintf(f, "  \"blank_sectors\": [");
    for (int i = 0; i < ns; i++) {
        std::fprintf(f, "%s{\"start_deg\": %d, \"end_deg\": %d}",
                     i ? ", " : "", sectors[i].start_deg, sectors[i].end_deg);
    }
    std::fprintf(f, "],\n");
    std::fprintf(f, "  \"chassis_mm\": [");
    for (int i = 0; i < LIDAR_BINS; i++) {
        if (i) std::fprintf(f, ",");
        if (i % 20 == 0) std::fprintf(f, "\n    ");
        std::fprintf(f, "%u", c->chassis_mm[i]);
    }
    std::fprintf(f, "\n  ]\n}\n");
    if (std::fclose(f) != 0) return false;
    if (std::rename(tmp, path) != 0) return false;
    return true;
}
