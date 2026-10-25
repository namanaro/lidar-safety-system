#pragma once

#include <cstdint>

#define LIDAR_STATUS_PATH "/dev/shm/lidar-status.json"
#define LIDAR_DIR_PATH "/dev/shm/lidar-dir"
#define LIDAR_BINS 360
#define LIDAR_MAX_SECTORS 64

struct Sector {
    int start_deg;
    int end_deg;
};

struct Config {
    char port[128];
    int baud;
    int angle_offset_deg;
    int l1_mm;
    int l2_mm;
    int min_adjacent;
    int confirm_scans;
    int clear_scans;
    int chassis_margin_mm;
    int chassis_max_mm;
    bool teach_chassis;
    bool blank[LIDAR_BINS];
    uint16_t chassis_mm[LIDAR_BINS];
};

void config_defaults(Config* c);
bool config_parse(const char* text, Config* c);
bool config_load(const char* path, Config* c);
bool config_save(const char* path, const Config* c);

int sector_count(const Config* c, Sector* out, int max_out);
bool sector_contains(int start_deg, int end_deg, int deg);
