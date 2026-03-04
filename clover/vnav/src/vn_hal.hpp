#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool vn_init();
bool vn_read_line(char* out_buf, size_t max_len);
void vn_uart_send(const uint8_t* data, size_t len);
void vn_configure_outputs();
bool vn_parse_ymr(const char* line, float* yaw, float* pitch, float* roll);
bool vn_parse_imu(const char* line, float* ax, float* ay, float* az,
                                     float* gx, float* gy, float* gz);
bool vn_parse_gps(const char* line, double* lat, double* lon, float* alt);
bool vn_parse_ins(const char* line, double* lat, double* lon, float* alt,
                                     float* vn, float* ve, float* vd);
bool vn_parse_mag(const char* line, float* mx, float* my, float* mz);