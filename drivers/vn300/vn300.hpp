#pragma once
#include <stdbool.h>
#include <stddef.h>

bool vn300_init();
bool vn300_read_line(char* out_buf, size_t max_len);
bool vn300_parse_ymr(const char* line, float* yaw, float* pitch, float* roll);