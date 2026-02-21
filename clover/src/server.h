#ifndef ARTY_SERVER_H
#define ARTY_SERVER_H

#include <string>
#include <expected>
#include "Error.h"
#include "clover.pb.h"

void serve_connections();
int send_fully(int sock, const char* buf, int len);
int send_string_fully(int sock, const std::string& payload);

// Linkage for the valve reset command used in server.cpp
std::expected<void, Error> handle_reset_valve_position(const ResetValvePositionRequest& req);

#endif  // ARTY_SERVER_H
