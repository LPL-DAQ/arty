#ifndef CLOVER_SERVER_H
#define CLOVER_SERVER_H

#include <string>

void serve_connections();

int send_fully(int sock, const char *buf, int len);

int send_string_fully(int sock, const std::string &payload);

#endif //CLOVER_SERVER_H
