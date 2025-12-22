#ifndef ARTY_SERVER_H
#define ARTY_SERVER_H

#include <string>

void serve_connections();

int send_fully(int sock, const char *buf, int len);

int send_string_fully(int sock, const std::string &payload);

#endif //ARTY_SERVER_H
