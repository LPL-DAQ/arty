#ifndef APP_SOCKETGUARD_H
#define APP_SOCKETGUARD_H


class SocketGuard {
public:
    int socket;

    explicit SocketGuard(int s);

    ~SocketGuard();
};


#endif //APP_SOCKETGUARD_H
