#ifndef APP_MUTEXGUARD_H
#define APP_MUTEXGUARD_H

#include <zephyr/kernel.h>

class MutexGuard {
private:
    k_mutex* mutex;

public:
    MutexGuard(k_mutex* mut);
    ~MutexGuard();
};

#endif  // APP_MUTEXGUARD_H
