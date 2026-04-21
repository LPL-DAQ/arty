#pragma once

#include <zephyr/kernel.h>


// TODO: should this be a class? If it is a class, should we do the = delte thing?
class MutexGuard {
private:
    k_mutex* mutex;

public:
    MutexGuard(k_mutex* mut);
    ~MutexGuard();
};
