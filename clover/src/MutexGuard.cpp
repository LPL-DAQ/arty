#include "MutexGuard.h"

/// Acquires a mutex, blocking till it can be acquired. The mutex is released when the guard
/// goes out of scope.
MutexGuard::MutexGuard(k_mutex* mutex) : mutex{mutex}
{
    k_mutex_lock(mutex, K_FOREVER);
}

/// Releases the mutex upon destruction of the guard.
MutexGuard::~MutexGuard()
{
    k_mutex_unlock(mutex);
}
