#include "sensors/VectornavIMU.h"

LOG_MODULE_REGISTER(VectornavIMU);

K_SEM_DEFINE(vectornav_ready_sem, 0, 1);
K_THREAD_DEFINE(vectornav, 4096, VectornavImu::sense, nullptr, nullptr, nullptr, VECTORNAV_THREAD_PRIORITY, 0, 0);