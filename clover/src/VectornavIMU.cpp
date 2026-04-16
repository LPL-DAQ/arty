#include "VectornavIMU.h"

LOG_MODULE_REGISTER(VectornavIMU, LOG_LEVEL_INF);

K_SEM_DEFINE(vectornav_1_ready_sem, 0, 1);
K_THREAD_DEFINE(vectornav_1, 4096, Vectornav1::sense, nullptr, nullptr, nullptr, VECTORNAV_THREAD_PRIORITY, 0, 0);