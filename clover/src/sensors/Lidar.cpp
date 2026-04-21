#include "sensors/Lidar.h"

LOG_MODULE_REGISTER(Lidar);

K_SEM_DEFINE(lidar_1_ready_sem, 0, 1);
K_THREAD_DEFINE(lidar_1, 2048, Lidar1::sense, nullptr, nullptr, nullptr, LIDAR_1_THREAD_PRIORITY, 0, 0);

K_SEM_DEFINE(lidar_2_ready_sem, 0, 1);
K_THREAD_DEFINE(lidar_2, 2048, Lidar2::sense, nullptr, nullptr, nullptr, LIDAR_2_THREAD_PRIORITY, 0, 0);
