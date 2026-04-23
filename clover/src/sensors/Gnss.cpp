#include "sensors/Gnss.h"

namespace Gnss { LOG_MODULE_REGISTER(Gnss); }

K_SEM_DEFINE(gnss_ready_sem, 0, 1);
K_THREAD_DEFINE(gnss_tid, 2048, Gnss::sense, nullptr, nullptr, nullptr, GNSS_THREAD_PRIORITY, 0, 0);
