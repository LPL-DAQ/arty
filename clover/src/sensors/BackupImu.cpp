#include "sensors/BackupImu.h"

// Separate from BackupImu.h because LOG_MODULE_REGISTER must appear in exactly one translation
// unit -- if it lived in the header, every file that includes BackupImu.h would register its own
// copy and the linker would reject the duplicate symbols. Same reason Lidar.h/VectornavIMU.h each
// have a matching .cpp for just this one line (see Lidar.cpp/VectornavIMU.cpp).
LOG_MODULE_REGISTER(BackupImu);
