if(CONFIG_GNSS)
target_sources(app PRIVATE Gnss.cpp)
endif()

if(CONFIG_ANALOG_SENSORS)
target_sources(app PRIVATE AnalogSensors.cpp)
endif()

if(CONFIG_LIDAR)
target_sources(app PRIVATE Lidar.cpp)
endif()

if(CONFIG_IMU)
target_sources(app PRIVATE VectornavIMU.cpp)
endif()

# Placeholder interface for the two backup IMUs -- no hardware/Kconfig gate yet since the
# hardware doesn't exist.
target_sources(app PRIVATE BackupImu.cpp)
