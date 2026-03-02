idf_component_register(SRCS "Sensors.c"
                    REQUIRES driver AD7689 LSM6DS3 LIS3MDL Utilities
                    INCLUDE_DIRS "include")
