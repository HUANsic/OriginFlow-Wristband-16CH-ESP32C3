list(APPEND SENSOR_SRC ${SENSOR_IF_DIR}/sensor.c
    ${SENSOR_IF_DIR}/platform/esp32/sensor_ctrl_i2c.c
    ${SENSOR_IF_DIR}/platform/esp32/sensor_ctrl_spi.c)
list(APPEND SENSOR_INCLUDE_DIRS ${SENSOR_IF_DIR}/include
    ${SENSOR_IF_DIR}/interface
    ${SENSOR_IF_DIR}/platform/esp32
    ${SENSOR_IF_DIR}/device
)

if(CONFIG_AD7689_SUPPORT)
    list(APPEND SENSOR_SRC ${SENSOR_IF_DIR}/device/ad/ad7689/ad7689.c)
    list(APPEND SENSOR_INCLUDE_DIRS ${SENSOR_IF_DIR}/device/ad/ad7689)
endif()

idf_component_register(SRCS ${SENSOR_SRC}
    REQUIRES driver LSM6DS3 LIS3MDL Utilities esp_driver_i2c
    INCLUDE_DIRS ${SENSOR_INCLUDE_DIRS})