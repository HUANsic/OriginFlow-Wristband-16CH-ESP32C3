set(EMBED_FILES "")

# list(APPEND EMBED_FILES
# )


set(SRC_DIRS ${SRC_DIRS} ${APP_MAIN_ROOT}/ ${APP_MAIN_ROOT}  ${APP_MAIN_ROOT}/bluetooth)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ".")
set(APP_REQUIRES
    esp_driver_i2s
    esp_driver_gpio
    efuse
    nvs_flash
    esp_wifi
    esp_event
    console
    driver
    esp_adc
    # esp_http_server
    # esp_https_ota
    # app_update
    sensors
    LSM6DS3
    # testbench1
    protocol_parser
)
message(STATUS "app req is ${APP_REQUIRES}")
idf_component_register(SRC_DIRS ${SRC_DIRS}
    INCLUDE_DIRS ${INCLUDE_DIRS} REQUIRES ${APP_REQUIRES} EMBED_TXTFILES "${EMBED_FILES}")

# if(CONFIG_BT_ENABLED)
    message(STATUS "CONFIG_BT_ENABLED is ${CONFIG_BT_ENABLED}")
    add_definitions(-DCONFIG_BT_ENABLED) 
    file(GLOB_RECURSE BT_SOURCES ${APP_MAIN_ROOT}/bluetooth/*.c**)
    target_sources(${COMPONENT_LIB} PRIVATE ${BT_SOURCES})
    target_include_directories(${COMPONENT_LIB} PRIVATE ${APP_MAIN_ROOT}/bluetooth)
    target_link_libraries(${COMPONENT_LIB} PUBLIC idf::bt)
# endif()

# message(STATUS "EXTRA_COMPONENT_DIRS is ${EXTRA_COMPONENT_DIRS}")
# list(APPEND UI_INCLUDES
#     ${APP_MAIN_ROOT}/ui
#     ${APP_MAIN_ROOT}/ui/components
#     ${APP_MAIN_ROOT}/ui/screens
#     ${APP_MAIN_ROOT}/ui/fonts
#     ${APP_MAIN_ROOT}/ui/assets
# )
# list(APPEND UI_SOURCES_DIR
#     ${APP_MAIN_ROOT}/ui
#     ${APP_MAIN_ROOT}/ui/components
#     ${APP_MAIN_ROOT}/ui/screens
#     ${APP_MAIN_ROOT}/ui/assets
#     ${APP_MAIN_ROOT}/ui/common
# )
# file(GLOB_RECURSE UI_SOURCES ${APP_MAIN_ROOT}/ui/*.c ${APP_MAIN_ROOT}/ui/components/*.c ${APP_MAIN_ROOT}/ui/screens/*.c ${APP_MAIN_ROOT}/ui/assets/*.c ${APP_MAIN_ROOT}/ui/common/*.c)

# target_add_binary_data(${COMPONENT_LIB} ${APP_MAIN_ROOT}/ui/assets/360.jpg BINARY)
# target_add_binary_data(${COMPONENT_LIB} ${APP_MAIN_ROOT}/ui/assets/peace_hand.jpg BINARY)

# set(UI_REQUIRES ${UI_REQUIRES} lvgl esp32_eyes phancy_idf freetype)

# foreach(s ${UI_REQUIRES})
#     target_link_libraries(${COMPONENT_LIB} PUBLIC idf::${s})
# endforeach()

# target_sources(${COMPONENT_LIB} PUBLIC ${UI_SOURCES})
# target_include_directories(${COMPONENT_LIB} PUBLIC ${UI_INCLUDES})


get_target_property(links ${COMPONENT_LIB} LINK_LIBRARIES)
message(STATUS "app main component links is: ${links}")

get_target_property(srcss ${COMPONENT_LIB} SOURCES)
message(STATUS "app main component srcs is: ${srcss}")

get_target_property(includes ${COMPONENT_LIB} INCLUDE_DIRECTORIES)
message(STATUS "app main ${COMPONENT_LIB} includes is: ${includes}")

message(STATUS "app main src dirs is:")

foreach(s ${SRC_DIRS})
    message(STATUS "${s}")
endforeach()

message(STATUS "app main include dirs is:")

foreach(s ${INCLUDE_DIRS})
    message(STATUS "${s}")
endforeach()

message(STATUS "app main requires is:")

foreach(s ${REQUIRES})
    message(STATUS "${s}")
endforeach()

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
