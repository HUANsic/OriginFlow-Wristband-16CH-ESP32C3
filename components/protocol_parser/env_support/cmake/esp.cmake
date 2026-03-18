idf_component_register(SRCS "protocol_parse.cpp" "crc.c" "packet_generate.cpp" INCLUDE_DIRS "../../.")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")