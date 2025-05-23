idf_component_register(
    SRCS "src/RV3032.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos
)
