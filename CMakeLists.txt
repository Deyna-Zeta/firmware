cmake_minimum_required(VERSION 3.16)

# Configuración del proyecto
set(PROJECT_NAME "firmware_vitamina_p")
set(PROJECT_VER "1.0.0")

# Incluir ESP-IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Definir el proyecto
project(${PROJECT_NAME})

# Configuración de componentes
idf_component_register(
    SRCS 
        "main.c"
        "src/tasks/task_postura_encorvada.c"
        "src/tasks/task_deteccion_estres.c"
        "src/tasks/task_calidad_sueño.c"
        "src/tasks/task_alertas_salud.c"
        "src/tasks/task_actividad_fisica.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        freertos
        esp_system
        esp_timer
        driver
        nvs_flash
        esp_log
        esp_pm
        esp_event
        esp_wifi
        bt
        app_update
        spiffs
        vfs
        fatfs
)