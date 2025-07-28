#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

// Headers de sensores
#include "sensor_bmi270.h"
#include "sensor_max30102.h"
#include "sensor_headers.h"

// Headers de servicios
#include "service_headers.h"

// Headers de tareas
#include "task_postura.h"
#include "task_salud.h"
#include "task_ambiente.h"
#include "task_sedentarismo.h"
#include "task_pasos.h"
#include "task_sueño.h"

static const char *TAG = "MAIN";

// Configuración I2C
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

// Prioridades de tareas
#define PRIORITY_HIGH     (tskIDLE_PRIORITY + 4)
#define PRIORITY_MEDIUM   (tskIDLE_PRIORITY + 3)
#define PRIORITY_LOW      (tskIDLE_PRIORITY + 2)

// Event groups
EventGroupHandle_t system_event_group;
#define SENSORS_INITIALIZED_BIT BIT0
#define BLE_READY_BIT           BIT1
#define STORAGE_READY_BIT       BIT2

// Colas de comunicación entre tareas
QueueHandle_t posture_queue;
QueueHandle_t health_queue;
QueueHandle_t environment_queue;
QueueHandle_t step_queue;

// Mutex para acceso I2C
SemaphoreHandle_t i2c_mutex;

/**
 * Inicialización del bus I2C
 */
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

/**
 * Inicialización de todos los sensores
 */
static esp_err_t sensors_init(void) {
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "Inicializando sensores...");
    
    // Inicializar BMI270 (acelerómetro + giroscopio)
    ret = bmi270_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar BMM150 (magnetómetro)
    ret = bmm150_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMM150 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar MAX30102 (PPG - pulso y SpO2)
    ret = max30102_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar MAX30205 (temperatura corporal)
    ret = max30205_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX30205 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar MAX86176 (hidratación)
    ret = max86176_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX86176 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar BME688 (sensor ambiental)
    ret = bme688_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BME688 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Todos los sensores inicializados correctamente");
    return ESP_OK;
}

/**
 * Inicialización de servicios del sistema
 */
static esp_err_t services_init(void) {
    esp_err_t ret = ESP_OK;
    
    // Inicializar NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Inicializar almacenamiento
    ret = storage_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Storage init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    xEventGroupSetBits(system_event_group, STORAGE_READY_BIT);
    
    // Inicializar motor vibrador
    ret = vibration_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Vibration init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Inicializar BLE
    ret = ble_service_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE service init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    xEventGroupSetBits(system_event_group, BLE_READY_BIT);
    
    // Inicializar gestión de energía
    ret = power_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power manager init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Servicios del sistema inicializados correctamente");
    return ESP_OK;
}

/**
 * Crear todas las tareas del sistema
 */
static void create_tasks(void) {
    ESP_LOGI(TAG, "Creando tareas del sistema...");
    
    // Esperar a que los sensores estén listos
    xEventGroupWaitBits(system_event_group, 
                       SENSORS_INITIALIZED_BIT | STORAGE_READY_BIT | BLE_READY_BIT,
                       pdFALSE, pdTRUE, portMAX_DELAY);
    
    // 1. DETECCIÓN DE POSTURA ENCORVADA
    xTaskCreatePinnedToCore(task_postura_encorvada, 
                           "PosturaEncorvada", 
                           4096, 
                           NULL, 
                           PRIORITY_MEDIUM, 
                           NULL, 
                           1);
    
    // 2. DETECCIÓN DE ESTRÉS
    xTaskCreatePinnedToCore(task_deteccion_estres, 
                           "DeteccionEstres", 
                           6144, 
                           NULL, 
                           PRIORITY_MEDIUM, 
                           NULL, 
                           1);
    
    // 3. CALIDAD DE SUEÑO
    xTaskCreatePinnedToCore(task_calidad_sueño, 
                           "CalidadSueño", 
                           4096, 
                           NULL, 
                           PRIORITY_LOW, 
                           NULL, 
                           1);
    
    // 4. ALERTA DE HIDRATACIÓN
    xTaskCreatePinnedToCore(task_alerta_hidratacion, 
                           "AlertaHidratacion", 
                           4096, 
                           NULL, 
                           PRIORITY_MEDIUM, 
                           NULL, 
                           1);
    
    // 5. RITMO CARDÍACO ELEVADO SIN ACTIVIDAD
    xTaskCreatePinnedToCore(task_ritmo_cardiaco_elevado, 
                           "RitmoCardiacoElevado", 
                           4096, 
                           NULL, 
                           PRIORITY_HIGH, 
                           NULL, 
                           1);
    
    // 6. AMBIENTE MAL VENTILADO
    xTaskCreatePinnedToCore(task_ambiente_mal_ventilado, 
                           "AmbienteMalVentilado", 
                           4096, 
                           NULL, 
                           PRIORITY_LOW, 
                           NULL, 
                           1);
    
    // 7. CONTADOR DE PASOS Y MOVIMIENTO
    xTaskCreatePinnedToCore(task_contador_pasos, 
                           "ContadorPasos", 
                           4096, 
                           NULL, 
                           PRIORITY_LOW, 
                           NULL, 
                           1);
    
    // 8. DETECCIÓN DE SEDENTARISMO MEJORADA
    xTaskCreatePinnedToCore(task_sedentarismo_mejorado, 
                           "SedentarismoMejorado", 
                           4096, 
                           NULL, 
                           PRIORITY_MEDIUM, 
                           NULL, 
                           1);
    
    // 9. ALERTA DE POSTURA INCORRECTA GENERAL
    xTaskCreatePinnedToCore(task_postura_incorrecta_general, 
                           "PosturaIncorrectaGeneral", 
                           4096, 
                           NULL, 
                           PRIORITY_MEDIUM, 
                           NULL, 
                           1);
    
    ESP_LOGI(TAG, "Todas las tareas creadas correctamente");
}

void app_main(void) {
    ESP_LOGI(TAG, "=== FIRMWARE VITAMINA P - INICIANDO ===");
    
    // Crear event group del sistema
    system_event_group = xEventGroupCreate();
    
    // Crear mutex para I2C
    i2c_mutex = xSemaphoreCreateMutex();
    
    // Crear colas de comunicación
    posture_queue = xQueueCreate(10, sizeof(posture_data_t));
    health_queue = xQueueCreate(10, sizeof(health_metrics_t));
    environment_queue = xQueueCreate(5, sizeof(environment_data_t));
    step_queue = xQueueCreate(20, sizeof(step_data_t));
    
    // Inicializar I2C
    ESP_ERROR_CHECK(i2c_master_init());
    
    // Inicializar servicios del sistema
    ESP_ERROR_CHECK(services_init());
    
    // Inicializar sensores
    esp_err_t ret = sensors_init();
    if (ret == ESP_OK) {
        xEventGroupSetBits(system_event_group, SENSORS_INITIALIZED_BIT);
    } else {
        ESP_LOGE(TAG, "Error inicializando sensores, reiniciando...");
        esp_restart();
    }
    
    // Crear tareas
    create_tasks();
    
    // Vibración de inicio
    vibration_set_pattern(VIB_PATTERN_STARTUP);
    
    ESP_LOGI(TAG, "=== FIRMWARE VITAMINA P - SISTEMA LISTO ===");
    
    // Tarea principal - monitoreo del sistema
    while (1) {
        // Verificar estado de batería
        power_manager_check_battery();
        
        // Watchdog del sistema
        esp_task_wdt_reset();
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // Check cada 10 segundos
    }
}