/**
 * FUNCIONES 4, 5, 6: ALERTAS DE SALUD Y AMBIENTE
 * 
 * FUNCIÓN 4: ALERTA DE HIDRATACIÓN
 * Sensores: MAX86176 + MAX30205
 * Función: Correlacionar hidratación cutánea óptica con temperatura corporal
 * Criterios: Hidratación baja detectada por sensor óptico + temperatura elevada
 * Alerta: "Hidratación baja: Bebe agua"
 * Acción: Notificación simple
 * 
 * FUNCIÓN 5: RITMO CARDÍACO ELEVADO SIN ACTIVIDAD
 * Sensores: MAX30102 + BMI270
 * Función: Detectar frecuencia cardíaca >100 BPM sin movimiento físico reciente
 * Tiempo: Sin actividad por >5 minutos pero HR elevada
 * Alerta: "Ritmo cardíaco elevado sin actividad física. ¿Estás estresado?"
 * Acción: Vibración + notificación
 * 
 * FUNCIÓN 6: AMBIENTE MAL VENTILADO
 * Sensor: BME688
 * Función: Detectar CO2 alto y VOCs (Compuestos Orgánicos Volátiles)
 * Umbral: Resistencia de gas <50 kΩ (indicador de mala calidad del aire)
 * Alerta: "Ambiente mal ventilado"
 * Sugerencia: "Abre una ventana"
 */

#include "task_salud.h"
#include "sensor_bmi270.h"
#include "sensor_max30102.h"
#include "sensor_headers.h"
#include "service_headers.h"
#include "common_types.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "ALERTAS_SALUD";

// ==================== FUNCIÓN 4: ALERTA DE HIDRATACIÓN ====================

// Variables de estado de hidratación
static uint32_t last_hydration_alert = 0;
static const uint32_t HYDRATION_ALERT_COOLDOWN_MS = 2 * 60 * 60 * 1000; // 2 horas
static const uint32_t HYDRATION_REMINDER_INTERVAL_MS = 2 * 60 * 60 * 1000; // 2 horas

/**
 * FUNCIÓN 4: Monitorear nivel de hidratación
 */
static void check_hydration_levels(void) {
    hydration_data_t hydration_data;
    float body_temperature;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Leer datos de hidratación del MAX86176
    esp_err_t ret = max86176_read_hydration(&hydration_data);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Error leyendo hidratación: %s", esp_err_to_name(ret));
        return;
    }
    
    // Leer temperatura corporal
    body_temperature = max30205_read_temperature();
    
    // Evaluar condiciones de hidratación baja
    bool low_hydration = hydration_data.hydration_level < HIDRATACION_BAJA_THRESHOLD;
    bool elevated_temp = body_temperature > 37.0f; // >37°C
    bool time_for_reminder = (current_time - last_hydration_alert) > HYDRATION_REMINDER_INTERVAL_MS;
    
    // Determinar si activar alerta
    bool should_alert = false;
    char alert_message[100];
    
    if (low_hydration && elevated_temp) {
        // Hidratación baja + temperatura elevada = alerta urgente
        should_alert = true;
        snprintf(alert_message, sizeof(alert_message), 
                "Hidratación baja (%.0f%%) y temperatura elevada (%.1f°C): Bebe agua AHORA", 
                (float)hydration_data.hydration_level, body_temperature);
        ESP_LOGW(TAG, "ALERTA URGENTE: %s", alert_message);
    } else if (low_hydration && time_for_reminder) {
        // Solo hidratación baja = recordatorio suave
        should_alert = true;
        snprintf(alert_message, sizeof(alert_message), 
                "Hidratación baja: Bebe agua");
        ESP_LOGI(TAG, "Recordatorio: %s", alert_message);
    }
    
    if (should_alert) {
        // Activar vibración suave
        vibration_set_pattern(VIB_PATTERN_HYDRATION_ALERT);
        
        // Enviar notificación por BLE
        if (ble_is_connected()) {
            ble_notify_hydration_alert(hydration_data.hydration_level, body_temperature, alert_message);
        }
        
        // Registrar evento
        storage_log_event("HYDRATION_ALERT", low_hydration && elevated_temp ? "URGENT" : "REMINDER");
        
        last_hydration_alert = current_time;
    }
    
    ESP_LOGD(TAG, "Hidratación: %d%%, Temp: %.1f°C", 
             hydration_data.hydration_level, body_temperature);
}

// ==================== FUNCIÓN 5: RITMO CARDÍACO ELEVADO SIN ACTIVIDAD ====================

// Variables de estado para HR elevada
static uint32_t last_significant_activity = 0;
static uint32_t elevated_hr_start = 0;
static uint32_t last_hr_alert = 0;
static const uint32_t HR_ALERT_COOLDOWN_MS = 30 * 60 * 1000; // 30 minutos

/**
 * FUNCIÓN 5: Detectar ritmo cardíaco elevado sin actividad física
 */
static void check_elevated_hr_without_activity(void) {
    // Leer datos de sensores
    max30102_sample_t ppg_samples[16];
    size_t sample_count;
    imu_data_t imu_data;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Obtener frecuencia cardíaca
    uint16_t heart_rate = 0;
    if (max30102_read_fifo(ppg_samples, &sample_count) == ESP_OK && sample_count > 0) {
        heart_rate = max30102_calculate_heart_rate(ppg_samples, sample_count);
    }
    
    if (heart_rate == 0) {
        return; // No hay datos válidos
    }
    
    // Obtener datos de movimiento
    esp_err_t ret = bmi270_read_data(&imu_data);
    if (ret != ESP_OK) {
        return;
    }
    
    // Calcular magnitud de movimiento
    float movement_magnitude = vector3_magnitude(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
    
    // Detectar actividad significativa (umbral más alto que para sedentarismo)
    bool significant_activity = movement_magnitude > 2.0f; // 2.0 m/s²
    
    if (significant_activity) {
        last_significant_activity = current_time;
        elevated_hr_start = 0; // Reset timer de HR elevada
        return;
    }
    
    // Verificar si HR está elevada
    bool hr_elevated = heart_rate > RITMO_CARDIACO_ELEVADO_THRESHOLD;
    
    if (hr_elevated) {
        if (elevated_hr_start == 0) {
            elevated_hr_start = current_time;
        }
        
        // Verificar si ha pasado suficiente tiempo sin actividad
        uint32_t time_without_activity = current_time - last_significant_activity;
        uint32_t time_with_elevated_hr = current_time - elevated_hr_start;
        
        bool prolonged_inactivity = time_without_activity > RITMO_CARDIACO_SIN_ACTIVIDAD_TIME_MS;
        bool prolonged_elevated_hr = time_with_elevated_hr > RITMO_CARDIACO_SIN_ACTIVIDAD_TIME_MS;
        bool can_alert = (current_time - last_hr_alert) > HR_ALERT_COOLDOWN_MS;
        
        if (prolonged_inactivity && prolonged_elevated_hr && can_alert) {
            ESP_LOGW(TAG, "ALERTA: Ritmo cardíaco elevado (%d BPM) sin actividad física por %lu minutos", 
                     heart_rate, time_without_activity / (60 * 1000));
            
            // Activar vibración de alerta
            vibration_set_pattern(VIB_PATTERN_HR_ELEVATED_ALERT);
            
            // Enviar notificación por BLE
            if (ble_is_connected()) {
                char message[] = "Ritmo cardíaco elevado sin actividad física. ¿Estás estresado?";
                ble_notify_hr_elevated_alert(heart_rate, time_without_activity, message);
            }
            
            // Registrar evento
            storage_log_event("HR_ELEVATED_NO_ACTIVITY", "ALERT_TRIGGERED");
            
            last_hr_alert = current_time;
            elevated_hr_start = current_time; // Reset para evitar alertas continuas
        }
    } else {
        elevated_hr_start = 0; // Reset si HR vuelve a normal
    }
    
    ESP_LOGD(TAG, "HR: %d BPM, Movimiento: %.2f m/s², Sin actividad: %lu min", 
             heart_rate, movement_magnitude, 
             (current_time - last_significant_activity) / (60 * 1000));
}

// ==================== FUNCIÓN 6: AMBIENTE MAL VENTILADO ====================

// Variables de estado del ambiente
static uint32_t last_air_quality_alert = 0;
static const uint32_t AIR_QUALITY_ALERT_COOLDOWN_MS = 30 * 60 * 1000; // 30 minutos

/**
 * FUNCIÓN 6: Monitorear calidad del aire y ventilación
 */
static void check_air_quality(void) {
    environment_data_t env_data;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Forzar medición del BME688
    bme688_force_measurement();
    vTaskDelay(pdMS_TO_TICKS(200)); // Esperar medición
    
    // Leer datos ambientales
    esp_err_t ret = bme688_read_all(&env_data);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Error leyendo BME688: %s", esp_err_to_name(ret));
        return;
    }
    
    // Evaluar calidad del aire
    bool poor_air_quality = env_data.gas_resistance < AMBIENTE_MAL_VENTILADO_THRESHOLD; // <50 kΩ
    bool high_voc = env_data.voc_equivalent > 250.0f; // >250 ppm VOC
    bool can_alert = (current_time - last_air_quality_alert) > AIR_QUALITY_ALERT_COOLDOWN_MS;
    
    if ((poor_air_quality || high_voc) && can_alert) {
        ESP_LOGW(TAG, "ALERTA: Ambiente mal ventilado - Resistencia gas: %d kΩ, VOC: %.1f ppm", 
                 env_data.gas_resistance, env_data.voc_equivalent);
        
        // Activar vibración de alerta de calidad del aire
        vibration_set_pattern(VIB_PATTERN_AIR_QUALITY_ALERT);
        
        // Enviar notificación por BLE
        if (ble_is_connected()) {
            char message[100];
            snprintf(message, sizeof(message), 
                    "Ambiente mal ventilado (Gas: %dkΩ, VOC: %.0fppm). Abre una ventana",
                    env_data.gas_resistance, env_data.voc_equivalent);
            ble_notify_air_quality_alert(&env_data, message);
        }
        
        // Registrar evento
        storage_log_event("AIR_QUALITY_POOR", "VENTILATION_NEEDED");
        
        last_air_quality_alert = current_time;
    }
    
    ESP_LOGD(TAG, "Calidad aire - Gas: %d kΩ, VOC: %.1f ppm, CO2 eq: %.1f ppm", 
             env_data.gas_resistance, env_data.voc_equivalent, env_data.co2_equivalent);
}

// ==================== TAREA PRINCIPAL ====================

/**
 * Tarea principal para alertas de hidratación, HR elevada y calidad del aire
 */
void task_alerta_hidratacion(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO TAREAS DE ALERTAS DE SALUD ===");
    
    // Esperar estabilización del sistema
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    uint32_t loop_count = 0;
    
    while (1) {
        // FUNCIÓN 4: Verificar hidratación cada 10 minutos
        if (loop_count % 10 == 0) {
            check_hydration_levels();
        }
        
        // FUNCIÓN 5: Verificar HR elevada sin actividad cada ciclo
        check_elevated_hr_without_activity();
        
        // FUNCIÓN 6: Verificar calidad del aire cada 2 minutos
        if (loop_count % 2 == 0) {
            check_air_quality();
        }
        
        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_HIDRATACION_MS / 10)); // 1 minuto
    }
}

/**
 * Tarea para ritmo cardíaco elevado (mayor frecuencia de verificación)
 */
void task_ritmo_cardiaco_elevado(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO TAREA RITMO CARDÍACO ELEVADO ===");
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    while (1) {
        check_elevated_hr_without_activity();
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_SALUD_MS)); // 2 segundos
    }
}

/**
 * Tarea para ambiente mal ventilado
 */
void task_ambiente_mal_ventilado(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO TAREA AMBIENTE MAL VENTILADO ===");
    
    vTaskDelay(pdMS_TO_TICKS(4000));
    
    while (1) {
        check_air_quality();
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_AMBIENTE_MS)); // 2 minutos
    }
}