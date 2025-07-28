/**
 * FUNCIÓN 2: DETECCIÓN DE ESTRÉS
 * 
 * Sensores: MAX30102 + BME688 + BMM150
 * Función: Analizar variabilidad cardíaca (HRV) + sudoración + tensión muscular
 * Criterios: Frecuencia cardíaca irregular + cambios en humedad ambiente + movimientos tensos
 * Alerta: "¿Estás estresado?" con sugerencia de respiración profunda
 * Acción: Vibración suave + notificación
 */

#include "task_salud.h"
#include "sensor_max30102.h"
#include "sensor_headers.h"
#include "service_headers.h"
#include "common_types.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "DETECCION_ESTRES";

// Buffer para análisis HRV
#define RR_INTERVAL_BUFFER_SIZE 30
static float rr_intervals[RR_INTERVAL_BUFFER_SIZE];
static uint8_t rr_buffer_index = 0;
static bool rr_buffer_full = false;

// Variables de estado
static float baseline_humidity = 0.0f;
static bool humidity_baseline_set = false;
static uint32_t last_stress_alert = 0;
static const uint32_t STRESS_ALERT_COOLDOWN_MS = 20 * 60 * 1000; // 20 minutos

// Umbrales de estrés
#define STRESS_HRV_THRESHOLD        30.0f  // ms RMSSD
#define STRESS_HR_VARIANCE_THRESHOLD 15.0f  // BPM variación
#define STRESS_HUMIDITY_CHANGE      3.0f   // % cambio en humedad local
#define STRESS_MOVEMENT_THRESHOLD   2.0f   // Umbral para movimientos tensos

/**
 * Calcular RMSSD (Root Mean Square of Successive Differences) para HRV
 */
static float calculate_rmssd(const float *rr_intervals, size_t count) {
    if (count < 2) return 0.0f;
    
    float sum_squared_diff = 0.0f;
    
    for (int i = 1; i < count; i++) {
        float diff = rr_intervals[i] - rr_intervals[i-1];
        sum_squared_diff += diff * diff;
    }
    
    float rmssd = sqrtf(sum_squared_diff / (count - 1));
    return rmssd;
}

/**
 * Convertir intervalos RR a índice de estrés (0-100)
 */
static float calculate_stress_index(float rmssd) {
    // Menor RMSSD = mayor estrés
    // Normalizar a escala 0-100
    float stress_index = 100.0f - (rmssd / 50.0f) * 100.0f;
    return clamp_float(stress_index, 0.0f, 100.0f);
}

/**
 * Agregar intervalo RR al buffer
 */
static void add_rr_interval(float rr_interval) {
    rr_intervals[rr_buffer_index] = rr_interval;
    rr_buffer_index = (rr_buffer_index + 1) % RR_INTERVAL_BUFFER_SIZE;
    
    if (rr_buffer_index == 0) {
        rr_buffer_full = true;
    }
}

/**
 * Detectar cambios en sudoración (humedad local)
 */
static bool detect_sweating_changes(float current_humidity) {
    if (!humidity_baseline_set) {
        baseline_humidity = current_humidity;
        humidity_baseline_set = true;
        return false;
    }
    
    float humidity_change = current_humidity - baseline_humidity;
    
    // Actualizar baseline gradualmente
    baseline_humidity = baseline_humidity * 0.99f + current_humidity * 0.01f;
    
    return fabsf(humidity_change) > STRESS_HUMIDITY_CHANGE;
}

/**
 * Detectar movimientos tensos usando magnetómetro
 */
static bool detect_tense_movements(const imu_data_t *imu_data) {
    // Calcular magnitud de aceleración
    float accel_magnitude = vector3_magnitude(imu_data->accel_x, 
                                            imu_data->accel_y, 
                                            imu_data->accel_z);
    
    // Calcular magnitud de rotación
    float gyro_magnitude = vector3_magnitude(imu_data->gyro_x, 
                                           imu_data->gyro_y, 
                                           imu_data->gyro_z);
    
    // Movimientos tensos = alta aceleración + alta rotación
    bool tense_movement = (accel_magnitude > STRESS_MOVEMENT_THRESHOLD) && 
                         (gyro_magnitude > STRESS_MOVEMENT_THRESHOLD);
    
    return tense_movement;
}

/**
 * Procesar alerta de estrés
 */
static void process_stress_alert(float stress_index, float hrv, bool sweating, bool tense_movements) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Evitar spam de alertas
    if (current_time - last_stress_alert < STRESS_ALERT_COOLDOWN_MS) {
        return;
    }
    
    ESP_LOGW(TAG, "ALERTA DE ESTRÉS: Índice=%.1f, HRV=%.1f ms, Sudoración=%s, Movimientos Tensos=%s",
             stress_index, hrv, sweating ? "SÍ" : "NO", tense_movements ? "SÍ" : "NO");
    
    // Activar vibración para respiración guiada (patrón rítmico)
    vibration_set_pattern(VIB_PATTERN_STRESS_BREATHING);
    
    // Enviar notificación por BLE
    if (ble_is_connected()) {
        char message[] = "¿Estás estresado? Intenta respirar profundamente";
        ble_notify_stress_alert(stress_index, message);
    }
    
    // Registrar evento
    storage_log_event("STRESS_DETECTED", "BREATHING_GUIDANCE_TRIGGERED");
    
    last_stress_alert = current_time;
}

/**
 * Análisis multimodal de estrés
 */
static bool analyze_stress_indicators(health_metrics_t *health, environment_data_t *env, imu_data_t *imu) {
    int stress_indicators = 0;
    
    // 1. Análisis HRV
    bool low_hrv = health->hrv < STRESS_HRV_THRESHOLD;
    if (low_hrv) {
        stress_indicators++;
        ESP_LOGD(TAG, "Indicador estrés: HRV baja (%.1f ms)", health->hrv);
    }
    
    // 2. Variabilidad de frecuencia cardíaca
    static uint16_t prev_hr = 0;
    bool high_hr_variance = false;
    if (prev_hr > 0) {
        float hr_diff = fabsf((float)health->heart_rate - (float)prev_hr);
        high_hr_variance = hr_diff > STRESS_HR_VARIANCE_THRESHOLD;
        if (high_hr_variance) {
            stress_indicators++;
            ESP_LOGD(TAG, "Indicador estrés: Alta variabilidad HR (%.1f BPM)", hr_diff);
        }
    }
    prev_hr = health->heart_rate;
    
    // 3. Cambios en sudoración (humedad local)
    bool sweating_detected = detect_sweating_changes(env->humidity);
    if (sweating_detected) {
        stress_indicators++;
        ESP_LOGD(TAG, "Indicador estrés: Cambio en sudoración");
    }
    
    // 4. Movimientos tensos
    bool tense_movements = detect_tense_movements(imu);
    if (tense_movements) {
        stress_indicators++;
        ESP_LOGD(TAG, "Indicador estrés: Movimientos tensos detectados");
    }
    
    // Determinar si hay estrés (2 o más indicadores)
    bool is_stressed = stress_indicators >= 2;
    
    if (is_stressed) {
        float stress_index = calculate_stress_index(health->hrv);
        process_stress_alert(stress_index, health->hrv, sweating_detected, tense_movements);
    }
    
    return is_stressed;
}

/**
 * Tarea principal de detección de estrés
 */
void task_deteccion_estres(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO TAREA DETECCIÓN DE ESTRÉS ===");
    
    // Esperar estabilización del sistema
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Variables para almacenar datos de sensores
    health_metrics_t health_data = {0};
    environment_data_t env_data = {0};
    imu_data_t imu_data = {0};
    
    uint32_t loop_count = 0;
    uint32_t last_env_read = 0;
    
    while (1) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // === Leer datos de salud (frecuente) ===
        max30102_sample_t ppg_samples[16];
        size_t sample_count;
        
        if (max30102_read_fifo(ppg_samples, &sample_count) == ESP_OK && sample_count > 0) {
            // Calcular métricas de salud
            health_data.heart_rate = max30102_calculate_heart_rate(ppg_samples, sample_count);
            health_data.spo2 = max30102_calculate_spo2(ppg_samples, sample_count);
            
            // Calcular intervalos RR y HRV
            for (int i = 1; i < sample_count; i++) {
                // Detectar picos y calcular intervalos RR
                // Simplificado: usar diferencias de tiempo entre muestras
                float rr_interval = 60000.0f / health_data.heart_rate; // ms aproximado
                add_rr_interval(rr_interval);
            }
            
            // Calcular HRV
            int count = rr_buffer_full ? RR_INTERVAL_BUFFER_SIZE : rr_buffer_index;
            if (count > 5) {
                health_data.hrv = calculate_rmssd(rr_intervals, count);
                health_data.stress_index = calculate_stress_index(health_data.hrv);
            }
            
            health_data.timestamp = current_time;
        }
        
        // === Leer datos ambientales (menos frecuente) ===
        if (current_time - last_env_read > 30000) { // Cada 30 segundos
            if (bme688_read_all(&env_data) == ESP_OK) {
                last_env_read = current_time;
            }
        }
        
        // === Leer datos de movimiento ===
        bmm150_read_data(&imu_data);
        
        // === Análisis de estrés ===
        if (health_data.heart_rate > 0 && env_data.timestamp > 0) {
            bool is_stressed = analyze_stress_indicators(&health_data, &env_data, &imu_data);
            
            // Log periódico cada 2 minutos
            if (loop_count % 60 == 0) {
                ESP_LOGI(TAG, "Estado: HR=%d BPM, HRV=%.1f ms, Estrés=%.1f%%, Estresado=%s",
                         health_data.heart_rate, 
                         health_data.hrv,
                         health_data.stress_index,
                         is_stressed ? "SÍ" : "NO");
            }
        }
        
        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_SALUD_MS));
    }
}