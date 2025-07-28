/**
 * FUNCIÓN 1: DETECCIÓN DE POSTURA ENCORVADA
 * 
 * Sensores: BMI270 + BMM150
 * Función: Detectar cuando los hombros se desplazan hacia adelante (encorvamiento)
 * Alerta: Mensaje "Postura encorvada detectada: ¿Necesitas un descanso?"
 * Acción: Vibración + notificación por Bluetooth
 * Umbral: Detectar inclinación >25° por más de 5 minutos
 */

#include "task_postura.h"
#include "sensor_bmi270.h"
#include "sensor_headers.h"
#include "service_headers.h"
#include "common_types.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "POSTURA_ENCORVADA";

// Variables para tracking de postura encorvada
static uint32_t bad_posture_start_time = 0;
static bool is_currently_bad_posture = false;
static uint32_t last_alert_time = 0;
static const uint32_t ALERT_COOLDOWN_MS = 15 * 60 * 1000; // 15 minutos entre alertas

// Buffer para suavizado de datos
#define POSTURE_BUFFER_SIZE 10
static float angle_buffer[POSTURE_BUFFER_SIZE];
static uint8_t buffer_index = 0;
static bool buffer_full = false;

// Calibración de postura inicial
static float baseline_forward_angle = 0.0f;
static bool is_calibrated = false;

/**
 * Calibrar la postura de referencia (postura correcta)
 */
static esp_err_t calibrate_posture_baseline(void) {
    ESP_LOGI(TAG, "Iniciando calibración de postura baseline...");
    
    float angle_sum = 0.0f;
    const int calibration_samples = 50;
    
    for (int i = 0; i < calibration_samples; i++) {
        imu_data_t imu_data;
        if (bmi270_read_data(&imu_data) == ESP_OK) {
            // Calcular ángulo de inclinación hacia adelante
            float forward_angle = atan2f(imu_data.accel_x, 
                                       sqrtf(imu_data.accel_y * imu_data.accel_y + 
                                            imu_data.accel_z * imu_data.accel_z)) * 180.0f / M_PI;
            angle_sum += forward_angle;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    baseline_forward_angle = angle_sum / calibration_samples;
    is_calibrated = true;
    
    ESP_LOGI(TAG, "Calibración completada. Ángulo baseline: %.2f°", baseline_forward_angle);
    return ESP_OK;
}

/**
 * Agregar valor al buffer de suavizado
 */
static void add_to_angle_buffer(float angle) {
    angle_buffer[buffer_index] = angle;
    buffer_index = (buffer_index + 1) % POSTURE_BUFFER_SIZE;
    if (buffer_index == 0) {
        buffer_full = true;
    }
}

/**
 * Obtener promedio del buffer de suavizado
 */
static float get_smoothed_angle(void) {
    if (!buffer_full && buffer_index == 0) {
        return 0.0f; // No hay datos suficientes
    }
    
    float sum = 0.0f;
    int count = buffer_full ? POSTURE_BUFFER_SIZE : buffer_index;
    
    for (int i = 0; i < count; i++) {
        sum += angle_buffer[i];
    }
    
    return sum / count;
}

/**
 * Detectar postura encorvada basada en ángulo de inclinación
 */
static bool detect_slouched_posture(const imu_data_t *imu_data, float *forward_angle_out) {
    // Calcular ángulo de inclinación hacia adelante (encorvamiento)
    float forward_angle = atan2f(imu_data->accel_x, 
                                sqrtf(imu_data->accel_y * imu_data->accel_y + 
                                     imu_data->accel_z * imu_data->accel_z)) * 180.0f / M_PI;
    
    // Aplicar suavizado
    add_to_angle_buffer(forward_angle);
    float smoothed_angle = get_smoothed_angle();
    
    if (forward_angle_out) {
        *forward_angle_out = smoothed_angle;
    }
    
    // Verificar si está calibrado
    if (!is_calibrated) {
        return false;
    }
    
    // Calcular desviación respecto al baseline
    float angle_deviation = smoothed_angle - baseline_forward_angle;
    
    // Detectar encorvamiento (inclinación hacia adelante mayor al umbral)
    bool is_slouched = angle_deviation > POSTURA_ENCORVADA_THRESHOLD_DEGREES;
    
    ESP_LOGD(TAG, "Ángulo forward: %.2f° | Baseline: %.2f° | Desviación: %.2f° | Encorvado: %s",
             smoothed_angle, baseline_forward_angle, angle_deviation, is_slouched ? "SÍ" : "NO");
    
    return is_slouched;
}

/**
 * Procesar alerta de postura encorvada
 */
static void process_slouched_posture_alert(float angle) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Evitar spam de alertas
    if (current_time - last_alert_time < ALERT_COOLDOWN_MS) {
        return;
    }
    
    ESP_LOGW(TAG, "ALERTA: Postura encorvada detectada (%.2f°) por más de %d minutos", 
             angle, POSTURA_ENCORVADA_TIME_MS / (60 * 1000));
    
    // Activar vibración específica para postura
    vibration_set_pattern(VIB_PATTERN_POSTURE_ALERT);
    
    // Enviar notificación por BLE
    if (ble_is_connected()) {
        char message[] = "Postura encorvada detectada: ¿Necesitas un descanso?";
        ble_notify_posture_alert(angle, message);
    }
    
    // Registrar evento en almacenamiento
    storage_log_event("POSTURE_SLOUCHED", "ALERT_TRIGGERED");
    
    // Actualizar tiempo de última alerta
    last_alert_time = current_time;
}

/**
 * Tarea principal de detección de postura encorvada
 */
void task_postura_encorvada(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO TAREA DETECCIÓN POSTURA ENCORVADA ===");
    
    // Esperar un poco para que el sistema se estabilice
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Calibrar postura de referencia
    calibrate_posture_baseline();
    
    imu_data_t imu_data;
    uint32_t loop_count = 0;
    
    while (1) {
        // Leer datos del IMU
        esp_err_t ret = bmi270_read_data(&imu_data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error leyendo BMI270: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_POSTURA_MS));
            continue;
        }
        
        // Detectar postura encorvada
        float current_angle;
        bool is_slouched = detect_slouched_posture(&imu_data, &current_angle);
        
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        if (is_slouched) {
            if (!is_currently_bad_posture) {
                // Inicio de postura mala
                bad_posture_start_time = current_time;
                is_currently_bad_posture = true;
                ESP_LOGI(TAG, "Inicio de postura encorvada detectado");
            } else {
                // Continúa en postura mala, verificar duración
                uint32_t bad_posture_duration = current_time - bad_posture_start_time;
                
                if (bad_posture_duration >= POSTURA_ENCORVADA_TIME_MS) {
                    // Activar alerta
                    process_slouched_posture_alert(current_angle);
                    
                    // Reset para evitar alertas continuas
                    bad_posture_start_time = current_time;
                }
            }
        } else {
            if (is_currently_bad_posture) {
                // Postura corregida
                uint32_t bad_posture_duration = current_time - bad_posture_start_time;
                ESP_LOGI(TAG, "Postura corregida después de %lu ms", bad_posture_duration);
                
                // Registrar corrección
                storage_log_event("POSTURE_CORRECTED", NULL);
                
                is_currently_bad_posture = false;
                bad_posture_start_time = 0;
            }
        }
        
        // Log periódico cada 30 segundos
        if (loop_count % 30 == 0) {
            ESP_LOGI(TAG, "Estado: ángulo=%.2f°, encorvado=%s, duración=%lums", 
                     current_angle, 
                     is_currently_bad_posture ? "SÍ" : "NO",
                     is_currently_bad_posture ? (current_time - bad_posture_start_time) : 0);
        }
        
        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_POSTURA_MS));
    }
}