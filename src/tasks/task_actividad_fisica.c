/**
 * FUNCIONES 7, 8, 9: ACTIVIDAD FÍSICA Y POSTURA GENERAL
 * 
 * FUNCIÓN 7: CONTADOR DE PASOS Y MOVIMIENTO
 * Sensores: BMI270 + BMM150
 * Función: Algoritmo de conteo de pasos usando acelerómetro + magnetómetro para orientación
 * Registro: Pasos diarios, distancia estimada, calorías
 * Mostrar: Estadísticas en la app
 * 
 * FUNCIÓN 8: DETECCIÓN DE SEDENTARISMO MEJORADA
 * Sensores: BMI270 + BMM150
 * Función: Detectar ausencia de movimiento significativo
 * Umbral: >1 hora sin movimiento (aceleración <0.1g)
 * Alerta: Vibración progresiva (suave → fuerte)
 * Mensaje: "Llevas más de 1 hora sin moverte. ¡Es hora de activarse!"
 * 
 * FUNCIÓN 9: ALERTA DE POSTURA INCORRECTA GENERAL
 * Sensores: BMI270 + BMM150
 * Función: Detectar cualquier desviación de la postura calibrada
 * Incluye: Cabeza hacia adelante, hombros caídos, espalda encorvada
 * Acción: Vibración inmediata + notificación
 * Mensaje: "Corrige tu postura"
 */

#include "task_pasos.h"
#include "task_sedentarismo.h"
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

static const char *TAG = "ACTIVIDAD_FISICA";

// ==================== FUNCIÓN 7: CONTADOR DE PASOS ====================

// Variables de conteo de pasos
static uint32_t daily_steps = 0;
static float total_distance_km = 0.0f;
static uint16_t estimated_calories = 0;
static uint32_t last_step_time = 0;
static uint32_t day_start_timestamp = 0;

// Buffer para detección de pasos
#define STEP_BUFFER_SIZE 10
static float accel_magnitude_buffer[STEP_BUFFER_SIZE];
static uint8_t step_buffer_index = 0;
static bool step_buffer_full = false;

// Parámetros del algoritmo de pasos
#define STEP_THRESHOLD_MIN      1.2f    // m/s² mínimo para paso
#define STEP_THRESHOLD_MAX      3.5f    // m/s² máximo para paso
#define STEP_MIN_INTERVAL_MS    300     // Mínimo 300ms entre pasos
#define STEP_MAX_INTERVAL_MS    2000    // Máximo 2s entre pasos
#define STEP_LENGTH_CM          75      // Longitud de paso promedio en cm

/**
 * Agregar valor al buffer de aceleración
 */
static void add_accel_to_buffer(float accel_magnitude) {
    accel_magnitude_buffer[step_buffer_index] = accel_magnitude;
    step_buffer_index = (step_buffer_index + 1) % STEP_BUFFER_SIZE;
    if (step_buffer_index == 0) {
        step_buffer_full = true;
    }
}

/**
 * Detectar pico de aceleración (paso)
 */
static bool detect_step_peak(float current_accel) {
    if (!step_buffer_full && step_buffer_index < 3) {
        return false; // No hay suficientes datos
    }
    
    int buffer_count = step_buffer_full ? STEP_BUFFER_SIZE : step_buffer_index;
    int center_index = (step_buffer_index - 2 + STEP_BUFFER_SIZE) % STEP_BUFFER_SIZE;
    float center_value = accel_magnitude_buffer[center_index];
    
    // Verificar que el valor central sea un pico
    bool is_peak = true;
    for (int i = 0; i < buffer_count; i++) {
        if (i == center_index) continue;
        if (accel_magnitude_buffer[i] >= center_value) {
            is_peak = false;
            break;
        }
    }
    
    // Verificar que esté en el rango de pasos
    bool valid_magnitude = (center_value >= STEP_THRESHOLD_MIN) && 
                          (center_value <= STEP_THRESHOLD_MAX);
    
    return is_peak && valid_magnitude;
}

/**
 * FUNCIÓN 7: Procesar detección de paso
 */
static void process_step_detected(void) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Verificar intervalo mínimo entre pasos
    if (last_step_time > 0) {
        uint32_t step_interval = current_time - last_step_time;
        if (step_interval < STEP_MIN_INTERVAL_MS || step_interval > STEP_MAX_INTERVAL_MS) {
            return; // Intervalo inválido
        }
    }
    
    // Incrementar contador de pasos
    daily_steps++;
    last_step_time = current_time;
    
    // Calcular distancia (longitud de paso * número de pasos)
    total_distance_km = (daily_steps * STEP_LENGTH_CM) / 100000.0f; // Convertir a km
    
    // Estimar calorías (aproximación: 0.04 calorías por paso)
    estimated_calories = daily_steps * 0.04f;
    
    ESP_LOGD(TAG, "Paso detectado #%lu - Distancia: %.2f km, Calorías: %d", 
             daily_steps, total_distance_km, estimated_calories);
    
    // Enviar actualización por BLE cada 10 pasos
    if (daily_steps % 10 == 0 && ble_is_connected()) {
        step_data_t step_data = {
            .daily_steps = daily_steps,
            .distance_km = total_distance_km,
            .calories_burned = estimated_calories,
            .avg_step_rate = 0, // Calcular si es necesario
            .timestamp = current_time
        };
        ble_notify_step_data(&step_data);
    }
}

/**
 * Reset diario de contadores
 */
static void reset_daily_counters(void) {
    // Guardar datos del día anterior
    if (daily_steps > 0) {
        step_data_t daily_summary = {
            .daily_steps = daily_steps,
            .distance_km = total_distance_km,
            .calories_burned = estimated_calories,
            .avg_step_rate = 0,
            .timestamp = day_start_timestamp
        };
        storage_save_daily_steps(&daily_summary);
        
        ESP_LOGI(TAG, "Resumen diario: %lu pasos, %.2f km, %d calorías", 
                 daily_steps, total_distance_km, estimated_calories);
    }
    
    // Reset contadores
    daily_steps = 0;
    total_distance_km = 0.0f;
    estimated_calories = 0;
    day_start_timestamp = esp_timer_get_time() / 1000;
}

// ==================== FUNCIÓN 8: DETECCIÓN DE SEDENTARISMO ====================

// Variables de sedentarismo
static uint32_t last_significant_movement = 0;
static uint32_t sedentary_alert_count = 0;
static uint32_t last_sedentary_alert = 0;
static const uint32_t SEDENTARY_ALERT_COOLDOWN_MS = 15 * 60 * 1000; // 15 minutos

/**
 * FUNCIÓN 8: Detectar sedentarismo
 */
static void check_sedentary_behavior(float movement_magnitude) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Detectar movimiento significativo
    bool significant_movement = movement_magnitude > SEDENTARISMO_ACCEL_THRESHOLD;
    
    if (significant_movement) {
        last_significant_movement = current_time;
        sedentary_alert_count = 0; // Reset contador de alertas
        return;
    }
    
    // Verificar tiempo sin movimiento
    uint32_t time_without_movement = current_time - last_significant_movement;
    bool prolonged_sedentary = time_without_movement >= SEDENTARISMO_THRESHOLD_TIME_MS;
    bool can_alert = (current_time - last_sedentary_alert) > SEDENTARY_ALERT_COOLDOWN_MS;
    
    if (prolonged_sedentary && can_alert) {
        sedentary_alert_count++;
        
        ESP_LOGW(TAG, "ALERTA SEDENTARISMO: %lu minutos sin movimiento (alerta #%lu)", 
                 time_without_movement / (60 * 1000), sedentary_alert_count);
        
        // Vibración progresiva: más intensa con cada alerta
        vibration_pattern_t pattern = (sedentary_alert_count <= 2) ? 
                                     VIB_PATTERN_SEDENTARISM_SOFT : 
                                     VIB_PATTERN_SEDENTARISM_STRONG;
        vibration_set_pattern(pattern);
        
        // Enviar notificación por BLE
        if (ble_is_connected()) {
            char message[100];
            snprintf(message, sizeof(message), 
                    "Llevas más de %lu minutos sin moverte. ¡Es hora de activarse!",
                    time_without_movement / (60 * 1000));
            ble_notify_sedentary_alert(time_without_movement, sedentary_alert_count, message);
        }
        
        // Registrar evento
        storage_log_event("SEDENTARY_ALERT", sedentary_alert_count > 2 ? "STRONG" : "SOFT");
        
        last_sedentary_alert = current_time;
    }
    
    ESP_LOGD(TAG, "Sedentarismo: Sin movimiento por %lu min", 
             time_without_movement / (60 * 1000));
}

// ==================== FUNCIÓN 9: POSTURA INCORRECTA GENERAL ====================

// Variables de postura general
static float baseline_tilt_x = 0.0f;
static float baseline_tilt_y = 0.0f;
static float baseline_tilt_z = 0.0f;
static bool posture_calibrated = false;
static uint32_t last_posture_alert = 0;
static const uint32_t POSTURE_ALERT_COOLDOWN_MS = 5 * 60 * 1000; // 5 minutos

// Umbrales para postura general
#define POSTURE_DEVIATION_THRESHOLD_XY  15.0f  // Grados desviación X/Y
#define POSTURE_DEVIATION_THRESHOLD_Z   20.0f  // Grados desviación Z (rotación)

/**
 * Calibrar postura general de referencia
 */
static esp_err_t calibrate_general_posture(void) {
    ESP_LOGI(TAG, "Calibrando postura general de referencia...");
    
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int valid_samples = 0;
    const int total_samples = 30;
    
    for (int i = 0; i < total_samples; i++) {
        imu_data_t imu_data;
        if (bmi270_read_data(&imu_data) == ESP_OK) {
            // Calcular ángulos de inclinación
            float tilt_x = atan2f(imu_data.accel_y, imu_data.accel_z) * 180.0f / M_PI;
            float tilt_y = atan2f(imu_data.accel_x, imu_data.accel_z) * 180.0f / M_PI;
            float tilt_z = atan2f(imu_data.accel_x, imu_data.accel_y) * 180.0f / M_PI;
            
            sum_x += tilt_x;
            sum_y += tilt_y;
            sum_z += tilt_z;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (valid_samples > total_samples / 2) {
        baseline_tilt_x = sum_x / valid_samples;
        baseline_tilt_y = sum_y / valid_samples;
        baseline_tilt_z = sum_z / valid_samples;
        posture_calibrated = true;
        
        ESP_LOGI(TAG, "Postura calibrada - X: %.1f°, Y: %.1f°, Z: %.1f°", 
                 baseline_tilt_x, baseline_tilt_y, baseline_tilt_z);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Error en calibración de postura");
    return ESP_FAIL;
}

/**
 * FUNCIÓN 9: Detectar postura incorrecta general
 */
static void check_general_posture(const imu_data_t *imu_data) {
    if (!posture_calibrated) {
        return;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Calcular ángulos actuales
    float current_tilt_x = atan2f(imu_data->accel_y, imu_data->accel_z) * 180.0f / M_PI;
    float current_tilt_y = atan2f(imu_data->accel_x, imu_data->accel_z) * 180.0f / M_PI;
    float current_tilt_z = atan2f(imu_data->accel_x, imu_data->accel_y) * 180.0f / M_PI;
    
    // Calcular desviaciones
    float deviation_x = fabsf(current_tilt_x - baseline_tilt_x);
    float deviation_y = fabsf(current_tilt_y - baseline_tilt_y);
    float deviation_z = fabsf(current_tilt_z - baseline_tilt_z);
    
    // Detectar postura incorrecta
    bool bad_posture_xy = (deviation_x > POSTURE_DEVIATION_THRESHOLD_XY) || 
                         (deviation_y > POSTURE_DEVIATION_THRESHOLD_XY);
    bool bad_posture_z = deviation_z > POSTURE_DEVIATION_THRESHOLD_Z;
    bool bad_posture = bad_posture_xy || bad_posture_z;
    
    bool can_alert = (current_time - last_posture_alert) > POSTURE_ALERT_COOLDOWN_MS;
    
    if (bad_posture && can_alert) {
        ESP_LOGW(TAG, "POSTURA INCORRECTA: X=%.1f°(±%.1f), Y=%.1f°(±%.1f), Z=%.1f°(±%.1f)",
                 current_tilt_x, deviation_x, current_tilt_y, deviation_y, 
                 current_tilt_z, deviation_z);
        
        // Activar vibración inmediata
        vibration_set_pattern(VIB_PATTERN_POSTURE_CORRECTION);
        
        // Enviar notificación por BLE
        if (ble_is_connected()) {
            char message[] = "Corrige tu postura";
            char details[100];
            snprintf(details, sizeof(details), 
                    "Desviaciones: X±%.0f° Y±%.0f° Z±%.0f°", 
                    deviation_x, deviation_y, deviation_z);
            ble_notify_posture_correction_alert(deviation_x, deviation_y, deviation_z, message);
        }
        
        // Registrar evento
        storage_log_event("POSTURE_GENERAL_BAD", "IMMEDIATE_CORRECTION");
        
        last_posture_alert = current_time;
    }
    
    ESP_LOGD(TAG, "Postura: X=%.1f°(±%.1f), Y=%.1f°(±%.1f), Z=%.1f°(±%.1f), Mala=%s",
             current_tilt_x, deviation_x, current_tilt_y, deviation_y, 
             current_tilt_z, deviation_z, bad_posture ? "SÍ" : "NO");
}

// ==================== TAREAS PRINCIPALES ====================

/**
 * FUNCIÓN 7: Tarea de contador de pasos
 */
void task_contador_pasos(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO TAREA CONTADOR DE PASOS ===");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Inicializar día
    day_start_timestamp = esp_timer_get_time() / 1000;
    last_significant_movement = day_start_timestamp;
    
    imu_data_t imu_data;
    uint32_t last_day_check = 0;
    
    while (1) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // Leer datos del IMU
        esp_err_t ret = bmi270_read_data(&imu_data);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_PASOS_MS));
            continue;
        }
        
        // Calcular magnitud de aceleración
        float accel_magnitude = vector3_magnitude(imu_data.accel_x, 
                                                imu_data.accel_y, 
                                                imu_data.accel_z);
        
        // Agregar al buffer y detectar pasos
        add_accel_to_buffer(accel_magnitude);
        if (detect_step_peak(accel_magnitude)) {
            process_step_detected();
        }
        
        // Verificar cambio de día (reset a medianoche)
        if (current_time - last_day_check > 60000) { // Check cada minuto
            time_t now = current_time / 1000;
            struct tm *timeinfo = localtime(&now);
            
            if (timeinfo->tm_hour == 0 && timeinfo->tm_min == 0) {
                reset_daily_counters();
            }
            last_day_check = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_PASOS_MS));
    }
}

/**
 * FUNCIÓN 8: Tarea de detección de sedentarismo
 */
void task_sedentarismo_mejorado(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO TAREA SEDENTARISMO MEJORADO ===");
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Inicializar tiempo de último movimiento
    last_significant_movement = esp_timer_get_time() / 1000;
    
    imu_data_t imu_data;
    
    while (1) {
        // Leer datos del IMU
        esp_err_t ret = bmi270_read_data(&imu_data);
        if (ret == ESP_OK) {
            float movement_magnitude = vector3_magnitude(imu_data.accel_x, 
                                                       imu_data.accel_y, 
                                                       imu_data.accel_z);
            check_sedentary_behavior(movement_magnitude);
        }
        
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_SEDENTARISMO_MS));
    }
}

/**
 * FUNCIÓN 9: Tarea de postura incorrecta general
 */
void task_postura_incorrecta_general(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO TAREA POSTURA INCORRECTA GENERAL ===");
    
    vTaskDelay(pdMS_TO_TICKS(4000));
    
    // Calibrar postura de referencia
    calibrate_general_posture();
    
    imu_data_t imu_data;
    
    while (1) {
        // Leer datos del IMU
        esp_err_t ret = bmi270_read_data(&imu_data);
        if (ret == ESP_OK) {
            check_general_posture(&imu_data);
        }
        
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_POSTURA_MS));
    }
}