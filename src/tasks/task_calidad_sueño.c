/**
 * FUNCIÓN 3: CALIDAD DE SUEÑO
 * 
 * Sensores: BMI270 + MAX30102 + MAX30205
 * Función: Monitorear movimientos nocturnos + SpO2 + temperatura corporal
 * Análisis: Detectar fases de sueño profundo vs. ligero
 * Reporte: "Dormiste X horas (Y% eficiencia)" al despertar
 */

#include "task_sueño.h"
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

static const char *TAG = "CALIDAD_SUEÑO";

// Estados de sueño
typedef enum {
    SLEEP_STATE_AWAKE,
    SLEEP_STATE_LIGHT_SLEEP,
    SLEEP_STATE_DEEP_SLEEP,
    SLEEP_STATE_REM_SLEEP
} sleep_state_t;

// Variables de estado del sueño
static sleep_state_t current_sleep_state = SLEEP_STATE_AWAKE;
static uint32_t sleep_start_time = 0;
static uint32_t wake_time = 0;
static bool is_sleep_session_active = false;

// Contadores de fases de sueño
static uint32_t deep_sleep_minutes = 0;
static uint32_t light_sleep_minutes = 0;
static uint32_t rem_sleep_minutes = 0;
static uint32_t awake_minutes = 0;

// Buffer para análisis de movimiento
#define MOVEMENT_BUFFER_SIZE 20
static float movement_buffer[MOVEMENT_BUFFER_SIZE];
static uint8_t movement_buffer_index = 0;
static bool movement_buffer_full = false;

// Buffer para análisis de HR
#define HR_BUFFER_SIZE 15
static uint16_t hr_buffer[HR_BUFFER_SIZE];
static uint8_t hr_buffer_index = 0;
static bool hr_buffer_full = false;

// Umbrales para detección de sueño
#define SLEEP_MOVEMENT_THRESHOLD       0.2f   // m/s² movimiento mínimo
#define SLEEP_HR_DROP_THRESHOLD        10     // BPM reducción HR
#define SLEEP_TEMP_DROP_THRESHOLD      0.5f   // °C reducción temperatura
#define SLEEP_SPO2_STABLE_THRESHOLD    2      // % variación SpO2
#define DEEP_SLEEP_MOVEMENT_THRESHOLD  0.1f   // m/s² para sueño profundo
#define REM_SLEEP_HR_VARIANCE          5      // BPM variabilidad para REM

/**
 * Agregar valor al buffer de movimiento
 */
static void add_movement_to_buffer(float movement) {
    movement_buffer[movement_buffer_index] = movement;
    movement_buffer_index = (movement_buffer_index + 1) % MOVEMENT_BUFFER_SIZE;
    if (movement_buffer_index == 0) {
        movement_buffer_full = true;
    }
}

/**
 * Agregar valor al buffer de HR
 */
static void add_hr_to_buffer(uint16_t hr) {
    hr_buffer[hr_buffer_index] = hr;
    hr_buffer_index = (hr_buffer_index + 1) % HR_BUFFER_SIZE;
    if (hr_buffer_index == 0) {
        hr_buffer_full = true;
    }
}

/**
 * Calcular promedio de movimiento
 */
static float get_average_movement(void) {
    int count = movement_buffer_full ? MOVEMENT_BUFFER_SIZE : movement_buffer_index;
    if (count == 0) return 0.0f;
    
    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += movement_buffer[i];
    }
    return sum / count;
}

/**
 * Calcular promedio y variabilidad de HR
 */
static void get_hr_stats(float *avg_hr, float *hr_variance) {
    int count = hr_buffer_full ? HR_BUFFER_SIZE : hr_buffer_index;
    if (count == 0) {
        *avg_hr = 0.0f;
        *hr_variance = 0.0f;
        return;
    }
    
    // Calcular promedio
    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += hr_buffer[i];
    }
    *avg_hr = sum / count;
    
    // Calcular varianza
    float variance_sum = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = hr_buffer[i] - *avg_hr;
        variance_sum += diff * diff;
    }
    *hr_variance = sqrtf(variance_sum / count);
}

/**
 * Detectar inicio de sueño
 */
static bool detect_sleep_onset(float movement, uint16_t heart_rate, float temperature, uint8_t spo2) {
    // Criterios para detectar inicio de sueño:
    // 1. Movimiento mínimo por período prolongado
    // 2. Reducción de frecuencia cardíaca
    // 3. Ligera reducción de temperatura corporal
    // 4. SpO2 estable
    
    static uint16_t baseline_hr = 0;
    static float baseline_temp = 0.0f;
    static uint32_t low_movement_start = 0;
    
    // Establecer baseline durante vigilia
    if (current_sleep_state == SLEEP_STATE_AWAKE) {
        if (baseline_hr == 0) baseline_hr = heart_rate;
        if (baseline_temp == 0.0f) baseline_temp = temperature;
        
        // Actualizar baseline gradualmente
        baseline_hr = baseline_hr * 0.95f + heart_rate * 0.05f;
        baseline_temp = baseline_temp * 0.95f + temperature * 0.05f;
    }
    
    // Verificar movimiento bajo
    bool low_movement = movement < SLEEP_MOVEMENT_THRESHOLD;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    if (low_movement) {
        if (low_movement_start == 0) {
            low_movement_start = current_time;
        }
    } else {
        low_movement_start = 0;
    }
    
    // Verificar otros criterios
    bool hr_dropped = (baseline_hr > 0) && (heart_rate < baseline_hr - SLEEP_HR_DROP_THRESHOLD);
    bool temp_dropped = (baseline_temp > 0.0f) && (temperature < baseline_temp - SLEEP_TEMP_DROP_THRESHOLD);
    bool prolonged_stillness = (low_movement_start > 0) && 
                              (current_time - low_movement_start > 10 * 60 * 1000); // 10 minutos
    
    // Detectar sueño si cumple múltiples criterios
    bool sleep_detected = prolonged_stillness && (hr_dropped || temp_dropped);
    
    if (sleep_detected) {
        ESP_LOGI(TAG, "Inicio de sueño detectado - Mov:%.3f, HR:%d->%d, Temp:%.1f->%.1f", 
                 movement, baseline_hr, heart_rate, baseline_temp, temperature);
    }
    
    return sleep_detected;
}

/**
 * Clasificar fase de sueño
 */
static sleep_state_t classify_sleep_phase(float movement, uint16_t heart_rate, float temperature, uint8_t spo2) {
    float avg_movement = get_average_movement();
    float avg_hr, hr_variance;
    get_hr_stats(&avg_hr, &hr_variance);
    
    // Sueño profundo: movimiento muy bajo, HR estable y bajo
    if (avg_movement < DEEP_SLEEP_MOVEMENT_THRESHOLD && hr_variance < 3.0f) {
        return SLEEP_STATE_DEEP_SLEEP;
    }
    
    // Sueño REM: movimiento bajo pero HR más variable
    if (avg_movement < SLEEP_MOVEMENT_THRESHOLD && hr_variance > REM_SLEEP_HR_VARIANCE) {
        return SLEEP_STATE_REM_SLEEP;
    }
    
    // Sueño ligero: movimiento bajo, HR moderadamente estable
    if (avg_movement < SLEEP_MOVEMENT_THRESHOLD * 2) {
        return SLEEP_STATE_LIGHT_SLEEP;
    }
    
    // Despierto: movimiento alto
    return SLEEP_STATE_AWAKE;
}

/**
 * Procesar cambio de fase de sueño
 */
static void process_sleep_phase_change(sleep_state_t new_state) {
    const char* state_names[] = {"DESPIERTO", "SUEÑO_LIGERO", "SUEÑO_PROFUNDO", "SUEÑO_REM"};
    
    ESP_LOGI(TAG, "Cambio de fase: %s -> %s", 
             state_names[current_sleep_state], 
             state_names[new_state]);
    
    // Registrar cambio
    storage_log_sleep_phase_change(current_sleep_state, new_state);
    
    current_sleep_state = new_state;
}

/**
 * Detectar despertar
 */
static bool detect_wake_up(float movement, uint16_t heart_rate) {
    // Despertar detectado por aumento significativo en movimiento y HR
    float avg_movement = get_average_movement();
    float avg_hr, hr_variance;
    get_hr_stats(&avg_hr, &hr_variance);
    
    bool high_movement = movement > SLEEP_MOVEMENT_THRESHOLD * 3;
    bool hr_increased = heart_rate > avg_hr + 15;
    
    return high_movement && hr_increased;
}

/**
 * Generar reporte de sueño
 */
static void generate_sleep_report(void) {
    if (!is_sleep_session_active || sleep_start_time == 0) {
        return;
    }
    
    uint32_t total_sleep_time = wake_time - sleep_start_time;
    uint32_t total_hours = total_sleep_time / (60 * 60 * 1000);
    uint32_t total_minutes = (total_sleep_time % (60 * 60 * 1000)) / (60 * 1000);
    
    // Calcular eficiencia de sueño
    uint32_t total_actual_sleep = deep_sleep_minutes + light_sleep_minutes + rem_sleep_minutes;
    uint32_t total_time_in_bed = total_sleep_time / (60 * 1000);
    uint8_t sleep_efficiency = (total_actual_sleep * 100) / total_time_in_bed;
    
    ESP_LOGI(TAG, "=== REPORTE DE SUEÑO ===");
    ESP_LOGI(TAG, "Tiempo total en cama: %luh %lum", total_hours, total_minutes);
    ESP_LOGI(TAG, "Sueño profundo: %lu min", deep_sleep_minutes);
    ESP_LOGI(TAG, "Sueño ligero: %lu min", light_sleep_minutes);
    ESP_LOGI(TAG, "Sueño REM: %lu min", rem_sleep_minutes);
    ESP_LOGI(TAG, "Tiempo despierto: %lu min", awake_minutes);
    ESP_LOGI(TAG, "Eficiencia del sueño: %d%%", sleep_efficiency);
    
    // Enviar reporte por BLE
    if (ble_is_connected()) {
        char message[128];
        snprintf(message, sizeof(message), 
                "Dormiste %luh %lum (%d%% eficiencia)", 
                total_hours, total_minutes, sleep_efficiency);
        ble_notify_sleep_report(total_sleep_time, sleep_efficiency, message);
    }
    
    // Guardar en almacenamiento
    sleep_data_t sleep_data = {
        .is_sleeping = false,
        .sleep_quality = sleep_efficiency,
        .deep_sleep_minutes = deep_sleep_minutes,
        .light_sleep_minutes = light_sleep_minutes,
        .rem_sleep_minutes = rem_sleep_minutes,
        .sleep_efficiency = sleep_efficiency,
        .bedtime = sleep_start_time,
        .wake_time = wake_time
    };
    storage_save_sleep_data(&sleep_data);
    
    // Reset contadores
    deep_sleep_minutes = light_sleep_minutes = rem_sleep_minutes = awake_minutes = 0;
    is_sleep_session_active = false;
}

/**
 * Tarea principal de monitoreo de calidad de sueño
 */
void task_calidad_sueño(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO TAREA CALIDAD DE SUEÑO ===");
    
    // Esperar estabilización
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    imu_data_t imu_data;
    uint32_t loop_count = 0;
    uint32_t last_phase_time = 0;
    
    while (1) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // === Leer sensores ===
        esp_err_t ret = bmi270_read_data(&imu_data);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_SUEÑO_MS));
            continue;
        }
        
        // Calcular movimiento total
        float movement = vector3_magnitude(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        add_movement_to_buffer(movement);
        
        // Leer datos de salud (menos frecuente)
        uint16_t heart_rate = 70; // Placeholder - obtener de MAX30102
        uint8_t spo2 = 98;        // Placeholder - obtener de MAX30102
        float temperature = 36.5f; // Placeholder - obtener de MAX30205
        
        if (loop_count % 3 == 0) { // Cada 90 segundos
            max30102_sample_t ppg_samples[8];
            size_t sample_count;
            if (max30102_read_fifo(ppg_samples, &sample_count) == ESP_OK && sample_count > 0) {
                heart_rate = max30102_calculate_heart_rate(ppg_samples, sample_count);
                spo2 = max30102_calculate_spo2(ppg_samples, sample_count);
            }
            temperature = max30205_read_temperature();
        }
        
        add_hr_to_buffer(heart_rate);
        
        // === Análisis del estado de sueño ===
        if (current_sleep_state == SLEEP_STATE_AWAKE) {
            // Detectar inicio de sueño
            if (detect_sleep_onset(movement, heart_rate, temperature, spo2)) {
                sleep_start_time = current_time;
                is_sleep_session_active = true;
                process_sleep_phase_change(SLEEP_STATE_LIGHT_SLEEP);
                last_phase_time = current_time;
            }
        } else {
            // Ya está durmiendo - analizar fase
            if (detect_wake_up(movement, heart_rate)) {
                // Despertar detectado
                wake_time = current_time;
                process_sleep_phase_change(SLEEP_STATE_AWAKE);
                generate_sleep_report();
            } else {
                // Clasificar fase de sueño
                sleep_state_t new_phase = classify_sleep_phase(movement, heart_rate, temperature, spo2);
                
                if (new_phase != current_sleep_state) {
                    // Actualizar contadores de tiempo en fase anterior
                    uint32_t phase_duration = (current_time - last_phase_time) / (60 * 1000); // minutos
                    
                    switch (current_sleep_state) {
                        case SLEEP_STATE_DEEP_SLEEP:
                            deep_sleep_minutes += phase_duration;
                            break;
                        case SLEEP_STATE_LIGHT_SLEEP:
                            light_sleep_minutes += phase_duration;
                            break;
                        case SLEEP_STATE_REM_SLEEP:
                            rem_sleep_minutes += phase_duration;
                            break;
                        case SLEEP_STATE_AWAKE:
                            awake_minutes += phase_duration;
                            break;
                    }
                    
                    process_sleep_phase_change(new_phase);
                    last_phase_time = current_time;
                }
            }
        }
        
        // Log periódico cada 5 minutos
        if (loop_count % 10 == 0) {
            ESP_LOGI(TAG, "Estado: %s, Mov:%.3f, HR:%d, Temp:%.1f°C", 
                     current_sleep_state == SLEEP_STATE_AWAKE ? "DESPIERTO" :
                     current_sleep_state == SLEEP_STATE_LIGHT_SLEEP ? "SUEÑO_LIGERO" :
                     current_sleep_state == SLEEP_STATE_DEEP_SLEEP ? "SUEÑO_PROFUNDO" : "REM",
                     movement, heart_rate, temperature);
        }
        
        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_SUEÑO_MS));
    }
}