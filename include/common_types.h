#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "esp_err.h"

// ==================== CONSTANTES GENERALES ====================

// Umbrales para las 9 funciones específicas
#define POSTURA_ENCORVADA_THRESHOLD_DEGREES    25.0f
#define POSTURA_ENCORVADA_TIME_MS              (5 * 60 * 1000)  // 5 minutos
#define SEDENTARISMO_THRESHOLD_TIME_MS         (60 * 60 * 1000) // 1 hora
#define SEDENTARISMO_ACCEL_THRESHOLD           0.1f             // 0.1g
#define RITMO_CARDIACO_ELEVADO_THRESHOLD       100              // BPM
#define RITMO_CARDIACO_SIN_ACTIVIDAD_TIME_MS   (5 * 60 * 1000)  // 5 minutos
#define HIDRATACION_BAJA_THRESHOLD             40               // %
#define AMBIENTE_MAL_VENTILADO_THRESHOLD       50               // kΩ resistencia gas
#define HRV_ESTRES_THRESHOLD                   30.0f            // ms

// Intervalos de verificación
#define CHECK_INTERVAL_POSTURA_MS              1000             // 1 segundo
#define CHECK_INTERVAL_SALUD_MS                2000             // 2 segundos
#define CHECK_INTERVAL_AMBIENTE_MS             (2 * 60 * 1000)  // 2 minutos
#define CHECK_INTERVAL_HIDRATACION_MS          (10 * 60 * 1000) // 10 minutos
#define CHECK_INTERVAL_PASOS_MS                100              // 100ms
#define CHECK_INTERVAL_SEDENTARISMO_MS         (5 * 60 * 1000)  // 5 minutos
#define CHECK_INTERVAL_SUEÑO_MS                (30 * 1000)      // 30 segundos

// ==================== ESTRUCTURAS DE DATOS ====================

// Datos del IMU (BMI270 + BMM150)
typedef struct {
    float accel_x, accel_y, accel_z;  // m/s²
    float gyro_x, gyro_y, gyro_z;     // rad/s
    float mag_x, mag_y, mag_z;        // µT
    uint32_t timestamp;
} imu_data_t;

// Datos de postura
typedef struct {
    float tilt_angle_x;           // Inclinación en X (encorvamiento)
    float tilt_angle_y;           // Inclinación en Y
    float tilt_angle_z;           // Rotación en Z
    float movement_variance;      // Varianza de movimiento
    bool is_bad_posture;          // Postura incorrecta detectada
    uint32_t bad_posture_duration; // Duración en ms
    uint32_t timestamp;
} posture_data_t;

// Datos de salud
typedef struct {
    uint16_t heart_rate;          // BPM
    uint8_t spo2;                // %
    float body_temperature;       // °C
    float hrv;                   // ms (RMSSD)
    float stress_index;          // 0-100
    uint8_t hydration_level;     // 0-100%
    bool flow_state;             // Estado de concentración
    uint32_t timestamp;
} health_metrics_t;

// Datos ambientales
typedef struct {
    float temperature;           // °C
    float humidity;             // %
    float pressure;             // hPa
    uint16_t gas_resistance;    // kΩ
    uint16_t iaq_index;         // 0-500
    float co2_equivalent;       // ppm
    float voc_equivalent;       // ppm
    uint32_t timestamp;
} environment_data_t;

// Datos de hidratación
typedef struct {
    uint8_t hydration_level;    // 0-100%
    uint32_t ir_value;          // Valor infrarrojo
    uint32_t red_value;         // Valor rojo
    float perfusion_index;      // Índice de perfusión
    bool is_low_hydration;      // Hidratación baja
    uint32_t timestamp;
} hydration_data_t;

// Datos de pasos
typedef struct {
    uint32_t daily_steps;       // Pasos del día
    float distance_km;          // Distancia en km
    uint16_t calories_burned;   // Calorías estimadas
    float avg_step_rate;        // Pasos por minuto promedio
    uint32_t timestamp;
} step_data_t;

// Datos de sueño
typedef struct {
    bool is_sleeping;           // ¿Está durmiendo?
    uint8_t sleep_quality;      // 0-100%
    uint32_t deep_sleep_minutes; // Minutos de sueño profundo
    uint32_t light_sleep_minutes; // Minutos de sueño ligero
    uint32_t rem_sleep_minutes;  // Minutos de sueño REM
    uint8_t sleep_efficiency;    // % eficiencia
    uint32_t bedtime;           // Timestamp hora de acostarse
    uint32_t wake_time;         // Timestamp hora de despertar
} sleep_data_t;

// Estados emocionales
typedef enum {
    EMOTION_CALM_RELAXED,
    EMOTION_ENERGETIC_HAPPY,
    EMOTION_STRESSED_ANXIOUS,
    EMOTION_TIRED_DOWN,
    EMOTION_FOCUSED_FLOW
} emotional_state_t;

// Patrones de vibración
typedef enum {
    VIB_PATTERN_NONE = 0,
    VIB_PATTERN_STARTUP,
    VIB_PATTERN_POSTURE_ALERT,
    VIB_PATTERN_STRESS_BREATHING,
    VIB_PATTERN_HYDRATION_ALERT,
    VIB_PATTERN_HR_ELEVATED_ALERT,
    VIB_PATTERN_AIR_QUALITY_ALERT,
    VIB_PATTERN_SEDENTARISM_SOFT,
    VIB_PATTERN_SEDENTARISM_STRONG,
    VIB_PATTERN_POSTURE_CORRECTION,
    VIB_PATTERN_SLEEP_WAKE_UP,
    VIB_PATTERN_PARTNER_SYNC
} vibration_pattern_t;

// Estados del sistema
typedef enum {
    SYSTEM_STATE_INIT,
    SYSTEM_STATE_ACTIVE,
    SYSTEM_STATE_SLEEP_MONITORING,
    SYSTEM_STATE_LOW_BATTERY,
    SYSTEM_STATE_CHARGING,
    SYSTEM_STATE_ERROR
} system_state_t;

// ==================== FUNCIONES DE UTILIDAD ====================

static inline float clamp_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static inline uint16_t clamp_uint16(uint16_t value, uint16_t min, uint16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Calcular magnitud de vector 3D
static inline float vector3_magnitude(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

// Calcular ángulo de inclinación desde acelerómetro
static inline float calculate_tilt_angle(float accel_x, float accel_y, float accel_z) {
    float magnitude = vector3_magnitude(accel_x, accel_y, accel_z);
    if (magnitude < 0.1f) return 0.0f; // Evitar división por cero
    
    // Ángulo respecto a la vertical (postura erguida)
    float angle_rad = acosf(accel_z / magnitude);
    return angle_rad * 180.0f / M_PI; // Convertir a grados
}

#endif // COMMON_TYPES_H