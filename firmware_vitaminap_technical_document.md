# Desarrollo de Firmware para el Collar Inteligente "Vitamina P" (ESP32-S3)

## Introducción

El collar inteligente **Vitamina P** es un dispositivo wearable de salud y bienestar diseñado para parejas, capaz de monitorear señales fisiológicas y ambientales, brindar retroalimentación háptica (vibración) y comunicarse vía **Bluetooth Low Energy (BLE)** y **Wi-Fi**. El sistema está construido en torno al microcontrolador **ESP32-S3FH4R2**, que integra conectividad Wi-Fi/BLE y potencia de procesamiento para ejecutar múltiples tareas en tiempo real.

El hardware del collar incluye diversos sensores avanzados: un sensor **MAX30102** para fotopletismografía (PPG) que mide pulso cardíaco y saturación de oxígeno en sangre (SpO₂), un sensor **MAX30205** para temperatura corporal de alta precisión, un **MAX86176** (un front-end analógico óptico) configurado para estimar hidratación y oxigenación muscular, un sensor ambiental **BME688** que mide temperatura, humedad, presión barométrica y compuestos orgánicos volátiles (VOC) del aire, un **BMI270** (unidad inercial de 6 ejes: acelerómetro + giroscopio) junto con un **BMM150** (magnetómetro de 3 ejes) para detección de postura y orientación, además de un **motor vibrador** para alertas hápticas y el integrado de **carga inalámbrica Qi BQ51013B** para recargar la batería.

**Nota:** El diseño original incluía un sensor ECG **AD8232** para electrocardiografía, pero en esta versión de firmware *se ha eliminado por completo el AD8232*, simplificando la electrónica y enfocando el monitoreo cardiaco en métodos PPG ópticos. Por lo tanto, no se implementa ningún módulo de ECG dedicado en el firmware, reduciendo la complejidad de hardware y consumo energético asociado a ese sensor.

Este documento técnico presenta la arquitectura integral del firmware del collar Vitamina P, abarcando su estructura modular por sensores y servicios, las consideraciones de multitarea con **ESP-IDF** (Framework de Espressif para ESP32) y **FreeRTOS**, la gestión de entradas/salidas (p. ej. el botón físico *SW5* como activador de BLE), y el manejo detallado de cada componente del hardware.

## Arquitectura General del Sistema

### Arquitectura de Hardware

El collar Vitamina P combina múltiples sensores biométricos y ambientales alrededor del microcontrolador ESP32-S3. Todos los sensores se comunican con el ESP32-S3 principalmente a través de buses I²C (y SPI en casos necesarios), de acuerdo al diseño esquemático y la lista de materiales (BOM).

El **ESP32-S3FH4R2** cuenta con:
- 4 MB de flash integrada
- 2 MB de PSRAM integrada
- Doble núcleo de 32 bits (240 MHz)
- Conectividad Wi-Fi 2.4 GHz y Bluetooth 5 LE
- Múltiples interfaces I²C, SPI, UART
- ADCs de 12 bits para medición de batería

Los sensores están conectados de la siguiente manera:
- **MAX30102**, **MAX30205**, **MAX86176**: I²C Bus 0
- **BME688**: I²C Bus 0 (dirección diferente)
- **BMI270**, **BMM150**: I²C Bus 1 o compartido con Bus 0
- **Motor vibrador**: GPIO con control PWM
- **Botón SW5**: GPIO con interrupción
- **Detección de carga**: GPIO para señal de alimentación externa

### Arquitectura de Software

La solución de firmware se basa en **ESP-IDF v5.x** y aprovecha el sistema operativo en tiempo real **FreeRTOS** integrado en el ESP32 para gestionar múltiples **tareas concurrentes**. El diseño sigue una arquitectura en capas:

#### Capa de Drivers de Sensores
- **sensor_max30102.c**: Driver para sensor PPG (pulso y SpO₂)
- **sensor_max30205.c**: Driver para temperatura corporal
- **sensor_max86176.c**: Driver para hidratación y PPG avanzado
- **sensor_bme688.c**: Driver para sensor ambiental (VOC, humedad, presión, temperatura)
- **sensor_bmi270.c**: Driver para IMU (acelerómetro + giroscopio)
- **sensor_bmm150.c**: Driver para magnetómetro

#### Capa de Servicios del Sistema
- **ble_service.c**: Conectividad Bluetooth LE y perfiles GATT
- **wifi_sync.c**: Sincronización Wi-Fi durante carga
- **storage.c**: Gestión de SPIFFS y NVS
- **vibration.c**: Control del motor vibrador
- **power_manager.c**: Gestión avanzada de energía

#### Capa de Lógica de Aplicación (Tareas FreeRTOS)
- **task_postura.c**: Análisis de postura corporal
- **task_salud.c**: Monitoreo de salud (HR, SpO₂, estrés, fatiga)
- **task_ambiente.c**: Calidad del aire y temperatura
- **task_pareja.c**: Comunicación y radar emocional entre parejas

#### Capa de Inicio y Configuración
- **main.c**: Punto de entrada y configuración del sistema

## Sistema Operativo y Multitarea (ESP-IDF + FreeRTOS)

El firmware aprovecha **FreeRTOS**, un sistema operativo de tiempo real integrado en ESP-IDF, para ejecutar múltiples tareas de manera simultánea y determinista. En el ESP32-S3, FreeRTOS corre en un entorno **SMP (Symmetric Multiprocessing)** con dos núcleos disponibles.

### Asignación de Núcleos
- **Núcleo 0**: Reservado para tareas internas de alta prioridad (Wi-Fi, Bluetooth y sistema)
- **Núcleo 1**: Tareas de la aplicación del collar

### Tareas Principales
```c
// Prioridades de tareas
#define PRIORITY_HIGH     (tskIDLE_PRIORITY + 3)
#define PRIORITY_MEDIUM   (tskIDLE_PRIORITY + 2)
#define PRIORITY_LOW      (tskIDLE_PRIORITY + 1)

// Creación de tareas en main.c
xTaskCreatePinnedToCore(task_postura, "Postura", 4096, NULL, PRIORITY_MEDIUM, NULL, 1);
xTaskCreatePinnedToCore(task_salud, "Salud", 8192, NULL, PRIORITY_MEDIUM, NULL, 1);
xTaskCreatePinnedToCore(task_ambiente, "Ambiente", 4096, NULL, PRIORITY_LOW, NULL, 1);
xTaskCreatePinnedToCore(task_pareja, "Pareja", 4096, NULL, PRIORITY_LOW, NULL, 1);
```

### Comunicación entre Tareas
- **Colas (queues)**: Para transferir datos entre tareas
- **Semáforos/Mutex**: Para proteger recursos compartidos (bus I²C, SPIFFS)
- **Event Groups**: Para estados globales del sistema

## Módulos de Sensores (Drivers)

### MAX30102 – Sensor PPG (Pulso y Oxímetro)

El **MAX30102** es un módulo integrado de pulso-oximetría que combina LEDs (rojo e infrarrojo) y un fotodetector para medir la señal PPG del pulso sanguíneo.

```c
// sensor_max30102.h
typedef struct {
    uint32_t ir_value;
    uint32_t red_value;
    uint32_t timestamp;
} max30102_sample_t;

esp_err_t max30102_init(i2c_port_t port);
esp_err_t max30102_read_fifo(max30102_sample_t *samples, size_t *count);
esp_err_t max30102_set_shutdown(bool enable);
float max30102_calculate_heart_rate(max30102_sample_t *samples, size_t count);
float max30102_calculate_spo2(max30102_sample_t *samples, size_t count);
```

**Características principales:**
- Comunicación I²C (dirección 0x57)
- FIFO interno para 32 muestras
- Interrupción cuando hay datos disponibles
- Modos de bajo consumo (<1µA en shutdown)

**Implementación:**
- Configuración de corriente de LEDs según tono de piel
- Algoritmos para cálculo de HR y SpO₂
- Detección de variabilidad cardíaca (HRV) para análisis de estrés

### MAX30205 – Sensor de Temperatura Corporal

Sensor digital de temperatura de alta precisión (±0.1°C) para medición de temperatura de la piel.

```c
// sensor_max30205.h
esp_err_t max30205_init(i2c_port_t port);
float max30205_read_temperature(void);
esp_err_t max30205_set_alarm_thresholds(float low, float high);
```

**Características:**
- Comunicación I²C (dirección 0x48)
- Resolución de 16 bits
- Conversión directa a grados Celsius
- Alarmas programables por hardware

### MAX86176 – Sensor Óptico de Hidratación

Front-End Analógico (AFE) avanzado para medición de hidratación corporal mediante señales PPG especializadas.

```c
// sensor_max86176.h
typedef struct {
    float hydration_level;  // 0-100%
    float perfusion_index;
    uint32_t quality_metric;
} hydration_data_t;

esp_err_t max86176_init(i2c_port_t port);
esp_err_t max86176_read_hydration(hydration_data_t *data);
esp_err_t max86176_calibrate_baseline(void);
```

**Funcionalidad:**
- Medición de hidratación usando múltiples longitudes de onda
- Algoritmos propietarios para estimar contenido de agua en tejidos
- Calibración personalizada por usuario

### BME688 – Sensor Ambiental

Sensor 4-en-1 que combina gas VOC, humedad, presión y temperatura ambiental.

```c
// sensor_bme688.h
typedef struct {
    float temperature;      // °C
    float humidity;        // %RH
    float pressure;        // hPa
    float gas_resistance;  // kΩ
    uint16_t iaq_index;    // 0-500 (Índice de Calidad del Aire)
} bme688_data_t;

esp_err_t bme688_init(i2c_port_t port);
esp_err_t bme688_read_all(bme688_data_t *data);
esp_err_t bme688_force_measurement(void);
```

**Características avanzadas:**
- Integración con librería BSEC para IAQ calibrado
- Modo de medición forzada para ahorro de energía
- Detección de gases como CO, H₂, etanol, etc.

### BMI270 – Unidad Inercial (IMU de 6 ejes)

IMU de última generación optimizada para wearables con detección inteligente de actividad.

```c
// sensor_bmi270.h
typedef struct {
    float accel_x, accel_y, accel_z;  // g
    float gyro_x, gyro_y, gyro_z;     // dps
    uint32_t timestamp;
} imu_data_t;

esp_err_t bmi270_init(i2c_port_t port);
esp_err_t bmi270_read_data(imu_data_t *data);
esp_err_t bmi270_setup_interrupts(void);
float bmi270_calculate_tilt_angle(imu_data_t *data);
```

**Funciones inteligentes:**
- Detección de movimiento/no-movimiento
- Cálculo de orientación para postura
- Interrupciones configurables
- Modos de bajo consumo

### BMM150 – Magnetómetro

Magnetómetro de 3 ejes para orientación absoluta y complemento de la IMU.

```c
// sensor_bmm150.h
typedef struct {
    float mag_x, mag_y, mag_z;  // µT
    float heading;              // grados (0-360)
} magnetometer_data_t;

esp_err_t bmm150_init(i2c_port_t port);
esp_err_t bmm150_read_data(magnetometer_data_t *data);
esp_err_t bmm150_calibrate(void);
```

## Módulos de Servicios

### Servicio Bluetooth Low Energy (BLE)

Implementa la comunicación con aplicaciones móviles usando un perfil GATT personalizado.

```c
// ble_service.h
#define VITAMINAP_SERVICE_UUID     0x1800
#define HEART_RATE_CHAR_UUID      0x2A37
#define SPO2_CHAR_UUID            0x2A5E
#define STRESS_CHAR_UUID          0x3001  // Custom
#define HYDRATION_CHAR_UUID       0x3002  // Custom
#define POSTURE_CHAR_UUID         0x3003  // Custom
#define SEDENTARY_STATUS_CHAR_UUID 0x3004  // Custom
#define SEDENTARY_STATS_CHAR_UUID  0x3005  // Custom
#define HAPTIC_CONFIG_CHAR_UUID    0x3006  // Custom - Configuración háptica
#define HAPTIC_CALIBRATION_CHAR_UUID 0x3007  // Custom - Calibración personal
#define HAPTIC_PATTERN_TEST_CHAR_UUID 0x3008  // Custom - Test de patrones
#define HAPTIC_LEARNING_DATA_CHAR_UUID 0x3009  // Custom - Datos de aprendizaje
#define HAPTIC_CONTEXT_CHAR_UUID   0x300A  // Custom - Contexto del usuario

esp_err_t ble_service_init(void);
esp_err_t ble_start_advertising(void);
esp_err_t ble_notify_heart_rate(uint8_t bpm);
esp_err_t ble_notify_stress_level(uint8_t level);
esp_err_t ble_set_partner_vibration(uint8_t pattern);
esp_err_t ble_notify_sedentary_alert(uint32_t minutes_inactive);
esp_err_t ble_notify_sedentary_stats(uint32_t daily_sedentary_time);
esp_err_t ble_set_sedentary_threshold(uint32_t threshold_minutes);

// Funciones BLE para sistema háptico avanzado
esp_err_t ble_haptic_start_calibration(void);
esp_err_t ble_haptic_set_pattern_intensity(haptic_pattern_t pattern, uint8_t intensity);
esp_err_t ble_haptic_set_user_sensitivity(float sensitivity);
esp_err_t ble_haptic_set_context(user_context_t context);
esp_err_t ble_haptic_enable_learning(bool enable);
esp_err_t ble_haptic_test_pattern(haptic_pattern_t pattern);
esp_err_t ble_haptic_get_learning_data(char *buffer, size_t buffer_size);
esp_err_t ble_haptic_set_circadian_adjustment(bool enable);
esp_err_t ble_haptic_save_memory_pattern(const char* event_name);
esp_err_t ble_haptic_replay_memory_pattern(const char* event_name);
esp_err_t ble_notify_haptic_responsiveness(haptic_pattern_t pattern);
esp_err_t ble_notify_haptic_context(user_context_t context);
```

**Características GATT definidas:**
- **Ritmo Cardíaco**: Notificación cada segundo
- **SpO₂**: Porcentaje de oxígeno en sangre
- **Índice de Estrés**: Valor 0-100 basado en HRV
- **Temperatura Corporal**: En grados Celsius
- **Nivel de Hidratación**: Porcentaje estimado
- **Estado de Postura**: Ángulo de inclinación
- **Calidad del Aire**: Índice IAQ
- **Estado Emocional**: Código de emoción detectada
- **Control de Pareja**: Comandos para vibración sincronizada
- **Estado de Sedentarismo**: Tiempo sin movimiento y alertas activas
- **Estadísticas de Sedentarismo**: Tiempo sedentario diario acumulado
- **Configuración Háptica**: Parámetros de intensidad y sensibilidad por patrón
- **Calibración Háptica**: Inicio de proceso de calibración personal
- **Test de Patrones**: Ejecutar patrones específicos para prueba
- **Datos de Aprendizaje**: Exportar estadísticas de respuesta del usuario
- **Contexto del Usuario**: Estado actual detectado automáticamente

### Sincronización Wi-Fi

Wi-Fi se activa únicamente durante la carga para sincronización con la nube.

```c
// wifi_sync.h
esp_err_t wifi_init(void);
esp_err_t wifi_connect_from_nvs(void);
esp_err_t wifi_provision_from_ble(const char* ssid, const char* password);
esp_err_t sync_data_to_cloud(void);
esp_err_t check_ota_updates(void);
```

**Funciones de sincronización:**
- Upload de datos almacenados localmente
- Actualización OTA de firmware
- Sincronización de hora vía NTP
- Comunicación remota entre parejas

### Almacenamiento Local

Gestión de memoria no volátil para datos y configuraciones.

```c
// storage.h
esp_err_t storage_init(void);
esp_err_t storage_log_event(const char* event, const char* data);
esp_err_t storage_save_daily_metrics(daily_metrics_t* metrics);
esp_err_t storage_get_config(const char* key, void* value, size_t* length);
esp_err_t storage_set_config(const char* key, const void* value, size_t length);
```

**Estructura de almacenamiento:**
- **SPIFFS**: Archivos de logs y memoria emocional
- **NVS**: Configuraciones y credenciales
- **Particiones**: Configuradas para OTA y datos

### Control de Vibración Háptica Inteligente

Sistema de retroalimentación háptica neurocientíficamente optimizada con patrones diferenciados y aprendizaje adaptativo.

```c
// advanced_haptic_system.h

#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>

// Configuración hardware optimizada para percepción neurológica
#define HAPTIC_PWM_RESOLUTION    LEDC_TIMER_12_BIT  // 4096 niveles de intensidad
#define HAPTIC_FREQUENCY_BASE    175                 // Hz óptimo neurológico  
#define HAPTIC_GPIO_PIN          GPIO_NUM_18        // Pin del motor vibrador
#define HAPTIC_PWM_CHANNEL       LEDC_CHANNEL_0
#define HAPTIC_PWM_TIMER         LEDC_TIMER_0

// Niveles de intensidad neurocientíficamente calibrados
typedef enum {
    HAPTIC_SUBLIMINAL = 0,      // 10-20% - Información no crítica
    HAPTIC_CONSCIOUS_SOFT,      // 25-35% - Recordatorios amigables  
    HAPTIC_ATTENTION_MEDIUM,    // 40-55% - Correcciones de hábitos
    HAPTIC_ACTION_REQUIRED,     // 60-80% - Alertas que requieren acción
    HAPTIC_EMERGENCY_CRITICAL   // 85-100% - Emergencias críticas
} haptic_intensity_level_t;

// Patrones hápticos neurocientíficamente optimizados
typedef enum {
    HAPTIC_PATTERN_POSTURE_GENTLE_REMINDER = 0,  // Triple toque ascendente
    HAPTIC_PATTERN_SEDENTARY_URGENCY_WAVE,       // Onda de activación 
    HAPTIC_PATTERN_STRESS_BREATH_SYNC,           // Respiración guiada
    HAPTIC_PATTERN_HYDRATION_AQUA_DROP,          // Gotas de agua simuladas
    HAPTIC_PATTERN_AIR_QUALITY_WIND_WHISPER,     // Brisa suave irregular
    HAPTIC_PATTERN_LOVE_RADAR_HEARTBEAT,         // Latido cardíaco real pareja
    HAPTIC_PATTERN_FALL_DETECTION_SOS,           // Código morse SOS + escalada
    HAPTIC_PATTERN_FLOW_STATE_FOCUS_LOCK,        // Confirmación sutil concentración
    HAPTIC_PATTERN_PARTNER_STRESS_EMPATHY,       // Pulso empático compartido
    HAPTIC_PATTERN_OPTIMAL_ENVIRONMENT_SPARKLE,  // Vibración "sparkle" bienestar
    HAPTIC_PATTERN_CUSTOM_MEMORY_REPLAY,         // Reproducción memoria especial
    HAPTIC_PATTERN_HARMONY_DETECTED,             // Sincronización biométrica detectada
    HAPTIC_PATTERN_MAX
} haptic_pattern_t;

// Contexto del usuario para adaptación inteligente
typedef enum {
    USER_CONTEXT_SLEEPING,           // Solo emergencias
    USER_CONTEXT_WORKING_FOCUSED,    // Intensidades -50%
    USER_CONTEXT_SOCIAL_INTERACTION, // Solo críticas
    USER_CONTEXT_EXERCISING,         // Patrones deportivos  
    USER_CONTEXT_RELAXING,           // Énfasis en bienestar
    USER_CONTEXT_COUPLE_TIME,        // Love radar activado
    USER_CONTEXT_AUTO_DETECT         // Detección automática
} user_context_t;

// Estructura de un pulso háptico individual
typedef struct {
    uint16_t duration_ms;           // Duración del pulso
    uint8_t intensity_percent;      // Intensidad 0-100%
    uint16_t pause_after_ms;        // Pausa después del pulso
    bool use_texture;               // Aplicar textura al pulso
    float texture_frequency_hz;     // Frecuencia de textura interna
} haptic_pulse_t;

// Patrón háptico complejo
typedef struct {
    haptic_pulse_t pulses[16];      // Hasta 16 pulsos por patrón
    uint8_t pulse_count;            // Número de pulsos activos
    uint16_t total_duration_ms;     // Duración total del patrón
    bool loop_pattern;              // Repetir patrón
    uint8_t loop_count;             // Número de repeticiones
    haptic_intensity_level_t base_intensity; // Nivel base de intensidad
    bool context_adaptive;          // Se adapta al contexto del usuario
} haptic_pattern_def_t;

// Perfil de personalización háptica con aprendizaje automático
typedef struct {
    float user_sensitivity;         // Sensibilidad aprendida en 7 días
    uint8_t preferred_intensity;    // Intensidad preferida por respuesta
    bool contextual_awareness;      // Detecta actividad actual
    float habituation_resistance;   // Evita acostumbramiento
    uint32_t pattern_rotation;      // Rota micro-variaciones
    float morning_responsiveness;   // Más sensible por las mañanas
    float working_hours_tolerance;  // Menor intensidad en trabajo
    float stress_vibration_efficiency; // Qué patrones lo calman más
    uint8_t preferred_posture_reminder; // Frecuencia personal óptima
    uint32_t total_interactions;    // Número total de interacciones
    float positive_response_rate;   // Tasa de respuesta positiva
} haptic_personalization_t;

// Monitor de estado háptico
typedef struct {
    bool is_active;                 // Sistema activo
    haptic_pattern_t current_pattern; // Patrón ejecutándose
    uint32_t pattern_start_time;    // Tiempo inicio patrón actual
    user_context_t current_context; // Contexto actual del usuario
    uint8_t daily_pattern_count[HAPTIC_PATTERN_MAX]; // Contador diario por patrón
    uint32_t last_pattern_time[HAPTIC_PATTERN_MAX];  // Última vez cada patrón
    bool emergency_override;        // Override para emergencias
    float current_ambient_noise;    // Ruido ambiente para compensación
    bool learning_mode_active;      // Modo aprendizaje activo
} haptic_monitor_t;

// Controlador háptico avanzado
typedef struct {
    ledc_channel_config_t pwm_channel;
    ledc_timer_config_t pwm_timer;
    haptic_pattern_def_t patterns[HAPTIC_PATTERN_MAX];
    haptic_personalization_t user_profile;
    haptic_monitor_t monitor;
    QueueHandle_t haptic_queue;     // Cola de comandos hápticos
    TaskHandle_t haptic_task_handle; // Handle de tarea háptica
    SemaphoreHandle_t haptic_mutex;  // Mutex para acceso concurrente
} advanced_haptic_controller_t;

// Comando háptico para cola
typedef struct {
    haptic_pattern_t pattern;
    haptic_intensity_level_t intensity_override;
    bool urgent;                    // Ejecutar inmediatamente
    uint32_t delay_ms;             // Retraso antes de ejecutar
    void (*completion_callback)(haptic_pattern_t); // Callback al completar
} haptic_command_t;

// Funciones principales del sistema háptico
esp_err_t advanced_haptic_init(void);
esp_err_t haptic_execute_pattern(haptic_pattern_t pattern);
esp_err_t haptic_execute_pattern_with_intensity(haptic_pattern_t pattern, haptic_intensity_level_t intensity);
esp_err_t haptic_execute_custom_pattern(haptic_pattern_def_t *custom_pattern);
esp_err_t haptic_stop_current_pattern(void);
esp_err_t haptic_emergency_override(haptic_pattern_t emergency_pattern);

// Funciones de personalización y aprendizaje
esp_err_t haptic_start_calibration(void);
esp_err_t haptic_update_user_response(haptic_pattern_t pattern, float response_quality);
esp_err_t haptic_set_context(user_context_t context);
esp_err_t haptic_enable_adaptive_learning(bool enable);
user_context_t haptic_detect_context_auto(void);

// Funciones de configuración avanzada
esp_err_t haptic_set_pattern_intensity(haptic_pattern_t pattern, uint8_t intensity_percent);
esp_err_t haptic_enable_pattern_variation(haptic_pattern_t pattern, bool enable);
esp_err_t haptic_set_sensitivity(float sensitivity_factor);
esp_err_t haptic_save_memory_pattern(const char* event_name, haptic_pattern_def_t *pattern);
esp_err_t haptic_replay_memory_pattern(const char* event_name);

// Funciones de análisis y estadísticas
uint32_t haptic_get_daily_pattern_count(haptic_pattern_t pattern);
float haptic_get_user_responsiveness(haptic_pattern_t pattern);
esp_err_t haptic_get_usage_stats(uint32_t *total_patterns, float *avg_response_rate);
esp_err_t haptic_export_learning_data(char *buffer, size_t buffer_size);

// Funciones de contexto inteligente
esp_err_t haptic_circadian_adjustment(void);
esp_err_t haptic_ambient_noise_compensation(float noise_level_db);
esp_err_t haptic_couple_sync_detection(float partner_heart_rate, bool partner_nearby);

// Funciones de emergency y seguridad
esp_err_t haptic_test_emergency_patterns(void);
esp_err_t haptic_fall_detection_alert(void);
esp_err_t haptic_stress_cascade_prevention(uint8_t stress_level);

// Funciones utilitarias internas
static void haptic_task(void *pvParameters);
static esp_err_t haptic_execute_pulse(haptic_pulse_t *pulse);
static esp_err_t haptic_apply_intensity_level(uint8_t base_intensity, haptic_intensity_level_t level);
static esp_err_t haptic_generate_pattern_variation(haptic_pattern_t base_pattern);
static float haptic_calculate_optimal_intensity(haptic_pattern_t pattern);
static esp_err_t haptic_update_personalization(haptic_pattern_t pattern, bool user_responded);
static uint32_t haptic_get_circadian_multiplier(void);
```

**Implementación de patrones neurocientíficos:**

```c
// advanced_haptic_patterns.c

// Inicialización de patrones neurocientíficamente optimizados
static void init_haptic_patterns(advanced_haptic_controller_t *controller) {
    
    // 1. POSTURA INCORRECTA - "GENTLE REMINDER"
    controller->patterns[HAPTIC_PATTERN_POSTURE_GENTLE_REMINDER] = {
        .pulses = {
            {150, 30, 80, false, 0},    // • (150ms,30%)
            {200, 40, 80, false, 0},    // •• (200ms,40%) 
            {250, 50, 0, false, 0}      // ••• (250ms,50%)
        },
        .pulse_count = 3,
        .total_duration_ms = 760,
        .loop_pattern = false,
        .loop_count = 1,
        .base_intensity = HAPTIC_ATTENTION_MEDIUM,
        .context_adaptive = true
    };

    // 2. SEDENTARISMO - "URGENCY WAVE"  
    controller->patterns[HAPTIC_PATTERN_SEDENTARY_URGENCY_WAVE] = {
        .pulses = {
            {2500, 20, 0, true, 0.5f}, // Onda larga con crescendo
            {300, 70, 100, false, 0},   // Pulso fuerte 1
            {300, 70, 800, false, 0}    // Pulso fuerte 2
        },
        .pulse_count = 3,
        .total_duration_ms = 4000,
        .loop_pattern = false,
        .loop_count = 1,
        .base_intensity = HAPTIC_ACTION_REQUIRED,
        .context_adaptive = true
    };

    // 3. ESTRÉS ELEVADO - "BREATH SYNC"
    controller->patterns[HAPTIC_PATTERN_STRESS_BREATH_SYNC] = {
        .pulses = {
            {4000, 10, 0, true, 0.25f}, // Inhalar - crescendo
            {1000, 40, 0, false, 0},     // Pausa
            {6000, 40, 3000, true, 0.15f} // Exhalar - decrescendo
        },
        .pulse_count = 3,
        .total_duration_ms = 14000,
        .loop_pattern = true,
        .loop_count = 3,
        .base_intensity = HAPTIC_CONSCIOUS_SOFT,
        .context_adaptive = true
    };

    // 4. HIDRATACIÓN BAJA - "AQUA DROP"
    controller->patterns[HAPTIC_PATTERN_HYDRATION_AQUA_DROP] = {
        .pulses = {
            {50, 20, 200, false, 0},    // 💧 gota 1
            {100, 25, 400, false, 0},   // 💧💧 gotas 2  
            {150, 30, 0, false, 0}      // 💧💧💧 gotas 3
        },
        .pulse_count = 3,
        .total_duration_ms = 900,
        .loop_pattern = false,
        .loop_count = 1,
        .base_intensity = HAPTIC_ATTENTION_MEDIUM,
        .context_adaptive = true
    };

    // 5. CALIDAD DEL AIRE - "WIND WHISPER"
    controller->patterns[HAPTIC_PATTERN_AIR_QUALITY_WIND_WHISPER] = {
        .pulses = {
            {100, 15, 50, false, 0},    // ~ brisa suave
            {150, 18, 30, false, 0},    // ~~ un poco más
            {80, 12, 70, false, 0},     // ~ suave otra vez  
            {200, 20, 0, false, 0}      // ~~~ final
        },
        .pulse_count = 4,
        .total_duration_ms = 680,
        .loop_pattern = false,
        .loop_count = 1,
        .base_intensity = HAPTIC_CONSCIOUS_SOFT,
        .context_adaptive = true
    };

    // 6. LOVE RADAR - "HEARTBEAT MIRROR" (dinámico, se configura en tiempo real)
    controller->patterns[HAPTIC_PATTERN_LOVE_RADAR_HEARTBEAT] = {
        .pulses = {
            // Se configura dinámicamente basado en BPM real de la pareja
            {0, 30, 0, false, 0}  // Placeholder - se actualiza con BPM real
        },
        .pulse_count = 1,
        .total_duration_ms = 0, // Calculado dinámicamente
        .loop_pattern = true,
        .loop_count = 5,
        .base_intensity = HAPTIC_CONSCIOUS_SOFT,
        .context_adaptive = true
    };

    // 7. DETECCIÓN DE CAÍDA - "SOS EMERGENCY"
    controller->patterns[HAPTIC_PATTERN_FALL_DETECTION_SOS] = {
        .pulses = {
            // ••• (SOS dots)
            {100, 80, 50, false, 0}, {100, 80, 50, false, 0}, {100, 80, 200, false, 0},
            // ——— (SOS dashes)  
            {300, 90, 100, false, 0}, {300, 90, 100, false, 0}, {300, 90, 200, false, 0},
            // ••• (SOS dots)
            {100, 80, 50, false, 0}, {100, 80, 50, false, 0}, {100, 80, 1000, false, 0},
            // Escalada continua
            {3000, 50, 0, true, 2.0f} // Escalada 50%→100% en 3 segundos
        },
        .pulse_count = 10,
        .total_duration_ms = 8000,
        .loop_pattern = true,
        .loop_count = 10, // Hasta cancelación manual
        .base_intensity = HAPTIC_EMERGENCY_CRITICAL,
        .context_adaptive = false // Siempre máxima intensidad
    };

    // 8. FLOW STATE DETECTADO - "FOCUS LOCK"
    controller->patterns[HAPTIC_PATTERN_FLOW_STATE_FOCUS_LOCK] = {
        .pulses = {
            {25, 15, 5000, false, 0},   // Confirmación sutil
            {25, 15, 0, false, 0}       // Solo al entrar en flow
        },
        .pulse_count = 2,
        .total_duration_ms = 5050,
        .loop_pattern = false,
        .loop_count = 1,
        .base_intensity = HAPTIC_SUBLIMINAL,
        .context_adaptive = false
    };

    // 9. PAREJA EN ESTRÉS - "EMPATHY PULSE"
    controller->patterns[HAPTIC_PATTERN_PARTNER_STRESS_EMPATHY] = {
        .pulses = {
            {250, 30, 1000, false, 0},  // ♥— pulso empático
            {250, 30, 3000, false, 0},  // ♥— segundo pulso
            {250, 30, 1000, false, 0},  // Repetir suavemente  
            {250, 30, 3000, false, 0},
            {250, 30, 1000, false, 0},
            {250, 30, 0, false, 0}
        },
        .pulse_count = 6,
        .total_duration_ms = 11500,
        .loop_pattern = false,
        .loop_count = 1,
        .base_intensity = HAPTIC_CONSCIOUS_SOFT,
        .context_adaptive = true
    };

    // 10. AMBIENTE ÓPTIMO - "PERFECT MOMENT"
    controller->patterns[HAPTIC_PATTERN_OPTIMAL_ENVIRONMENT_SPARKLE] = {
        .pulses = {
            {30, 20, 20, false, 0},     // ✨ micro-pulso 1
            {50, 25, 20, false, 0},     // ✨✨ micro-pulso 2
            {70, 30, 0, false, 0}       // ✨✨✨ micro-pulso 3
        },
        .pulse_count = 3,
        .total_duration_ms = 190,
        .loop_pattern = false,
        .loop_count = 1,
        .base_intensity = HAPTIC_SUBLIMINAL,
        .context_adaptive = true
    };
}
```

**Sistema de Inteligencia Adaptativa y Aprendizaje Automático:**

```c
// advanced_haptic_intelligence.c

#include "advanced_haptic_system.h"
#include "esp_timer.h"
#include "storage.h"
#include <time.h>

// Controller global del sistema háptico
static advanced_haptic_controller_t g_haptic_controller = {0};

// Inicialización del sistema háptico avanzado
esp_err_t advanced_haptic_init(void) {
    ESP_LOGI("HAPTIC", "Initializing Advanced Haptic System");
    
    // Configurar timer PWM para control de alta precisión
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = HAPTIC_PWM_RESOLUTION,
        .timer_num = HAPTIC_PWM_TIMER,
        .freq_hz = HAPTIC_FREQUENCY_BASE,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    
    // Configurar canal PWM
    ledc_channel_config_t channel_config = {
        .gpio_num = HAPTIC_GPIO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = HAPTIC_PWM_CHANNEL,
        .timer_sel = HAPTIC_PWM_TIMER,
        .duty = 0,
        .point = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    
    g_haptic_controller.pwm_channel = channel_config;
    g_haptic_controller.pwm_timer = timer_config;
    
    // Crear mutex para acceso thread-safe
    g_haptic_controller.haptic_mutex = xSemaphoreCreateMutex();
    if (g_haptic_controller.haptic_mutex == NULL) {
        ESP_LOGE("HAPTIC", "Failed to create haptic mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Crear cola de comandos hápticos
    g_haptic_controller.haptic_queue = xQueueCreate(10, sizeof(haptic_command_t));
    if (g_haptic_controller.haptic_queue == NULL) {
        ESP_LOGE("HAPTIC", "Failed to create haptic queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Inicializar patrones neurocientíficos
    init_haptic_patterns(&g_haptic_controller);
    
    // Cargar perfil de personalización desde NVS
    haptic_load_user_profile(&g_haptic_controller.user_profile);
    
    // Inicializar monitor de estado
    g_haptic_controller.monitor.is_active = true;
    g_haptic_controller.monitor.current_context = USER_CONTEXT_AUTO_DETECT;
    g_haptic_controller.monitor.learning_mode_active = true;
    
    // Crear tarea de procesamiento háptico
    BaseType_t task_created = xTaskCreatePinnedToCore(
        haptic_task,
        "haptic_processor",
        4096,
        &g_haptic_controller,
        configMAX_PRIORITIES - 2,
        &g_haptic_controller.haptic_task_handle,
        1  // Core 1 para aplicación
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE("HAPTIC", "Failed to create haptic task");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI("HAPTIC", "Advanced Haptic System initialized successfully");
    return ESP_OK;
}

// Tarea principal de procesamiento háptico
static void haptic_task(void *pvParameters) {
    advanced_haptic_controller_t *controller = (advanced_haptic_controller_t *)pvParameters;
    haptic_command_t command;
    TickType_t last_context_check = 0;
    TickType_t last_learning_update = 0;
    
    ESP_LOGI("HAPTIC", "Haptic processing task started");
    
    while (1) {
        // Procesar comandos en cola con timeout
        if (xQueueReceive(controller->haptic_queue, &command, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // Aplicar retraso si es necesario
            if (command.delay_ms > 0) {
                vTaskDelay(pdMS_TO_TICKS(command.delay_ms));
            }
            
            // Verificar si es comando urgente o en modo emergencia
            if (command.urgent || controller->monitor.emergency_override) {
                ESP_LOGI("HAPTIC", "Executing urgent/emergency pattern: %d", command.pattern);
                haptic_execute_pattern_immediate(command.pattern, command.intensity_override);
            } else {
                // Verificar contexto antes de ejecutar
                if (haptic_should_execute_in_context(command.pattern, controller->monitor.current_context)) {
                    haptic_execute_pattern_intelligent(command.pattern, command.intensity_override);
                } else {
                    ESP_LOGD("HAPTIC", "Pattern %d suppressed due to context %d", 
                            command.pattern, controller->monitor.current_context);
                }
            }
            
            // Ejecutar callback si existe
            if (command.completion_callback) {
                command.completion_callback(command.pattern);
            }
        }
        
        // Actualización periódica del contexto (cada 30 segundos)
        TickType_t current_time = xTaskGetTickCount();
        if (current_time - last_context_check > pdMS_TO_TICKS(30000)) {
            if (controller->monitor.current_context == USER_CONTEXT_AUTO_DETECT) {
                controller->monitor.current_context = haptic_detect_context_auto();
            }
            
            // Aplicar ajustes circadianos
            haptic_circadian_adjustment();
            
            last_context_check = current_time;
        }
        
        // Actualización del aprendizaje (cada 5 minutos)
        if (controller->monitor.learning_mode_active && 
            current_time - last_learning_update > pdMS_TO_TICKS(300000)) {
            
            haptic_update_learning_algorithms();
            haptic_save_user_profile(&controller->user_profile);
            
            last_learning_update = current_time;
        }
    }
}

// Detección automática de contexto usando sensores y algoritmos de ML
user_context_t haptic_detect_context_auto(void) {
    // Obtener datos de sensores actuales
    uint32_t current_time = esp_timer_get_time() / 1000000; // segundos
    struct tm timeinfo;
    localtime_r((time_t*)&current_time, &timeinfo);
    
    // Variables para análisis de contexto
    float movement_intensity = get_current_movement_intensity();
    uint8_t heart_rate = get_current_heart_rate();
    bool partner_nearby = is_partner_nearby();
    float ambient_light = get_ambient_light_level();
    
    // Algoritmo de detección de contexto
    
    // 1. Detectar sueño (22:00 - 06:00 + baja actividad)
    if ((timeinfo.tm_hour >= 22 || timeinfo.tm_hour < 6) && movement_intensity < 0.1f) {
        return USER_CONTEXT_SLEEPING;
    }
    
    // 2. Detectar ejercicio (movimiento alto + HR elevado)
    if (movement_intensity > 2.0f && heart_rate > get_baseline_heart_rate() + 30) {
        return USER_CONTEXT_EXERCISING;
    }
    
    // 3. Detectar tiempo de pareja (pareja cerca + horario social)
    if (partner_nearby && (timeinfo.tm_hour >= 18 || timeinfo.tm_hour < 9)) {
        return USER_CONTEXT_COUPLE_TIME;
    }
    
    // 4. Detectar trabajo concentrado (horario laboral + baja variabilidad de movimiento)
    if (timeinfo.tm_hour >= 9 && timeinfo.tm_hour < 17 && 
        movement_intensity < 0.5f && get_movement_variability() < 0.2f) {
        return USER_CONTEXT_WORKING_FOCUSED;
    }
    
    // 5. Detectar interacción social (movimiento moderado + variabilidad alta + horario social)
    if (movement_intensity > 0.5f && get_movement_variability() > 0.8f && 
        (timeinfo.tm_hour >= 17 || timeinfo.tm_hour < 2)) {
        return USER_CONTEXT_SOCIAL_INTERACTION;
    }
    
    // 6. Por defecto: relajación
    return USER_CONTEXT_RELAXING;
}

// Verificar si un patrón debe ejecutarse en el contexto actual
static bool haptic_should_execute_in_context(haptic_pattern_t pattern, user_context_t context) {
    switch (context) {
        case USER_CONTEXT_SLEEPING:
            // Solo emergencias durante el sueño
            return (pattern == HAPTIC_PATTERN_FALL_DETECTION_SOS);
            
        case USER_CONTEXT_WORKING_FOCUSED:
            // Solo patrones críticos o muy suaves
            return (pattern == HAPTIC_PATTERN_FALL_DETECTION_SOS ||
                   pattern == HAPTIC_PATTERN_FLOW_STATE_FOCUS_LOCK ||
                   pattern == HAPTIC_PATTERN_POSTURE_GENTLE_REMINDER);
                   
        case USER_CONTEXT_SOCIAL_INTERACTION:
            // Solo emergencias y avisos discretos
            return (pattern == HAPTIC_PATTERN_FALL_DETECTION_SOS ||
                   pattern == HAPTIC_PATTERN_PARTNER_STRESS_EMPATHY);
                   
        case USER_CONTEXT_EXERCISING:
            // Patrones relacionados con actividad física
            return (pattern != HAPTIC_PATTERN_SEDENTARY_URGENCY_WAVE &&
                   pattern != HAPTIC_PATTERN_FLOW_STATE_FOCUS_LOCK);
                   
        case USER_CONTEXT_COUPLE_TIME:
            // Énfasis en patrones de pareja y bienestar
            return true; // Todos los patrones permitidos
            
        case USER_CONTEXT_RELAXING:
        default:
            return true; // Todos los patrones permitidos
    }
}

// Ejecución inteligente de patrones con adaptación
static esp_err_t haptic_execute_pattern_intelligent(haptic_pattern_t pattern, 
                                                   haptic_intensity_level_t intensity_override) {
    if (xSemaphoreTake(g_haptic_controller.haptic_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW("HAPTIC", "Failed to acquire mutex for pattern execution");
        return ESP_ERR_TIMEOUT;
    }
    
    // Obtener definición del patrón
    haptic_pattern_def_t *pattern_def = &g_haptic_controller.patterns[pattern];
    
    // Aplicar personalización del usuario
    float intensity_multiplier = haptic_calculate_optimal_intensity(pattern);
    
    // Aplicar ajustes de contexto
    if (pattern_def->context_adaptive) {
        intensity_multiplier *= haptic_get_context_intensity_multiplier(g_haptic_controller.monitor.current_context);
    }
    
    // Aplicar ajuste circadiano
    intensity_multiplier *= haptic_get_circadian_multiplier();
    
    // Generar variación del patrón para evitar habituación
    if (g_haptic_controller.user_profile.habituation_resistance > 0.5f) {
        haptic_generate_pattern_variation(pattern);
    }
    
    // Registrar inicio de patrón
    g_haptic_controller.monitor.current_pattern = pattern;
    g_haptic_controller.monitor.pattern_start_time = esp_timer_get_time() / 1000;
    g_haptic_controller.monitor.daily_pattern_count[pattern]++;
    g_haptic_controller.monitor.last_pattern_time[pattern] = esp_timer_get_time() / 1000;
    
    // Ejecutar cada pulso del patrón
    for (int i = 0; i < pattern_def->pulse_count; i++) {
        haptic_pulse_t pulse = pattern_def->pulses[i];
        
        // Aplicar multiplicador de intensidad
        pulse.intensity_percent = (uint8_t)(pulse.intensity_percent * intensity_multiplier);
        if (pulse.intensity_percent > 100) pulse.intensity_percent = 100;
        
        // Aplicar override de intensidad si se especifica
        if (intensity_override != 0) {
            pulse.intensity_percent = haptic_apply_intensity_level(pulse.intensity_percent, intensity_override);
        }
        
        // Ejecutar pulso
        esp_err_t result = haptic_execute_pulse(&pulse);
        if (result != ESP_OK) {
            ESP_LOGW("HAPTIC", "Failed to execute pulse %d of pattern %d", i, pattern);
        }
        
        // Verificar si se ha solicitado parada
        if (!g_haptic_controller.monitor.is_active) {
            break;
        }
    }
    
    // Si el patrón tiene loop, repetir
    if (pattern_def->loop_pattern && pattern_def->loop_count > 1) {
        for (int loop = 1; loop < pattern_def->loop_count; loop++) {
            if (!g_haptic_controller.monitor.is_active) break;
            
            vTaskDelay(pdMS_TO_TICKS(200)); // Pausa entre loops
            
            // Re-ejecutar el patrón
            for (int i = 0; i < pattern_def->pulse_count; i++) {
                haptic_pulse_t pulse = pattern_def->pulses[i];
                pulse.intensity_percent = (uint8_t)(pulse.intensity_percent * intensity_multiplier);
                if (pulse.intensity_percent > 100) pulse.intensity_percent = 100;
                
                haptic_execute_pulse(&pulse);
                
                if (!g_haptic_controller.monitor.is_active) break;
            }
        }
    }
    
    // Limpiar estado actual
    g_haptic_controller.monitor.current_pattern = 0;
    
    xSemaphoreGive(g_haptic_controller.haptic_mutex);
    
    ESP_LOGD("HAPTIC", "Pattern %d executed successfully with intensity %.2f", pattern, intensity_multiplier);
    return ESP_OK;
}

// Ejecutar pulso individual con textura opcional
static esp_err_t haptic_execute_pulse(haptic_pulse_t *pulse) {
    if (!pulse || pulse->duration_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t duty_value = (pulse->intensity_percent * ((1 << HAPTIC_PWM_RESOLUTION) - 1)) / 100;
    
    if (pulse->use_texture && pulse->texture_frequency_hz > 0) {
        // Aplicar textura modulando la intensidad
        uint32_t texture_period_ms = 1000.0f / pulse->texture_frequency_hz;
        uint32_t elapsed_ms = 0;
        
        while (elapsed_ms < pulse->duration_ms) {
            float texture_phase = (float)(elapsed_ms % texture_period_ms) / texture_period_ms;
            float texture_multiplier = 0.5f + 0.5f * sinf(texture_phase * 2 * M_PI);
            
            uint32_t textured_duty = duty_value * texture_multiplier;
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, HAPTIC_PWM_CHANNEL, textured_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, HAPTIC_PWM_CHANNEL));
            
            vTaskDelay(pdMS_TO_TICKS(10));
            elapsed_ms += 10;
        }
    } else {
        // Pulso simple sin textura
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, HAPTIC_PWM_CHANNEL, duty_value));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, HAPTIC_PWM_CHANNEL));
        
        vTaskDelay(pdMS_TO_TICKS(pulse->duration_ms));
    }
    
    // Apagar motor
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, HAPTIC_PWM_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, HAPTIC_PWM_CHANNEL));
    
    // Pausa después del pulso
    if (pulse->pause_after_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(pulse->pause_after_ms));
    }
    
    return ESP_OK;
}

// Calcular intensidad óptima basada en aprendizaje del usuario
static float haptic_calculate_optimal_intensity(haptic_pattern_t pattern) {
    haptic_personalization_t *profile = &g_haptic_controller.user_profile;
    
    // Intensidad base del perfil del usuario
    float base_intensity = profile->user_sensitivity;
    
    // Ajustar según respuesta histórica a este patrón específico
    if (profile->total_interactions > 10) {
        float pattern_effectiveness = haptic_get_user_responsiveness(pattern);
        if (pattern_effectiveness < 0.5f) {
            base_intensity *= 1.2f; // Aumentar si el usuario no responde bien
        } else if (pattern_effectiveness > 0.8f) {
            base_intensity *= 0.9f; // Reducir si el usuario responde muy bien
        }
    }
    
    // Ajuste circadiano
    struct tm timeinfo;
    time_t now;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    if (timeinfo.tm_hour >= 6 && timeinfo.tm_hour <= 9) {
        base_intensity *= profile->morning_responsiveness;
    }
    
    if (timeinfo.tm_hour >= 9 && timeinfo.tm_hour <= 17) {
        base_intensity *= profile->working_hours_tolerance;
    }
    
    // Limitar rango válido
    if (base_intensity < 0.1f) base_intensity = 0.1f;
    if (base_intensity > 2.0f) base_intensity = 2.0f;
    
    return base_intensity;
}

// Actualizar algoritmos de aprendizaje
static void haptic_update_learning_algorithms(void) {
    haptic_personalization_t *profile = &g_haptic_controller.user_profile;
    
    // Calcular tasa de respuesta promedio
    uint32_t total_patterns = 0;
    uint32_t total_responses = 0;
    
    for (int i = 0; i < HAPTIC_PATTERN_MAX; i++) {
        total_patterns += g_haptic_controller.monitor.daily_pattern_count[i];
        // Simular respuestas del usuario (en implementación real se obtendría de sensores)
        total_responses += g_haptic_controller.monitor.daily_pattern_count[i] * 0.7f; // 70% de respuesta
    }
    
    if (total_patterns > 0) {
        float daily_response_rate = (float)total_responses / total_patterns;
        
        // Actualizar perfil con promedio móvil
        profile->positive_response_rate = (profile->positive_response_rate * 0.8f) + (daily_response_rate * 0.2f);
        
        // Ajustar sensibilidad basada en respuesta
        if (profile->positive_response_rate < 0.6f) {
            profile->user_sensitivity *= 1.05f; // Aumentar gradualmente
        } else if (profile->positive_response_rate > 0.9f) {
            profile->user_sensitivity *= 0.98f; // Reducir gradualmente
        }
        
        // Actualizar contador de interacciones totales
        profile->total_interactions += total_patterns;
    }
    
    ESP_LOGI("HAPTIC", "Learning update: sensitivity=%.2f, response_rate=%.2f", 
            profile->user_sensitivity, profile->positive_response_rate);
}

// Funciones públicas de la API

esp_err_t haptic_execute_pattern(haptic_pattern_t pattern) {
    if (pattern >= HAPTIC_PATTERN_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    haptic_command_t command = {
        .pattern = pattern,
        .intensity_override = 0,
        .urgent = false,
        .delay_ms = 0,
        .completion_callback = NULL
    };
    
    if (xQueueSend(g_haptic_controller.haptic_queue, &command, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW("HAPTIC", "Failed to queue pattern %d - queue full", pattern);
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t haptic_execute_pattern_with_intensity(haptic_pattern_t pattern, haptic_intensity_level_t intensity) {
    if (pattern >= HAPTIC_PATTERN_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    haptic_command_t command = {
        .pattern = pattern,
        .intensity_override = intensity,
        .urgent = false,
        .delay_ms = 0,
        .completion_callback = NULL
    };
    
    if (xQueueSend(g_haptic_controller.haptic_queue, &command, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW("HAPTIC", "Failed to queue pattern %d with intensity %d", pattern, intensity);
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t haptic_emergency_override(haptic_pattern_t emergency_pattern) {
    // Detener patrón actual inmediatamente
    g_haptic_controller.monitor.is_active = false;
    g_haptic_controller.monitor.emergency_override = true;
    
    haptic_command_t command = {
        .pattern = emergency_pattern,
        .intensity_override = HAPTIC_EMERGENCY_CRITICAL,
        .urgent = true,
        .delay_ms = 0,
        .completion_callback = NULL
    };
    
    // Limpiar cola y agregar comando de emergencia
    xQueueReset(g_haptic_controller.haptic_queue);
    xQueueSend(g_haptic_controller.haptic_queue, &command, 0);
    
    ESP_LOGI("HAPTIC", "Emergency override activated for pattern %d", emergency_pattern);
    return ESP_OK;
}

esp_err_t haptic_set_context(user_context_t context) {
    g_haptic_controller.monitor.current_context = context;
    ESP_LOGI("HAPTIC", "Context set to %d", context);
    return ESP_OK;
}

float haptic_get_user_responsiveness(haptic_pattern_t pattern) {
    if (pattern >= HAPTIC_PATTERN_MAX) {
        return 0.0f;
    }
    
    // En implementación real, esto se basaría en datos de sensores
    // Por ahora, simulamos basándose en el uso histórico
    uint32_t pattern_count = g_haptic_controller.monitor.daily_pattern_count[pattern];
    if (pattern_count == 0) {
        return 0.5f; // Valor neutro por defecto
    }
    
    // Simular respuesta basada en el tipo de patrón
    switch (pattern) {
        case HAPTIC_PATTERN_FALL_DETECTION_SOS:
            return 0.95f; // Emergencias tienen alta respuesta
        case HAPTIC_PATTERN_FLOW_STATE_FOCUS_LOCK:
            return 0.3f;  // Patrones sutiles tienen baja respuesta medible
        default:
            return g_haptic_controller.user_profile.positive_response_rate;
    }
}
```

**Integración con Tareas Existentes del Firmware:**

```c
// integration_haptic_tasks.c

// Actualización de task_postura.c para usar sistema háptico avanzado
void task_postura_haptic_upgraded(void *pvParameters) {
    imu_data_t imu_data;
    float current_angle;
    uint32_t bad_posture_start = 0;
    
    // Inicializar monitor de sedentarismo con sistema háptico
    sedentary_monitor_t sedentary_monitor = {
        .last_movement_time = esp_timer_get_time() / 1000,
        .activity_threshold = SEDENTARY_MOVEMENT_THRESHOLD,
        .sedentary_alert_interval = SEDENTARY_ALERT_THRESHOLD_MS,
        .sedentary_alert_active = false,
        .daily_sedentary_time = 0
    };
    
    ESP_LOGI("POSTURE", "Advanced haptic posture monitoring started");
    
    while (1) {
        // Leer datos del IMU
        if (bmi270_read_data(&imu_data) == ESP_OK) {
            
            // === DETECCIÓN DE POSTURA CON SISTEMA HÁPTICO INTELIGENTE ===
            current_angle = bmi270_calculate_tilt_angle(&imu_data);
            
            if (current_angle > POSTURE_THRESHOLD_DEGREES) {
                if (bad_posture_start == 0) {
                    bad_posture_start = esp_timer_get_time() / 1000;
                } else {
                    uint32_t duration = (esp_timer_get_time() / 1000) - bad_posture_start;
                    if (duration > POSTURE_ALERT_TIME_MS) {
                        // Usar sistema háptico inteligente con contexto
                        haptic_execute_pattern(HAPTIC_PATTERN_POSTURE_GENTLE_REMINDER);
                        storage_log_event("POSTURE_ALERT_HAPTIC", NULL);
                        ble_notify_posture_alert(current_angle);
                        
                        // Actualizar aprendizaje del usuario
                        haptic_update_user_response(HAPTIC_PATTERN_POSTURE_GENTLE_REMINDER, 0.8f);
                        
                        bad_posture_start = 0;
                        vTaskDelay(pdMS_TO_TICKS(POSTURE_COOLDOWN_MS));
                    }
                }
            } else {
                bad_posture_start = 0;
            }
            
            // === DETECCIÓN DE SEDENTARISMO CON PATRÓN ESPECÍFICO ===
            static uint32_t last_sedentary_check = 0;
            uint32_t now = esp_timer_get_time() / 1000;
            
            if (now - last_sedentary_check >= SEDENTARY_CHECK_INTERVAL_MS) {
                if (check_sedentary_behavior(&sedentary_monitor, &imu_data) == ESP_OK) {
                    // Ejecutar patrón de onda de urgencia neurocientífico
                    haptic_execute_pattern(HAPTIC_PATTERN_SEDENTARY_URGENCY_WAVE);
                    storage_log_event("SEDENTARY_ALERT_HAPTIC", NULL);
                    
                    ESP_LOGI("SEDENTARY", "Urgency wave pattern triggered after 1 hour inactivity");
                    
                    // Cooldown para evitar alertas repetitivas
                    vTaskDelay(pdMS_TO_TICKS(SEDENTARY_COOLDOWN_MS));
                }
                last_sedentary_check = now;
            }
            
            // === DETECCIÓN DE FLOW STATE ===
            static bool previous_flow_state = false;
            bool current_flow_state = detect_flow_state_from_movement(&imu_data);
            
            // Solo alertar cuando ENTRA en flow state (no durante)
            if (current_flow_state && !previous_flow_state) {
                haptic_execute_pattern(HAPTIC_PATTERN_FLOW_STATE_FOCUS_LOCK);
                storage_log_event("FLOW_STATE_DETECTED", NULL);
                ESP_LOGI("FLOW", "Flow state detected - subtle confirmation sent");
            }
            previous_flow_state = current_flow_state;
        }
        
        vTaskDelay(pdMS_TO_TICKS(POSTURE_CHECK_INTERVAL_MS));
    }
}

// Funciones auxiliares para integración háptica
esp_err_t haptic_configure_heartbeat_pattern(float partner_bpm) {
    if (partner_bpm < 50.0f || partner_bpm > 150.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calcular duración del latido basado en BPM real
    uint32_t beat_interval_ms = 60000.0f / partner_bpm;
    uint32_t beat_duration_ms = beat_interval_ms * 0.3f; // 30% del intervalo
    uint32_t pause_duration_ms = beat_interval_ms - beat_duration_ms;
    
    // Actualizar patrón dinámicamente
    haptic_pattern_def_t *heartbeat_pattern = &g_haptic_controller.patterns[HAPTIC_PATTERN_LOVE_RADAR_HEARTBEAT];
    heartbeat_pattern->pulses[0].duration_ms = beat_duration_ms;
    heartbeat_pattern->pulses[0].pause_after_ms = pause_duration_ms;
    heartbeat_pattern->total_duration_ms = beat_interval_ms * 5; // 5 latidos
    
    ESP_LOGD("HEARTBEAT", "Configured heartbeat: %.1f BPM, beat=%dms, pause=%dms", 
            partner_bpm, beat_duration_ms, pause_duration_ms);
    
    return ESP_OK;
}
```

## Modos de Operación

### Modo Autónomo (Offline)

El collar funciona independientemente sin conexión BLE o Wi-Fi activa.

**Características:**
- Muestreo continuo de sensores con algoritmos locales
- Alertas hápticas inmediatas
- Almacenamiento local de datos y eventos
- Radar BLE de pareja en segundo plano
- Gestión agresiva de energía

**Políticas de muestreo:**
- **MAX30102**: 25 Hz durante medición, intervalos de 5 min
- **MAX30205**: Cada 60 segundos
- **MAX86176**: Cada 10 minutos para hidratación
- **BME688**: Cada 2 minutos para calidad del aire
- **BMI270**: 50 Hz para postura continua
- **BMM150**: Cada 5 segundos

### Modo Conectado (BLE Activo)

Conexión activa con aplicación móvil para intercambio de datos en tiempo real.

**Funcionalidades:**
- Notificaciones BLE continuas
- Configuración remota de parámetros
- Visualización en tiempo real en la app
- Comandos de control desde el móvil
- Sincronización inmediata de eventos

### Modo Sincronización Wi-Fi

Activado automáticamente al detectar carga inalámbrica.

**Proceso de sincronización:**
1. Detección de base de carga
2. Activación de Wi-Fi station
3. Conexión a red doméstica
4. Upload de datos almacenados
5. Verificación de actualizaciones OTA
6. Sincronización de hora NTP
7. Desactivación de Wi-Fi tras completar

## Gestión Avanzada de Energía

### Estrategias de Ahorro

**Deep Sleep:**
- Activado cuando el collar no está en uso
- Consumo < 10 µA con ULP activo
- Wake-up por botón, movimiento o timer

**Light Sleep:**
- Durante pausas entre mediciones
- Configuración automática con Tickless Idle
- Reducción de frecuencia CPU dinámica

**Standby de Sensores:**
- Apagado selectivo según contexto
- Uso de modos de bajo consumo específicos
- Control de alimentación por GPIO cuando disponible

```c
// power_manager.h
typedef enum {
    POWER_MODE_ACTIVE,
    POWER_MODE_LIGHT_SLEEP,
    POWER_MODE_DEEP_SLEEP,
    POWER_MODE_CHARGING
} power_mode_t;

esp_err_t power_manager_init(void);
esp_err_t power_set_mode(power_mode_t mode);
uint8_t power_get_battery_percentage(void);
bool power_is_charging(void);
```

### Políticas de Gestión de Energía

**Modo Normal (Batería > 20%):**
- Todos los sensores activos según programación
- BLE radar de pareja activo
- Vibraciones sin restricción

**Modo Ahorro (Batería 10-20%):**
- Reducción de frecuencia de sensores ambientales
- Limitación de vibraciones
- Suspensión de radar de pareja

**Modo Crítico (Batería < 10%):**
- Solo sensores vitales (HR, postura)
- Alertas mínimas
- Preparación para deep sleep de emergencia

## Funcionalidades Principales

### 1. Monitoreo y Corrección de Postura

**Algoritmo de detección:**
```c
float calculate_posture_angle(imu_data_t *imu) {
    // Calcular ángulo de inclinación respecto a la vertical
    float magnitude = sqrt(imu->accel_x * imu->accel_x + 
                          imu->accel_y * imu->accel_y + 
                          imu->accel_z * imu->accel_z);
    
    // Asumiendo que Z apunta hacia abajo en buena postura
    float angle = acos(imu->accel_z / magnitude) * 180.0 / M_PI;
    return angle;
}
```

**Lógica de alertas:**
- Umbral: >15° de inclinación
- Tiempo sostenido: >2 minutos
- Frecuencia máxima: 1 alerta cada 5 minutos
- Patrón de vibración: Triple pulso corto

### 2. Detección de Estrés

**Basado en HRV (Heart Rate Variability):**
```c
float calculate_stress_index(float *rr_intervals, size_t count) {
    // Calcular RMSSD (Root Mean Square of Successive Differences)
    float sum_squared_diff = 0;
    for (int i = 1; i < count; i++) {
        float diff = rr_intervals[i] - rr_intervals[i-1];
        sum_squared_diff += diff * diff;
    }
    float rmssd = sqrt(sum_squared_diff / (count - 1));
    
    // Convertir a índice de estrés (0-100)
    // Menor RMSSD = mayor estrés
    float stress_index = 100.0 - (rmssd / 50.0) * 100.0;
    return CLAMP(stress_index, 0, 100);
}
```

**Respuestas al estrés:**
- Vibración rítmica para respiración guiada
- Notificación a la pareja (si configurado)
- Registro en memoria emocional

### 3. Monitor de Hidratación

**Algoritmo de estimación:**
- Análisis de señales PPG especializadas del MAX86176
- Correlación con factores ambientales (humedad, temperatura)
- Calibración personal basada en valores baseline

**Alertas proactivas:**
- Recordatorio cada 2 horas si hidratación normal
- Alerta inmediata si nivel < 40%
- Correlación con actividad física detectada

### 4. Monitoreo de Calidad del Aire

**Métricas del BME688:**
- Índice IAQ (0-500 usando librería BSEC)
- Detección de VOCs específicos
- Correlación con síntomas de fatiga

**Acciones correctivas:**
- Alerta si IAQ > 100 (calidad pobre)
- Sugerencia de ventilación
- Registro de exposición para análisis de salud

### 5. Detección de Flow State

**Criterios de detección:**
- HRV estable y elevado
- Frecuencia cardíaca moderadamente elevada pero constante
- Mínima variación en postura
- Duración > 20 minutos

**Implementación:**
```c
bool detect_flow_state(health_metrics_t *metrics, posture_data_t *posture) {
    static uint32_t flow_start_time = 0;
    
    bool stable_hr = (metrics->hr_variance < FLOW_HR_VARIANCE_THRESHOLD);
    bool good_hrv = (metrics->hrv > FLOW_HRV_THRESHOLD);
    bool stable_posture = (posture->movement_variance < FLOW_POSTURE_THRESHOLD);
    bool moderate_engagement = (metrics->heart_rate > metrics->baseline_hr + 10 &&
                               metrics->heart_rate < metrics->baseline_hr + 30);
    
    if (stable_hr && good_hrv && stable_posture && moderate_engagement) {
        if (flow_start_time == 0) {
            flow_start_time = esp_timer_get_time() / 1000;
        } else if ((esp_timer_get_time() / 1000 - flow_start_time) > FLOW_MIN_DURATION) {
            return true;
        }
    } else {
        flow_start_time = 0;
    }
    
    return false;
}
```

### 6. Radar Emocional y Comunicación de Pareja

**Estados emocionales detectados:**
```c
typedef enum {
    EMOTION_CALM_RELAXED,
    EMOTION_ENERGETIC_HAPPY,
    EMOTION_STRESSED_ANXIOUS,
    EMOTION_TIRED_DOWN,
    EMOTION_FOCUSED_FLOW
} emotional_state_t;

emotional_state_t classify_emotion(health_metrics_t *health, activity_data_t *activity) {
    float energy_level = (activity->movement_intensity + 
                         (health->heart_rate - health->baseline_hr)) / 2.0;
    float stress_level = health->stress_index;
    
    if (stress_level < 30) {
        return (energy_level > 50) ? EMOTION_ENERGETIC_HAPPY : EMOTION_CALM_RELAXED;
    } else if (stress_level > 70) {
        return (energy_level > 50) ? EMOTION_STRESSED_ANXIOUS : EMOTION_TIRED_DOWN;
    } else {
        // Estado intermedio, verificar flow
        if (health->flow_state) {
            return EMOTION_FOCUSED_FLOW;
        }
        return EMOTION_CALM_RELAXED;
    }
}
```

**Comunicación entre collares:**
- Advertising BLE con estado emocional codificado
- Detección mutua por RSSI para proximidad
- Vibración sincronizada automática o manual
- Respuesta empática (vibración calmante si pareja estresada)

### 7. Vibración Sincronizada

**Implementación de sincronización:**
```c
esp_err_t trigger_partner_vibration(vibration_pattern_t pattern) {
    // Vibrar localmente
    vibration_set_pattern(pattern);
    
    // Enviar señal a pareja via BLE advertising
    ble_advertise_vibration_trigger(pattern);
    
    // Registrar evento
    storage_log_event("PARTNER_VIBRATION", "SENT");
    
    return ESP_OK;
}

void partner_vibration_received_callback(vibration_pattern_t pattern) {
    // Ejecutar vibración sincronizada
    vibration_set_pattern(pattern);
    
    // Registrar evento
    storage_log_event("PARTNER_VIBRATION", "RECEIVED");
}
```

### 8. Memoria Emocional

**Estructura de datos:**
```c
typedef struct {
    uint32_t timestamp;
    emotional_state_t emotion;
    uint8_t stress_level;
    uint8_t energy_level;
    bool partner_present;
    char context[64];  // "work_time", "evening_together", etc.
} emotional_memory_entry_t;
```

**Almacenamiento y análisis:**
- Registro cada 5 minutos de estado emocional
- Correlación con presencia de pareja
- Identificación de patrones temporales
- Exportación para análisis en app/nube

### 9. Detección de Sedentarismo

**Objetivo:**
Detectar períodos prolongados de inactividad (>1 hora) y alertar al usuario mediante vibración para promover la actividad física y reducir los riesgos asociados al sedentarismo.

**Algoritmo de detección:**
```c
typedef struct {
    uint32_t last_movement_time;
    float activity_threshold;
    uint32_t sedentary_alert_interval;
    bool sedentary_alert_active;
    uint32_t daily_sedentary_time;
} sedentary_monitor_t;

bool detect_significant_movement(imu_data_t *imu) {
    static float prev_accel_magnitude = 0;
    
    // Calcular magnitud de aceleración
    float accel_magnitude = sqrt(imu->accel_x * imu->accel_x + 
                                imu->accel_y * imu->accel_y + 
                                imu->accel_z * imu->accel_z);
    
    // Calcular diferencia respecto a la muestra anterior
    float magnitude_diff = fabs(accel_magnitude - prev_accel_magnitude);
    prev_accel_magnitude = accel_magnitude;
    
    // Considerar movimiento significativo si supera el umbral
    return (magnitude_diff > SEDENTARY_MOVEMENT_THRESHOLD);
}

esp_err_t check_sedentary_behavior(sedentary_monitor_t *monitor, imu_data_t *imu) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    if (detect_significant_movement(imu)) {
        // Resetear timer de sedentarismo al detectar movimiento
        monitor->last_movement_time = current_time;
        monitor->sedentary_alert_active = false;
    } else {
        // Verificar tiempo transcurrido sin movimiento
        uint32_t sedentary_duration = current_time - monitor->last_movement_time;
        
        if (sedentary_duration >= SEDENTARY_ALERT_THRESHOLD_MS && 
            !monitor->sedentary_alert_active) {
            
            // Activar alerta de sedentarismo
            vibration_set_pattern(VIB_PATTERN_SEDENTARY_ALERT);
            storage_log_event("SEDENTARY_ALERT", NULL);
            
            if (ble_is_connected()) {
                ble_notify_sedentary_alert(sedentary_duration / 60000); // minutos
            }
            
            monitor->sedentary_alert_active = true;
            
            // Actualizar estadísticas diarias
            monitor->daily_sedentary_time += sedentary_duration;
            
            return ESP_OK;
        }
    }
    
    return ESP_ERR_NOT_FOUND; // No hay alerta
}
```

**Constantes de configuración:**
```c
#define SEDENTARY_MOVEMENT_THRESHOLD    0.1f    // g - umbral de movimiento significativo
#define SEDENTARY_ALERT_THRESHOLD_MS    3600000 // 1 hora en milisegundos
#define SEDENTARY_CHECK_INTERVAL_MS     10000   // verificar cada 10 segundos
#define SEDENTARY_COOLDOWN_MS           300000  // 5 minutos entre alertas
```

**Patrón de vibración específico:**
```c
// En vibration.h se añade:
VIB_PATTERN_SEDENTARY_ALERT,    // Patrón específico para sedentarismo

// En vibration.c se implementa:
case VIB_PATTERN_SEDENTARY_ALERT:
    // Secuencia: 3 pulsos largos + pausa + 2 pulsos cortos
    vibration_pulse(500, 100);  // Pulso largo
    vibration_pulse(500, 100);  // Pulso largo  
    vibration_pulse(500, 200);  // Pulso largo + pausa
    vibration_pulse(200, 100);  // Pulso corto
    vibration_pulse(200, 0);    // Pulso corto final
    break;
```

**Integración con task_postura.c:**
La detección de sedentarismo se integra en la tarea de postura existente para aprovechar los datos del IMU ya siendo procesados:

```c
void task_postura(void *pvParameters) {
    imu_data_t imu_data;
    float current_angle;
    uint32_t bad_posture_start = 0;
    
    // Inicializar monitor de sedentarismo
    sedentary_monitor_t sedentary_monitor = {
        .last_movement_time = esp_timer_get_time() / 1000,
        .activity_threshold = SEDENTARY_MOVEMENT_THRESHOLD,
        .sedentary_alert_interval = SEDENTARY_ALERT_THRESHOLD_MS,
        .sedentary_alert_active = false,
        .daily_sedentary_time = 0
    };
    
    while (1) {
        // Leer datos del IMU
        if (bmi270_read_data(&imu_data) == ESP_OK) {
            
            // === DETECCIÓN DE POSTURA EXISTENTE ===
            current_angle = bmi270_calculate_tilt_angle(&imu_data);
            
            if (current_angle > POSTURE_THRESHOLD_DEGREES) {
                if (bad_posture_start == 0) {
                    bad_posture_start = esp_timer_get_time() / 1000;
                } else {
                    uint32_t duration = (esp_timer_get_time() / 1000) - bad_posture_start;
                    if (duration > POSTURE_ALERT_TIME_MS) {
                        vibration_set_pattern(VIB_PATTERN_POSTURE_ALERT);
                        storage_log_event("POSTURE_ALERT", NULL);
                        ble_notify_posture_alert(current_angle);
                        
                        bad_posture_start = 0;
                        vTaskDelay(pdMS_TO_TICKS(POSTURE_COOLDOWN_MS));
                    }
                }
            } else {
                bad_posture_start = 0;
            }
            
            // === NUEVA DETECCIÓN DE SEDENTARISMO ===
            static uint32_t last_sedentary_check = 0;
            uint32_t now = esp_timer_get_time() / 1000;
            
            if (now - last_sedentary_check >= SEDENTARY_CHECK_INTERVAL_MS) {
                if (check_sedentary_behavior(&sedentary_monitor, &imu_data) == ESP_OK) {
                    // Alerta de sedentarismo activada
                    ESP_LOGI("SEDENTARY", "Alert triggered after 1 hour of inactivity");
                    
                    // Cooldown para evitar alertas repetitivas
                    vTaskDelay(pdMS_TO_TICKS(SEDENTARY_COOLDOWN_MS));
                }
                last_sedentary_check = now;
            }
            
            // Notificar estadísticas de sedentarismo vía BLE
            if (ble_is_connected()) {
                static uint32_t last_stats_update = 0;
                if (now - last_stats_update >= 300000) { // cada 5 minutos
                    ble_notify_sedentary_stats(sedentary_monitor.daily_sedentary_time);
                    last_stats_update = now;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(POSTURE_CHECK_INTERVAL_MS));
    }
}
```

**Características GATT adicionales:**
```c
// En ble_service.h se añaden:
#define SEDENTARY_STATUS_CHAR_UUID    0x3004  // Estado de sedentarismo
#define SEDENTARY_STATS_CHAR_UUID     0x3005  // Estadísticas diarias

// Funciones en ble_service.c
esp_err_t ble_notify_sedentary_alert(uint32_t minutes_inactive);
esp_err_t ble_notify_sedentary_stats(uint32_t daily_sedentary_time);
esp_err_t ble_set_sedentary_threshold(uint32_t threshold_minutes);
```

**Configuraciones personalizables:**
- **Umbral de tiempo**: Configurable desde 30 minutos hasta 2 horas
- **Sensibilidad de movimiento**: Ajustable según perfil de usuario
- **Horario de funcionamiento**: Desactivar durante horas de sueño
- **Días de la semana**: Permitir configuración de días laborales vs. fin de semana

**Métricas y análisis:**
- Tiempo total sedentario por día
- Frecuencia de alertas activadas vs. ignoradas
- Correlación con otros biomarcadores (estrés, fatiga)
- Patrones de actividad a lo largo del día
- Comparación con recomendaciones de salud (máximo 8 horas sedentarias)

**Integración con gestión de energía:**
La detección de sedentarismo se adapta al modo de energía:
- **Modo normal**: Verificación cada 10 segundos
- **Modo ahorro**: Verificación cada 30 segundos
- **Modo crítico**: Función desactivada para conservar batería

## Gestión del Botón SW5 y Entrada de Usuario

### Funcionalidades del Botón

**Pulsación corta (<1s):**
- Activar BLE advertising si está apagado
- Despertar de modo ahorro de energía

**Pulsación larga (>3s):**
- Reset de configuración BLE
- Borrar pairing information

**Doble pulsación rápida:**
- Enviar vibración sincronizada a pareja
- Activar "señal de cariño"

```c
// Implementación de detección de patrones
void button_interrupt_handler(void* arg) {
    static uint32_t last_press = 0;
    static uint32_t press_start = 0;
    static bool long_press_triggered = false;
    
    uint32_t now = esp_timer_get_time() / 1000;
    bool button_state = gpio_get_level(BUTTON_GPIO);
    
    if (!button_state) {  // Pressed (assuming active low)
        press_start = now;
        long_press_triggered = false;
        
        // Check for double press
        if ((now - last_press) < DOUBLE_PRESS_INTERVAL) {
            xEventGroupSetBits(button_events, BUTTON_DOUBLE_PRESS_BIT);
        }
    } else {  // Released
        uint32_t press_duration = now - press_start;
        
        if (press_duration > LONG_PRESS_DURATION && !long_press_triggered) {
            xEventGroupSetBits(button_events, BUTTON_LONG_PRESS_BIT);
        } else if (press_duration > DEBOUNCE_TIME) {
            xEventGroupSetBits(button_events, BUTTON_SHORT_PRESS_BIT);
        }
        
        last_press = now;
    }
}
```

## Soporte Multilingüe

### Implementación

**Configuración de idioma:**
```c
typedef enum {
    LANG_SPANISH = 0,
    LANG_ENGLISH = 1
} language_t;

// Almacenado en NVS
esp_err_t set_language(language_t lang) {
    return storage_set_config("language", &lang, sizeof(lang));
}

language_t get_language(void) {
    language_t lang = LANG_SPANISH;  // Default
    size_t size = sizeof(lang);
    storage_get_config("language", &lang, &size);
    return lang;
}
```

**Aplicación en firmware:**
- Logs de debug en idioma seleccionado
- Códigos de evento con strings localizados para export
- Configuración transmitida a app vía BLE para UI apropiada

## Estructura de Archivos del Proyecto

```
firmware_vitaminap/
├── main/
│   ├── main.c                    // Punto de entrada y configuración
│   ├── sensor_max30102.c/.h      // Driver sensor PPG
│   ├── sensor_max30205.c/.h      // Driver temperatura corporal
│   ├── sensor_max86176.c/.h      // Driver hidratación
│   ├── sensor_bme688.c/.h        // Driver ambiental
│   ├── sensor_bmi270.c/.h        // Driver IMU
│   ├── sensor_bmm150.c/.h        // Driver magnetómetro
│   ├── ble_service.c/.h          // Servicio Bluetooth LE
│   ├── wifi_sync.c/.h            // Sincronización Wi-Fi
│   ├── storage.c/.h              // Gestión de almacenamiento
│   ├── vibration.c/.h            // Control de vibración
│   ├── power_manager.c/.h        // Gestión de energía
│   ├── task_postura.c/.h         // Tarea de postura
│   ├── task_salud.c/.h           // Tarea de salud
│   ├── task_ambiente.c/.h        // Tarea ambiental
│   ├── task_pareja.c/.h          // Tarea de pareja
│   └── CMakeLists.txt
├── components/
│   └── bsec/                     // Librería BSEC para BME688
├── partitions.csv               // Tabla de particiones
├── sdkconfig                    // Configuración del proyecto
└── README.md
```

### Configuración de Particiones (partitions.csv)

```csv
# Name,   Type, SubType,   Offset,   Size,     Flags
nvs,      data, nvs,      0x9000,   0x6000,
otadata,  data, ota,      0xF000,   0x2000,
phy_init, data, phy,      0x11000,  0x1000,
ota_0,    app,  ota_0,    0x12000,  0x1A0000,
ota_1,    app,  ota_1,              0x1A0000,
spiffs,   data, spiffs,             0x100000
```

### Configuración Clave en sdkconfig

```ini
# Bluetooth
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_NIMBLE_MAX_CONNECTIONS=3

# Wi-Fi
CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE=y

# Power Management
CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
CONFIG_PM_ENABLE=y

# Partitions
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"

# SPIFFS
CONFIG_SPIFFS_MAX_PARTITIONS=3

# ULP
CONFIG_ULP_COPROC_ENABLED=y
```

## Implementación de Tareas Principales

### Task Postura (task_postura.c)

```c
void task_postura(void *pvParameters) {
    imu_data_t imu_data;
    float current_angle;
    uint32_t bad_posture_start = 0;
    
    // Inicializar monitor de sedentarismo
    sedentary_monitor_t sedentary_monitor = {
        .last_movement_time = esp_timer_get_time() / 1000,
        .activity_threshold = SEDENTARY_MOVEMENT_THRESHOLD,
        .sedentary_alert_interval = SEDENTARY_ALERT_THRESHOLD_MS,
        .sedentary_alert_active = false,
        .daily_sedentary_time = 0
    };
    
    while (1) {
        // Leer datos del IMU
        if (bmi270_read_data(&imu_data) == ESP_OK) {
            
            // === DETECCIÓN DE POSTURA EXISTENTE ===
            current_angle = bmi270_calculate_tilt_angle(&imu_data);
            
            if (current_angle > POSTURE_THRESHOLD_DEGREES) {
                if (bad_posture_start == 0) {
                    bad_posture_start = esp_timer_get_time() / 1000;
                } else {
                    uint32_t duration = (esp_timer_get_time() / 1000) - bad_posture_start;
                    if (duration > POSTURE_ALERT_TIME_MS) {
                        vibration_set_pattern(VIB_PATTERN_POSTURE_ALERT);
                        storage_log_event("POSTURE_ALERT", NULL);
                        ble_notify_posture_alert(current_angle);
                        
                        bad_posture_start = 0;
                        vTaskDelay(pdMS_TO_TICKS(POSTURE_COOLDOWN_MS));
                    }
                }
            } else {
                bad_posture_start = 0;
            }
            
            // === NUEVA DETECCIÓN DE SEDENTARISMO ===
            static uint32_t last_sedentary_check = 0;
            uint32_t now = esp_timer_get_time() / 1000;
            
            if (now - last_sedentary_check >= SEDENTARY_CHECK_INTERVAL_MS) {
                if (check_sedentary_behavior(&sedentary_monitor, &imu_data) == ESP_OK) {
                    // Alerta de sedentarismo activada
                    ESP_LOGI("SEDENTARY", "Alert triggered after 1 hour of inactivity");
                    
                    // Cooldown para evitar alertas repetitivas
                    vTaskDelay(pdMS_TO_TICKS(SEDENTARY_COOLDOWN_MS));
                }
                last_sedentary_check = now;
            }
            
            // Notificar estadísticas de sedentarismo vía BLE
            if (ble_is_connected()) {
                static uint32_t last_stats_update = 0;
                if (now - last_stats_update >= 300000) { // cada 5 minutos
                    ble_notify_sedentary_stats(sedentary_monitor.daily_sedentary_time);
                    last_stats_update = now;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(POSTURE_CHECK_INTERVAL_MS));
    }
}
```

### Task Salud (task_salud.c)

```c
void task_salud(void *pvParameters) {
    health_metrics_t metrics = {0};
    max30102_sample_t ppg_samples[32];
    size_t sample_count;
    
    while (1) {
        // Leer datos PPG
        if (max30102_read_fifo(ppg_samples, &sample_count) == ESP_OK && sample_count > 0) {
            // Calcular métricas vitales
            metrics.heart_rate = max30102_calculate_heart_rate(ppg_samples, sample_count);
            metrics.spo2 = max30102_calculate_spo2(ppg_samples, sample_count);
            metrics.hrv = calculate_hrv(ppg_samples, sample_count);
            metrics.stress_index = calculate_stress_index_from_hrv(metrics.hrv);
            
            // Leer temperatura corporal
            metrics.body_temperature = max30205_read_temperature();
            
            // Leer hidratación (menos frecuente)
            static uint32_t last_hydration_check = 0;
            uint32_t now = esp_timer_get_time() / 1000;
            if (now - last_hydration_check > HYDRATION_CHECK_INTERVAL_MS) {
                hydration_data_t hydration;
                if (max86176_read_hydration(&hydration) == ESP_OK) {
                    metrics.hydration_level = hydration.hydration_level;
                    last_hydration_check = now;
                }
            }
            
            // Detectar flow state
            metrics.flow_state = detect_flow_state(&metrics, &current_posture);
            
            // Evaluar alertas
            evaluate_health_alerts(&metrics);
            
            // Enviar datos por BLE si conectado
            if (ble_is_connected()) {
                ble_notify_health_metrics(&metrics);
            }
            
            // Guardar métricas
            storage_save_health_metrics(&metrics);
        }
        
        vTaskDelay(pdMS_TO_TICKS(HEALTH_CHECK_INTERVAL_MS));
    }
}
```

### Task Ambiente (task_ambiente.c)

```c
void task_ambiente(void *pvParameters) {
    bme688_data_t env_data;
    
    while (1) {
        // Forzar medición del BME688
        bme688_force_measurement();
        vTaskDelay(pdMS_TO_TICKS(BME688_MEASUREMENT_TIME_MS));
        
        if (bme688_read_all(&env_data) == ESP_OK) {
            // Evaluar calidad del aire
            if (env_data.iaq_index > IAQ_POOR_THRESHOLD) {
                vibration_set_pattern(VIB_PATTERN_AIR_QUALITY);
                storage_log_event("AIR_QUALITY_ALERT", NULL);
                
                if (ble_is_connected()) {
                    ble_notify_air_quality(env_data.iaq_index);
                }
            }
            
            // Correlacionar con fatiga si aire muy pobre
            if (env_data.iaq_index > IAQ_FATIGUE_CORRELATION_THRESHOLD) {
                // Marcar posible fatiga relacionada con ambiente
                storage_log_event("ENVIRONMENT_FATIGUE_CORRELATION", NULL);
            }
            
            // Guardar datos ambientales
            storage_save_environment_data(&env_data);
        }
        
        vTaskDelay(pdMS_TO_TICKS(ENVIRONMENT_CHECK_INTERVAL_MS));
    }
}
```

### Task Pareja (task_pareja.c)

```c
void task_pareja(void *pvParameters) {
    bool partner_present = false;
    emotional_state_t my_emotion, partner_emotion;
    
    while (1) {
        // Escanear por pareja
        bool partner_detected = ble_scan_for_partner();
        
        if (partner_detected && !partner_present) {
            // Pareja llegó - vibración suave de saludo
            vibration_set_pattern(VIB_PATTERN_PARTNER_HELLO);
            storage_log_event("PARTNER_ARRIVED", NULL);
            partner_present = true;
        } else if (!partner_detected && partner_present) {
            // Pareja se fue
            storage_log_event("PARTNER_LEFT", NULL);
            partner_present = false;
        }
        
        if (partner_present) {
            // Intercambiar estados emocionales
            my_emotion = get_current_emotional_state();
            partner_emotion = ble_get_partner_emotion();
            
            // Respuesta empática
            if (partner_emotion == EMOTION_STRESSED_ANXIOUS && 
                my_emotion == EMOTION_CALM_RELAXED) {
                // Ofrecer apoyo - vibración calmante
                vibration_set_pattern(VIB_PATTERN_CALMING);
            }
            
            // Registrar interacción emocional
            storage_log_emotional_interaction(my_emotion, partner_emotion);
        }
        
        vTaskDelay(pdMS_TO_TICKS(PARTNER_CHECK_INTERVAL_MS));
    }
}
```

## Conclusiones

El firmware del collar inteligente Vitamina P representa una solución integral que combina:

1. **Arquitectura modular robusta** basada en ESP-IDF y FreeRTOS
2. **Sensores avanzados** para monitoreo completo de bienestar
3. **Algoritmos inteligentes** para detección de estados fisiológicos y emocionales
4. **Comunicación innovadora** entre parejas via BLE directo
5. **Gestión eficiente de energía** para autonomía extendida
6. **Almacenamiento local** con sincronización inteligente
7. **Interfaz háptica** rica para comunicación sin pantalla

### Innovaciones Clave

- **Eliminación del ECG AD8232** simplificando el diseño sin perder capacidades cardíacas
- **Estimación de hidratación** mediante MAX86176 usando técnicas ópticas avanzadas
- **Radar emocional de pareja** con comunicación BLE directa
- **Detección de flow state** usando métricas fisiológicas combinadas
- **Memoria emocional** para análisis de patrones a largo plazo
- **Detección inteligente de sedentarismo** con alertas preventivas después de 1 hora de inactividad
- **Sistema háptico neurocientífico revolucionario** con 10+ patrones diferenciados e inteligencia adaptativa
- **Calibración personal automática** con aprendizaje de preferencias del usuario en tiempo real
- **Memoria emocional háptica** para reproducir patrones de momentos especiales

### Ventajas Técnicas

- **Multitarea real** con núcleos dedicados para estabilidad
- **Conectividad dual** BLE para tiempo real, Wi-Fi para sincronización
- **Algoritmos optimizados** para procesamiento local sin depender de la nube
- **Actualizaciones OTA** para evolución continua del firmware
- **Escalabilidad** para agregar nuevos sensores o funcionalidades
- **Inteligencia háptica contextual** que se adapta automáticamente al estado del usuario
- **Comunicación no verbal avanzada** entre parejas mediante patrones de latido sincronizado
- **Sistema de aprendizaje automático** que personaliza la experiencia sin intervención manual

El firmware resultante proporciona una base sólida para un dispositivo wearable de próxima generación, enfocado en el bienestar integral y la conexión emocional entre parejas, manteniendo los más altos estándares de calidad técnica y eficiencia energética.

**Sistema BLE para Configuración y Calibración Háptica:**

```c
// ble_haptic_service.c

#include "ble_service.h"
#include "advanced_haptic_system.h"
#include "esp_gatts_api.h"
#include "esp_log.h"

// Estructura para comandos de configuración háptica
typedef struct {
    uint8_t command_type;
    uint8_t pattern_id;
    uint8_t intensity;
    float sensitivity;
    uint8_t context;
    char event_name[32];
} haptic_ble_command_t;

// Tipos de comandos BLE para sistema háptico
typedef enum {
    HAPTIC_CMD_SET_INTENSITY = 0x01,
    HAPTIC_CMD_SET_SENSITIVITY = 0x02,
    HAPTIC_CMD_SET_CONTEXT = 0x03,
    HAPTIC_CMD_ENABLE_LEARNING = 0x04,
    HAPTIC_CMD_TEST_PATTERN = 0x05,
    HAPTIC_CMD_START_CALIBRATION = 0x06,
    HAPTIC_CMD_SAVE_MEMORY = 0x07,
    HAPTIC_CMD_REPLAY_MEMORY = 0x08,
    HAPTIC_CMD_GET_LEARNING_DATA = 0x09,
    HAPTIC_CMD_RESET_LEARNING = 0x0A,
    HAPTIC_CMD_SET_CIRCADIAN = 0x0B,
    HAPTIC_CMD_EMERGENCY_TEST = 0x0C
} haptic_ble_command_type_t;

// Handler para configuración háptica vía BLE
esp_err_t ble_haptic_config_handler(uint16_t conn_id, uint16_t attr_handle, 
                                   uint8_t *data, uint16_t len) {
    if (len < sizeof(haptic_ble_command_t)) {
        ESP_LOGW("BLE_HAPTIC", "Invalid command length: %d", len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    haptic_ble_command_t *cmd = (haptic_ble_command_t *)data;
    esp_err_t result = ESP_OK;
    
    ESP_LOGI("BLE_HAPTIC", "Received command type: 0x%02X", cmd->command_type);
    
    switch (cmd->command_type) {
        case HAPTIC_CMD_SET_INTENSITY:
            result = haptic_set_pattern_intensity((haptic_pattern_t)cmd->pattern_id, cmd->intensity);
            ESP_LOGI("BLE_HAPTIC", "Set pattern %d intensity to %d%%", cmd->pattern_id, cmd->intensity);
            break;
            
        case HAPTIC_CMD_SET_SENSITIVITY:
            result = haptic_set_sensitivity(cmd->sensitivity);
            ESP_LOGI("BLE_HAPTIC", "Set user sensitivity to %.2f", cmd->sensitivity);
            break;
            
        case HAPTIC_CMD_SET_CONTEXT:
            result = haptic_set_context((user_context_t)cmd->context);
            ESP_LOGI("BLE_HAPTIC", "Set context to %d", cmd->context);
            break;
            
        case HAPTIC_CMD_ENABLE_LEARNING:
            result = haptic_enable_adaptive_learning(cmd->intensity > 0);
            ESP_LOGI("BLE_HAPTIC", "Adaptive learning %s", cmd->intensity > 0 ? "enabled" : "disabled");
            break;
            
        case HAPTIC_CMD_TEST_PATTERN:
            if (cmd->pattern_id < HAPTIC_PATTERN_MAX) {
                result = haptic_execute_pattern((haptic_pattern_t)cmd->pattern_id);
                ESP_LOGI("BLE_HAPTIC", "Testing pattern %d", cmd->pattern_id);
                
                // Notificar test en progreso
                ble_notify_haptic_test_status(cmd->pattern_id, true);
            } else {
                result = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case HAPTIC_CMD_START_CALIBRATION:
            result = haptic_start_calibration();
            ESP_LOGI("BLE_HAPTIC", "Starting haptic calibration process");
            break;
            
        case HAPTIC_CMD_SAVE_MEMORY:
            if (strlen(cmd->event_name) > 0) {
                // Capturar patrón actual y guardarlo como memoria especial
                result = haptic_save_current_as_memory(cmd->event_name);
                ESP_LOGI("BLE_HAPTIC", "Saving memory pattern: %s", cmd->event_name);
            } else {
                result = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case HAPTIC_CMD_REPLAY_MEMORY:
            if (strlen(cmd->event_name) > 0) {
                result = haptic_replay_memory_pattern(cmd->event_name);
                ESP_LOGI("BLE_HAPTIC", "Replaying memory pattern: %s", cmd->event_name);
            } else {
                result = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case HAPTIC_CMD_GET_LEARNING_DATA:
            // Preparar datos de aprendizaje para transmisión
            result = ble_prepare_learning_data_transmission();
            break;
            
        case HAPTIC_CMD_RESET_LEARNING:
            result = haptic_reset_learning_data();
            ESP_LOGI("BLE_HAPTIC", "Reset learning data");
            break;
            
        case HAPTIC_CMD_SET_CIRCADIAN:
            result = haptic_enable_circadian_adjustment(cmd->intensity > 0);
            ESP_LOGI("BLE_HAPTIC", "Circadian adjustment %s", cmd->intensity > 0 ? "enabled" : "disabled");
            break;
            
        case HAPTIC_CMD_EMERGENCY_TEST:
            // Test de patrones de emergencia con override
            result = haptic_test_emergency_patterns();
            ESP_LOGI("BLE_HAPTIC", "Testing emergency patterns");
            break;
            
        default:
            ESP_LOGW("BLE_HAPTIC", "Unknown command type: 0x%02X", cmd->command_type);
            result = ESP_ERR_NOT_SUPPORTED;
            break;
    }
    
    // Enviar respuesta con resultado
    uint8_t response[4] = {cmd->command_type, (result == ESP_OK) ? 0x01 : 0x00, 0, 0};
    esp_ble_gatts_send_indicate(g_gatts_if, conn_id, attr_handle, sizeof(response), response, false);
    
    return result;
}

// Proceso de calibración háptica personalizada
esp_err_t haptic_start_calibration(void) {
    ESP_LOGI("HAPTIC_CAL", "Starting personal haptic calibration");
    
    // Secuencia de calibración de 7 patrones con intensidades crecientes
    static const haptic_pattern_t calibration_patterns[] = {
        HAPTIC_PATTERN_POSTURE_GENTLE_REMINDER,
        HAPTIC_PATTERN_HYDRATION_AQUA_DROP,
        HAPTIC_PATTERN_AIR_QUALITY_WIND_WHISPER,
        HAPTIC_PATTERN_SEDENTARY_URGENCY_WAVE,
        HAPTIC_PATTERN_STRESS_BREATH_SYNC,
        HAPTIC_PATTERN_LOVE_RADAR_HEARTBEAT,
        HAPTIC_PATTERN_OPTIMAL_ENVIRONMENT_SPARKLE
    };
    
    // Ejecutar cada patrón con intensidad creciente
    for (int i = 0; i < 7; i++) {
        uint8_t intensity = 20 + (i * 10); // 20%, 30%, 40%, 50%, 60%, 70%, 80%
        
        ESP_LOGI("HAPTIC_CAL", "Calibration step %d: pattern %d at %d%% intensity", 
                i + 1, calibration_patterns[i], intensity);
        
        // Notificar app el paso actual
        ble_notify_calibration_step(i + 1, calibration_patterns[i], intensity);
        
        // Ejecutar patrón con intensidad específica
        haptic_execute_pattern_with_intensity(calibration_patterns[i], 
                                            (haptic_intensity_level_t)(intensity / 20));
        
        // Esperar 3 segundos entre patrones para respuesta del usuario
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    
    ESP_LOGI("HAPTIC_CAL", "Calibration sequence completed");
    ble_notify_calibration_complete();
    
    return ESP_OK;
}

// Guardar patrón actual como memoria especial
esp_err_t haptic_save_current_as_memory(const char* event_name) {
    // Crear patrón personalizado basado en contexto actual
    haptic_pattern_def_t memory_pattern = {0};
    
    // Obtener métricas actuales para crear patrón único
    float current_hr = get_current_heart_rate();
    user_context_t current_context = haptic_get_current_context();
    uint32_t timestamp = esp_timer_get_time() / 1000000;
    
    // Generar patrón basado en estado emocional actual
    if (current_context == USER_CONTEXT_COUPLE_TIME) {
        // Patrón romántico basado en heartbeat actual
        memory_pattern.pulses[0] = (haptic_pulse_t){
            .duration_ms = (uint16_t)(60000 / current_hr * 0.3f),
            .intensity_percent = 35,
            .pause_after_ms = (uint16_t)(60000 / current_hr * 0.7f),
            .use_texture = false,
            .texture_frequency_hz = 0
        };
        memory_pattern.pulse_count = 1;
        memory_pattern.loop_pattern = true;
        memory_pattern.loop_count = 3;
        memory_pattern.base_intensity = HAPTIC_CONSCIOUS_SOFT;
    } else {
        // Patrón general personalizado
        memory_pattern.pulses[0] = (haptic_pulse_t){200, 40, 100, false, 0};
        memory_pattern.pulses[1] = (haptic_pulse_t){150, 30, 100, false, 0};
        memory_pattern.pulses[2] = (haptic_pulse_t){200, 40, 0, false, 0};
        memory_pattern.pulse_count = 3;
        memory_pattern.loop_pattern = false;
        memory_pattern.loop_count = 1;
        memory_pattern.base_intensity = HAPTIC_CONSCIOUS_SOFT;
    }
    
    // Guardar en memoria no volátil
    esp_err_t result = haptic_save_memory_pattern(event_name, &memory_pattern);
    
    if (result == ESP_OK) {
        ESP_LOGI("HAPTIC_MEMORY", "Saved memory pattern '%s' at timestamp %lu", event_name, timestamp);
        storage_log_event("HAPTIC_MEMORY_SAVED", event_name);
    }
    
    return result;
}

// Notificaciones BLE para sistema háptico
esp_err_t ble_notify_haptic_responsiveness(haptic_pattern_t pattern) {
    if (!ble_is_connected()) return ESP_ERR_INVALID_STATE;
    
    float responsiveness = haptic_get_user_responsiveness(pattern);
    uint32_t daily_count = haptic_get_daily_pattern_count(pattern);
    
    uint8_t notification_data[8] = {
        pattern,                           // ID del patrón
        (uint8_t)(responsiveness * 100),   // Responsiveness 0-100
        (daily_count >> 8) & 0xFF,        // Contador diario (high byte)
        daily_count & 0xFF,               // Contador diario (low byte)
        0, 0, 0, 0                        // Reservado para futuro uso
    };
    
    esp_err_t result = esp_ble_gatts_send_indicate(
        g_gatts_if, 
        g_conn_id, 
        g_haptic_responsiveness_handle,
        sizeof(notification_data),
        notification_data,
        false
    );
    
    ESP_LOGD("BLE_HAPTIC", "Notified responsiveness for pattern %d: %.1f%% (%lu uses)", 
            pattern, responsiveness * 100, daily_count);
    
    return result;
}

esp_err_t ble_notify_haptic_context(user_context_t context) {
    if (!ble_is_connected()) return ESP_ERR_INVALID_STATE;
    
    uint32_t timestamp = esp_timer_get_time() / 1000000;
    
    uint8_t notification_data[8] = {
        context,                          // Contexto actual
        haptic_get_context_confidence(),  // Confianza de detección 0-100
        (timestamp >> 24) & 0xFF,        // Timestamp
        (timestamp >> 16) & 0xFF,
        (timestamp >> 8) & 0xFF,
        timestamp & 0xFF,
        0, 0                             // Reservado
    };
    
    esp_err_t result = esp_ble_gatts_send_indicate(
        g_gatts_if,
        g_conn_id,
        g_haptic_context_handle,
        sizeof(notification_data),
        notification_data,
        false
    );
    
    ESP_LOGD("BLE_HAPTIC", "Notified context change to %d at %lu", context, timestamp);
    
    return result;
}

esp_err_t ble_notify_calibration_step(uint8_t step, haptic_pattern_t pattern, uint8_t intensity) {
    if (!ble_is_connected()) return ESP_ERR_INVALID_STATE;
    
    uint8_t notification_data[6] = {
        0x01,       // Calibration in progress
        step,       // Step number (1-7)
        pattern,    // Pattern being tested
        intensity,  // Intensity percentage
        0, 0        // Reservado
    };
    
    return esp_ble_gatts_send_indicate(
        g_gatts_if,
        g_conn_id,
        g_haptic_calibration_handle,
        sizeof(notification_data),
        notification_data,
        false
    );
}

esp_err_t ble_notify_calibration_complete(void) {
    if (!ble_is_connected()) return ESP_ERR_INVALID_STATE;
    
    uint8_t notification_data[6] = {
        0x02,       // Calibration complete
        0x07,       // Total steps completed
        0xFF,       // Success indicator
        haptic_get_learned_sensitivity(),
        0, 0        // Reservado
    };
    
    ESP_LOGI("BLE_HAPTIC", "Calibration completed - learned sensitivity: %d", 
            haptic_get_learned_sensitivity());
    
    return esp_ble_gatts_send_indicate(
        g_gatts_if,
        g_conn_id,
        g_haptic_calibration_handle,
        sizeof(notification_data),
        notification_data,
        false
    );
}

// Preparar transmisión de datos de aprendizaje
esp_err_t ble_prepare_learning_data_transmission(void) {
    static char learning_data_buffer[512];
    size_t buffer_size = sizeof(learning_data_buffer);
    
    esp_err_t result = haptic_export_learning_data(learning_data_buffer, buffer_size);
    if (result != ESP_OK) {
        ESP_LOGE("BLE_HAPTIC", "Failed to export learning data");
        return result;
    }
    
    // Transmitir datos en chunks de 20 bytes (límite MTU típico)
    size_t data_len = strlen(learning_data_buffer);
    size_t offset = 0;
    uint8_t chunk_id = 0;
    
    while (offset < data_len) {
        size_t chunk_size = (data_len - offset > 18) ? 18 : (data_len - offset);
        
        uint8_t chunk_packet[20] = {0};
        chunk_packet[0] = chunk_id++;
        chunk_packet[1] = (offset + chunk_size >= data_len) ? 0xFF : 0x00; // Último chunk
        
        memcpy(&chunk_packet[2], &learning_data_buffer[offset], chunk_size);
        
        esp_ble_gatts_send_indicate(
            g_gatts_if,
            g_conn_id,
            g_haptic_learning_data_handle,
            2 + chunk_size,
            chunk_packet,
            false
        );
        
        offset += chunk_size;
        vTaskDelay(pdMS_TO_TICKS(50)); // Pequeña pausa entre chunks
    }
    
    ESP_LOGI("BLE_HAPTIC", "Learning data transmitted: %d bytes in %d chunks", data_len, chunk_id);
    
    return ESP_OK;
}

// Comandos de configuración avanzada desde app móvil
typedef struct {
    char command[32];
    char parameters[64];
} haptic_text_command_t;

esp_err_t ble_haptic_text_command_handler(const char* command_string) {
    haptic_text_command_t cmd = {0};
    
    // Parsear comando de texto (ej: "SET_INTENSITY POSTURE 75")
    sscanf(command_string, "%31s %63s", cmd.command, cmd.parameters);
    
    ESP_LOGI("BLE_HAPTIC", "Text command: %s %s", cmd.command, cmd.parameters);
    
    if (strcmp(cmd.command, "HAPTIC_CALIBRATE_START") == 0) {
        return haptic_start_calibration();
    }
    else if (strcmp(cmd.command, "SET_PATTERN_INTENSITY") == 0) {
        char pattern_name[32];
        int intensity;
        if (sscanf(cmd.parameters, "%31s %d", pattern_name, &intensity) == 2) {
            haptic_pattern_t pattern = haptic_pattern_name_to_id(pattern_name);
            return haptic_set_pattern_intensity(pattern, (uint8_t)intensity);
        }
    }
    else if (strcmp(cmd.command, "ENABLE_ADAPTIVE_LEARNING") == 0) {
        bool enable = (strcmp(cmd.parameters, "ON") == 0);
        return haptic_enable_adaptive_learning(enable);
    }
    else if (strcmp(cmd.command, "SET_CONTEXT_MODE") == 0) {
        if (strcmp(cmd.parameters, "AUTO") == 0) {
            return haptic_set_context(USER_CONTEXT_AUTO_DETECT);
        } else if (strcmp(cmd.parameters, "MANUAL") == 0) {
            // Mantener contexto actual sin auto-detección
            return ESP_OK;
        }
    }
    else if (strcmp(cmd.command, "HAPTIC_SENSITIVITY_TEST") == 0) {
        return haptic_sensitivity_test_sequence();
    }
    else if (strcmp(cmd.command, "PATTERN_VARIATION_ENABLE") == 0) {
        char pattern_name[32];
        if (sscanf(cmd.parameters, "%31s", pattern_name) == 1) {
            haptic_pattern_t pattern = haptic_pattern_name_to_id(pattern_name);
            return haptic_enable_pattern_variation(pattern, true);
        }
    }
    else if (strcmp(cmd.command, "EMERGENCY_PATTERN_TEST") == 0) {
        return haptic_test_emergency_patterns();
    }
    else if (strcmp(cmd.command, "MEMORY_HAPTIC_SAVE") == 0) {
        return haptic_save_current_as_memory(cmd.parameters);
    }
    else if (strcmp(cmd.command, "PREDICTIVE_ALERTS") == 0) {
        bool enable = (strcmp(cmd.parameters, "ON") == 0);
        return haptic_enable_predictive_alerts(enable);
    }
    
    ESP_LOGW("BLE_HAPTIC", "Unknown text command: %s", cmd.command);
    return ESP_ERR_NOT_SUPPORTED;
}