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
- **Configuración granular completa** con control individual de 40+ funciones específicas
- **Sistema de privacidad avanzado** con modo privado temporal y control de compartición
- **Perfiles predefinidos inteligentes** para diferentes estilos de vida y necesidades
- **Modos contextuales automáticos** que se adaptan a trabajo, sueño, ejercicio y vida social
- **Alertas de cuidado proactivo** con análisis predictivo del bienestar de la pareja
- **Compartición inteligente de datos** con configuración personalizable por tipo de información

### Ventajas Técnicas

- **Multitarea real** con núcleos dedicados para estabilidad
- **Conectividad dual** BLE para tiempo real, Wi-Fi para sincronización
- **Algoritmos optimizados** para procesamiento local sin depender de la nube
- **Actualizaciones OTA** para evolución continua del firmware
- **Escalabilidad** para agregar nuevos sensores o funcionalidades
- **Inteligencia háptica contextual** que se adapta automáticamente al estado del usuario
- **Comunicación no verbal avanzada** entre parejas mediante patrones de latido sincronizado
- **Sistema de aprendizaje automático** que personaliza la experiencia sin intervención manual
- **Arquitectura de configuración modular** que permite personalización extrema sin complejidad
- **Protección de privacidad líder en la industria** con control granular de datos compartidos
- **Detección contextual inteligente** que anticipa necesidades antes de que surjan
- **Sistema de cuidado mutuo** que fortalece vínculos emocionales entre parejas

El firmware resultante establece un nuevo estándar en la industria de wearables, creando **el dispositivo más personalizable y respetuoso de la privacidad del mercado**. Con su sistema de configuración granular, inteligencia adaptativa y enfoque en el cuidado mutuo, el collar Vitamina P trasciende las limitaciones de los wearables tradicionales para ofrecer una experiencia verdaderamente personalizada que evoluciona con las necesidades únicas de cada usuario y pareja.

**El resultado es revolucionario:** un wearable que no solo monitorea la salud, sino que **comprende, anticipa y cuida** activamente el bienestar físico y emocional de sus usuarios, estableciendo un nuevo paradigma en tecnología wearable centrada en el ser humano.

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
```

## Sistema de Configuración Granular y Privacidad Avanzada

### Configuración Individual de Funciones

Sistema completo de personalización que permite al usuario controlar cada aspecto del collar con granularidad extrema.

```c
// granular_config_system.h

#include "esp_err.h"
#include "storage.h"
#include "advanced_haptic_system.h"
#include <stdbool.h>
#include <stdint.h>

// Configuración granular de funciones individuales
typedef struct {
    // Sensores básicos
    bool heart_rate_monitoring;           // Monitoreo de ritmo cardíaco
    bool spo2_monitoring;                 // Oximetría
    bool temperature_monitoring;          // Temperatura corporal
    bool hydration_monitoring;            // Nivel de hidratación
    bool posture_monitoring;              // Detección de postura
    bool movement_tracking;               // Seguimiento de movimiento
    bool sedentary_detection;            // Detección de sedentarismo
    bool sleep_tracking;                 // Seguimiento del sueño
    
    // Funciones ambientales
    bool air_quality_monitoring;         // Calidad del aire
    bool ambient_temperature_tracking;   // Temperatura ambiental
    bool humidity_monitoring;            // Humedad relativa
    bool voc_detection;                  // Compuestos orgánicos volátiles
    
    // Funciones de bienestar
    bool stress_detection;               // Detección de estrés
    bool hrv_analysis;                   // Análisis de variabilidad cardíaca
    bool flow_state_detection;          // Detección de estado de flow
    bool energy_level_tracking;         // Seguimiento de nivel de energía
    bool mood_analysis;                  // Análisis de estado de ánimo
    
    // Alertas y notificaciones
    bool posture_alerts;                 // Alertas de postura
    bool hydration_reminders;            // Recordatorios de hidratación
    bool movement_reminders;             // Recordatorios de movimiento
    bool stress_relief_guidance;        // Guía para alivio del estrés
    bool breathing_exercises;            // Ejercicios de respiración
    bool environmental_alerts;           // Alertas ambientales
    
    // Funciones de pareja
    bool partner_presence_detection;     // Detección de presencia de pareja
    bool emotional_sharing;              // Compartir estado emocional
    bool heartbeat_sharing;              // Compartir latidos
    bool stress_alerts_to_partner;      // Alertas de estrés a pareja
    bool location_sharing;               // Compartir ubicación aproximada
    bool emergency_contact_notification; // Notificación de emergencia
    
    // Sistema háptico
    bool haptic_feedback;                // Retroalimentación háptica general
    bool adaptive_intensity;             // Intensidad adaptativa
    bool contextual_patterns;            // Patrones contextuales
    bool learning_mode;                  // Modo de aprendizaje
    bool pattern_variations;             // Variaciones de patrones
    
    // Conectividad
    bool bluetooth_connectivity;         // Conectividad Bluetooth
    bool wifi_sync;                      // Sincronización WiFi
    bool cloud_backup;                   // Respaldo en la nube
    bool automatic_updates;              // Actualizaciones automáticas
    
    // Privacidad y seguridad
    bool data_encryption;                // Cifrado de datos
    bool anonymous_analytics;            // Analíticas anónimas
    bool location_services;              // Servicios de ubicación
    bool voice_commands;                 // Comandos de voz (futuro)
} granular_config_t;

// Perfiles predefinidos de usuario
typedef enum {
    USER_PROFILE_CUSTOM = 0,            // Personalizado por usuario
    USER_PROFILE_MINIMALIST,            // Minimalista - solo esencial
    USER_PROFILE_ATHLETIC,              // Atlético - enfoque fitness
    USER_PROFILE_COUPLE,                // Pareja - máxima conectividad
    USER_PROFILE_PROFESSIONAL,         // Profesional - trabajo enfocado
    USER_PROFILE_WELLNESS,              // Bienestar - salud integral
    USER_PROFILE_PRIVACY_FOCUSED,       // Privacidad - mínimo compartir
    USER_PROFILE_SOCIAL,                // Social - máxima interacción
    USER_PROFILE_SENIOR,                // Senior - simplicidad y seguridad
    USER_PROFILE_MAX
} user_profile_type_t;

// Configuración de compartición de datos
typedef struct {
    // Datos básicos compartibles
    bool share_heart_rate;              // Compartir ritmo cardíaco
    bool share_stress_level;            // Compartir nivel de estrés
    bool share_energy_level;            // Compartir nivel de energía
    bool share_mood;                    // Compartir estado de ánimo
    bool share_activity_status;         // Compartir estado de actividad
    bool share_sleep_status;            // Compartir estado de sueño
    bool share_location_zone;           // Compartir zona de ubicación
    
    // Eventos especiales compartibles
    bool share_stress_alerts;           // Compartir alertas de estrés
    bool share_emergency_events;        // Compartir eventos de emergencia
    bool share_achievement_milestones;  // Compartir logros
    bool share_flow_state_events;       // Compartir estados de flow
    bool share_wellness_insights;       // Compartir insights de bienestar
    
    // Configuración de frecuencia
    uint32_t sharing_frequency_ms;      // Frecuencia de compartición
    bool real_time_sharing;             // Compartición en tiempo real
    bool batch_sharing;                 // Compartición por lotes
    
    // Filtros de compartición
    uint8_t stress_threshold_to_share;  // Umbral de estrés para compartir
    uint8_t energy_threshold_to_share;  // Umbral de energía para compartir
    bool share_only_significant_changes; // Solo cambios significativos
    
    // Configuración temporal
    uint32_t sharing_start_time;        // Hora de inicio de compartición
    uint32_t sharing_end_time;          // Hora de fin de compartición
    bool weekends_different_sharing;    // Compartición diferente fines de semana
} data_sharing_config_t;

// Modos contextuales automáticos
typedef struct {
    // Configuración de modos
    bool auto_work_mode;                // Modo trabajo automático
    bool auto_sleep_mode;               // Modo sueño automático
    bool auto_exercise_mode;            // Modo ejercicio automático
    bool auto_social_mode;              // Modo social automático
    bool auto_focus_mode;               // Modo concentración automático
    bool auto_relaxation_mode;          // Modo relajación automático
    
    // Umbrales de detección
    uint32_t work_hours_start;          // Inicio horas laborales
    uint32_t work_hours_end;            // Fin horas laborales
    float exercise_detection_threshold; // Umbral detección ejercicio
    uint32_t sleep_detection_time;      // Tiempo para detectar sueño
    float social_activity_threshold;    // Umbral actividad social
    
    // Configuraciones por modo
    granular_config_t work_mode_config;       // Config modo trabajo
    granular_config_t sleep_mode_config;      // Config modo sueño
    granular_config_t exercise_mode_config;   // Config modo ejercicio
    granular_config_t social_mode_config;     // Config modo social
    granular_config_t focus_mode_config;      // Config modo concentración
    granular_config_t relaxation_mode_config; // Config modo relajación
} contextual_modes_config_t;

// Configuración temporal
typedef struct {
    // Modo privado temporal
    bool private_mode_active;           // Modo privado activo
    uint32_t private_mode_start_time;   // Inicio modo privado
    uint32_t private_mode_duration_ms;  // Duración modo privado
    bool auto_exit_private_mode;        // Salir automáticamente
    
    // Desactivación temporal de funciones
    uint64_t disabled_functions_mask;   // Máscara funciones desactivadas
    uint32_t temp_disable_start_time;   // Inicio desactivación temporal
    uint32_t temp_disable_duration_ms;  // Duración desactivación
    
    // Programación de funciones
    bool scheduled_mode_changes;        // Cambios programados de modo
    uint32_t scheduled_mode_times[8];   // Horarios programados
    user_profile_type_t scheduled_profiles[8]; // Perfiles programados
    
    // Modo de emergencia
    bool emergency_override_active;     // Override de emergencia activo
    granular_config_t emergency_config; // Configuración de emergencia
} temporal_config_t;

// Alertas de cuidado proactivo
typedef struct {
    // Configuración de alertas para pareja
    bool partner_stress_alert;          // Alerta estrés de pareja
    bool partner_health_concern_alert;  // Alerta preocupación salud pareja
    bool partner_low_energy_alert;      // Alerta baja energía pareja
    bool partner_sleep_disturbance_alert; // Alerta alteración sueño pareja
    bool partner_inactivity_alert;      // Alerta inactividad pareja
    
    // Umbrales de alerta
    uint8_t stress_alert_threshold;     // Umbral alerta estrés
    uint32_t inactivity_alert_time_ms;  // Tiempo para alerta inactividad
    uint8_t energy_low_threshold;       // Umbral energía baja
    
    // Tipos de respuesta
    bool gentle_reminder_response;      // Respuesta recordatorio suave
    bool direct_notification_response;  // Respuesta notificación directa
    bool haptic_empathy_response;       // Respuesta háptica empática
    bool suggested_actions_response;    // Respuesta acciones sugeridas
    
    // Personalización de mensajes
    char custom_stress_message[64];     // Mensaje personalizado estrés
    char custom_support_message[64];    // Mensaje personalizado apoyo
    char custom_care_message[64];       // Mensaje personalizado cuidado
} proactive_care_config_t;

// Estructura principal de configuración
typedef struct {
    granular_config_t granular_settings;
    data_sharing_config_t sharing_settings;
    contextual_modes_config_t contextual_settings;
    temporal_config_t temporal_settings;
    proactive_care_config_t proactive_care_settings;
    
    user_profile_type_t current_profile;
    uint32_t config_version;
    uint32_t last_modified_timestamp;
    bool factory_reset_available;
} master_config_t;

// Funciones principales del sistema de configuración
esp_err_t granular_config_init(void);
esp_err_t granular_config_load_from_nvs(master_config_t *config);
esp_err_t granular_config_save_to_nvs(const master_config_t *config);
esp_err_t granular_config_set_profile(user_profile_type_t profile);
esp_err_t granular_config_apply_profile(user_profile_type_t profile, master_config_t *config);

// Funciones de configuración individual
esp_err_t granular_config_enable_function(uint32_t function_id, bool enable);
esp_err_t granular_config_set_sharing_preference(uint32_t data_type, bool share);
esp_err_t granular_config_set_contextual_mode(user_context_t context, bool auto_enable);

// Funciones de modo privado y temporal
esp_err_t granular_config_enable_private_mode(uint32_t duration_minutes);
esp_err_t granular_config_disable_private_mode(void);
esp_err_t granular_config_disable_function_temporarily(uint32_t function_id, uint32_t duration_minutes);
esp_err_t granular_config_schedule_profile_change(uint32_t time_hour_minute, user_profile_type_t profile);

// Funciones de cuidado proactivo
esp_err_t proactive_care_check_partner_status(void);
esp_err_t proactive_care_send_support_alert(const char* message);
esp_err_t proactive_care_configure_alert_threshold(uint32_t alert_type, uint8_t threshold);

// Funciones utilitarias
bool granular_config_is_function_enabled(uint32_t function_id);
bool granular_config_is_sharing_enabled(uint32_t data_type);
user_profile_type_t granular_config_get_current_profile(void);
esp_err_t granular_config_factory_reset(void);
esp_err_t granular_config_export_settings(char *json_buffer, size_t buffer_size);
esp_err_t granular_config_import_settings(const char *json_string);
```

**Implementación de Perfiles Predefinidos:**

```c
// user_profiles.c

// Definición de perfiles predefinidos
static void init_minimalist_profile(master_config_t *config) {
    // Perfil minimalista - solo funciones esenciales
    memset(&config->granular_settings, 0, sizeof(granular_config_t));
    
    // Solo funciones básicas habilitadas
    config->granular_settings.heart_rate_monitoring = true;
    config->granular_settings.movement_tracking = true;
    config->granular_settings.sedentary_detection = true;
    config->granular_settings.posture_alerts = true;
    config->granular_settings.haptic_feedback = true;
    config->granular_settings.emergency_contact_notification = true;
    
    // Configuración de compartición mínima
    config->sharing_settings.share_emergency_events = true;
    config->sharing_settings.sharing_frequency_ms = 300000; // 5 minutos
    config->sharing_settings.real_time_sharing = false;
    
    ESP_LOGI("PROFILE", "Minimalist profile configured");
}

static void init_athletic_profile(master_config_t *config) {
    // Perfil atlético - enfoque en fitness y rendimiento
    memset(&config->granular_settings, 0, sizeof(granular_config_t));
    
    // Todas las funciones de salud y rendimiento
    config->granular_settings.heart_rate_monitoring = true;
    config->granular_settings.spo2_monitoring = true;
    config->granular_settings.temperature_monitoring = true;
    config->granular_settings.hydration_monitoring = true;
    config->granular_settings.movement_tracking = true;
    config->granular_settings.hrv_analysis = true;
    config->granular_settings.energy_level_tracking = true;
    config->granular_settings.stress_detection = true;
    config->granular_settings.breathing_exercises = true;
    
    // Alertas de rendimiento
    config->granular_settings.hydration_reminders = true;
    config->granular_settings.movement_reminders = true;
    config->granular_settings.stress_relief_guidance = true;
    
    // Modo ejercicio automático habilitado
    config->contextual_settings.auto_exercise_mode = true;
    config->contextual_settings.exercise_detection_threshold = 1.5f;
    
    ESP_LOGI("PROFILE", "Athletic profile configured");
}

static void init_couple_profile(master_config_t *config) {
    // Perfil pareja - máxima conectividad y compartición
    memset(&config->granular_settings, 0xff, sizeof(granular_config_t));
    
    // Todas las funciones habilitadas
    // Configuración de compartición completa
    config->sharing_settings.share_heart_rate = true;
    config->sharing_settings.share_stress_level = true;
    config->sharing_settings.share_energy_level = true;
    config->sharing_settings.share_mood = true;
    config->sharing_settings.share_activity_status = true;
    config->sharing_settings.share_sleep_status = true;
    config->sharing_settings.share_stress_alerts = true;
    config->sharing_settings.share_emergency_events = true;
    config->sharing_settings.share_wellness_insights = true;
    config->sharing_settings.real_time_sharing = true;
    config->sharing_settings.sharing_frequency_ms = 30000; // 30 segundos
    
    // Cuidado proactivo completo
    config->proactive_care_settings.partner_stress_alert = true;
    config->proactive_care_settings.partner_health_concern_alert = true;
    config->proactive_care_settings.partner_low_energy_alert = true;
    config->proactive_care_settings.gentle_reminder_response = true;
    config->proactive_care_settings.haptic_empathy_response = true;
    
    ESP_LOGI("PROFILE", "Couple profile configured");
}

static void init_professional_profile(master_config_t *config) {
    // Perfil profesional - enfoque en productividad
    memset(&config->granular_settings, 0, sizeof(granular_config_t));
    
    // Funciones de bienestar y productividad
    config->granular_settings.heart_rate_monitoring = true;
    config->granular_settings.stress_detection = true;
    config->granular_settings.flow_state_detection = true;
    config->granular_settings.posture_monitoring = true;
    config->granular_settings.air_quality_monitoring = true;
    config->granular_settings.posture_alerts = true;
    config->granular_settings.stress_relief_guidance = true;
    config->granular_settings.breathing_exercises = true;
    
    // Modo trabajo automático
    config->contextual_settings.auto_work_mode = true;
    config->contextual_settings.auto_focus_mode = true;
    config->contextual_settings.work_hours_start = 9 * 60; // 9:00 AM
    config->contextual_settings.work_hours_end = 17 * 60;  // 5:00 PM
    
    // Compartición limitada durante trabajo
    config->sharing_settings.share_stress_alerts = true;
    config->sharing_settings.share_emergency_events = true;
    config->sharing_settings.sharing_frequency_ms = 600000; // 10 minutos
    
    ESP_LOGI("PROFILE", "Professional profile configured");
}

static void init_privacy_focused_profile(master_config_t *config) {
    // Perfil enfocado en privacidad - mínimo compartir
    memset(&config->granular_settings, 0, sizeof(granular_config_t));
    
    // Solo funciones locales esenciales
    config->granular_settings.heart_rate_monitoring = true;
    config->granular_settings.posture_monitoring = true;
    config->granular_settings.movement_tracking = true;
    config->granular_settings.stress_detection = true;
    config->granular_settings.haptic_feedback = true;
    
    // Sin compartición excepto emergencias
    memset(&config->sharing_settings, 0, sizeof(data_sharing_config_t));
    config->sharing_settings.share_emergency_events = true;
    config->sharing_settings.sharing_frequency_ms = 86400000; // 24 horas
    
    // Sin analíticas ni respaldo en nube
    config->granular_settings.anonymous_analytics = false;
    config->granular_settings.cloud_backup = false;
    config->granular_settings.location_services = false;
    
    ESP_LOGI("PROFILE", "Privacy-focused profile configured");
}

esp_err_t granular_config_apply_profile(user_profile_type_t profile, master_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    switch (profile) {
        case USER_PROFILE_MINIMALIST:
            init_minimalist_profile(config);
            break;
        case USER_PROFILE_ATHLETIC:
            init_athletic_profile(config);
            break;
        case USER_PROFILE_COUPLE:
            init_couple_profile(config);
            break;
        case USER_PROFILE_PROFESSIONAL:
            init_professional_profile(config);
            break;
        case USER_PROFILE_PRIVACY_FOCUSED:
            init_privacy_focused_profile(config);
            break;
        case USER_PROFILE_CUSTOM:
            // No cambiar configuración para perfil personalizado
            break;
        default:
            ESP_LOGW("PROFILE", "Unknown profile type: %d", profile);
            return ESP_ERR_INVALID_ARG;
    }
    
    config->current_profile = profile;
    config->last_modified_timestamp = esp_timer_get_time() / 1000000;
    
    return granular_config_save_to_nvs(config);
}
```

**Sistema de Compartición Inteligente de Datos:**

```c
// intelligent_data_sharing.c

#include "granular_config_system.h"
#include "ble_service.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

// Estado de emparejamiento de collares
typedef struct {
    char partner_device_id[32];         // ID único del collar de la pareja
    bool is_paired;                     // Estado de emparejamiento
    uint32_t last_seen_timestamp;       // Última vez que se vio la pareja
    uint8_t connection_strength;        // Fuerza de conexión (RSSI)
    bool real_time_connection_active;   // Conexión en tiempo real activa
    uint32_t total_shared_data_bytes;   // Total de datos compartidos
    uint32_t last_sync_timestamp;       // Última sincronización completa
} partner_pairing_state_t;

// Paquete de datos compartidos
typedef struct {
    uint32_t timestamp;                 // Timestamp del dato
    uint8_t data_type;                  // Tipo de dato
    uint8_t priority;                   // Prioridad (0=baja, 255=emergencia)
    uint16_t data_size;                 // Tamaño de los datos
    uint8_t data_payload[64];           // Payload de datos
    uint32_t checksum;                  // Checksum para integridad
    bool requires_acknowledgment;       // Requiere confirmación
} shared_data_packet_t;

// Cola de datos para compartir
typedef struct {
    shared_data_packet_t packets[32];   // Buffer circular de paquetes
    uint8_t write_index;                // Índice de escritura
    uint8_t read_index;                 // Índice de lectura
    uint8_t packet_count;               // Número de paquetes en cola
    SemaphoreHandle_t queue_mutex;      // Mutex para acceso concurrente
} data_sharing_queue_t;

// Tipos de datos compartibles
typedef enum {
    SHARED_DATA_HEART_RATE = 0x01,
    SHARED_DATA_STRESS_LEVEL = 0x02,
    SHARED_DATA_ENERGY_LEVEL = 0x03,
    SHARED_DATA_MOOD_STATE = 0x04,
    SHARED_DATA_ACTIVITY_STATUS = 0x05,
    SHARED_DATA_SLEEP_STATUS = 0x06,
    SHARED_DATA_LOCATION_ZONE = 0x07,
    SHARED_DATA_EMERGENCY_ALERT = 0x08,
    SHARED_DATA_STRESS_ALERT = 0x09,
    SHARED_DATA_WELLNESS_INSIGHT = 0x0A,
    SHARED_DATA_ACHIEVEMENT = 0x0B,
    SHARED_DATA_FLOW_STATE = 0x0C,
    SHARED_DATA_CUSTOM_MESSAGE = 0x0D,
    SHARED_DATA_HAPTIC_SYNC = 0x0E,
    SHARED_DATA_CARE_REQUEST = 0x0F
} shared_data_type_t;

// Estado global del sistema de compartición
static partner_pairing_state_t g_partner_state = {0};
static data_sharing_queue_t g_sharing_queue = {0};
static master_config_t *g_current_config = NULL;
static TaskHandle_t g_data_sharing_task = NULL;

// Inicialización del sistema de compartición
esp_err_t intelligent_data_sharing_init(master_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    g_current_config = config;
    
    // Inicializar mutex de la cola
    g_sharing_queue.queue_mutex = xSemaphoreCreateMutex();
    if (g_sharing_queue.queue_mutex == NULL) {
        ESP_LOGE("DATA_SHARE", "Failed to create queue mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Inicializar estado de pareja desde NVS
    load_partner_pairing_state();
    
    // Crear tarea de procesamiento de compartición
    BaseType_t task_created = xTaskCreatePinnedToCore(
        data_sharing_task,
        "data_sharing",
        4096,
        NULL,
        configMAX_PRIORITIES - 3,
        &g_data_sharing_task,
        1  // Core 1
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE("DATA_SHARE", "Failed to create data sharing task");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI("DATA_SHARE", "Intelligent data sharing system initialized");
    return ESP_OK;
}

// Tarea principal de compartición de datos
static void data_sharing_task(void *pvParameters) {
    TickType_t last_routine_share = 0;
    TickType_t last_partner_scan = 0;
    
    ESP_LOGI("DATA_SHARE", "Data sharing task started");
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // Escanear por pareja cada 10 segundos
        if (current_time - last_partner_scan > pdMS_TO_TICKS(10000)) {
            scan_for_partner();
            last_partner_scan = current_time;
        }
        
        // Procesar cola de datos para compartir
        process_data_sharing_queue();
        
        // Compartición rutinaria según configuración
        if (g_current_config->sharing_settings.real_time_sharing ||
            (current_time - last_routine_share > pdMS_TO_TICKS(g_current_config->sharing_settings.sharing_frequency_ms))) {
            
            perform_routine_data_sharing();
            last_routine_share = current_time;
        }
        
        // Verificar si la pareja necesita cuidado proactivo
        check_partner_care_needs();
        
        // Procesar mensajes recibidos de la pareja
        process_received_partner_data();
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Verificar cada segundo
    }
}

// Escanear por collar de la pareja
static esp_err_t scan_for_partner(void) {
    if (!g_current_config->granular_settings.partner_presence_detection) {
        return ESP_OK; // Función deshabilitada
    }
    
    // Escanear dispositivos BLE cercanos
    esp_ble_gap_start_scanning(5); // 5 segundos de escaneo
    
    // Aquí se implementaría la lógica de reconocimiento del collar de la pareja
    // basada en el device_id almacenado
    
    return ESP_OK;
}

// Añadir datos a la cola de compartición
esp_err_t intelligent_share_data(shared_data_type_t data_type, const void *data, size_t data_size, uint8_t priority) {
    if (!data || data_size == 0 || data_size > 64) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Verificar si este tipo de dato está habilitado para compartir
    if (!is_data_type_shareable(data_type)) {
        ESP_LOGD("DATA_SHARE", "Data type %d not enabled for sharing", data_type);
        return ESP_OK; // No es error, simplemente no se comparte
    }
    
    // Verificar si estamos en modo privado
    if (g_current_config->temporal_settings.private_mode_active) {
        if (data_type != SHARED_DATA_EMERGENCY_ALERT) {
            ESP_LOGD("DATA_SHARE", "Private mode active, not sharing data type %d", data_type);
            return ESP_OK;
        }
    }
    
    if (xSemaphoreTake(g_sharing_queue.queue_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW("DATA_SHARE", "Failed to acquire queue mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Verificar si hay espacio en la cola
    if (g_sharing_queue.packet_count >= 32) {
        ESP_LOGW("DATA_SHARE", "Sharing queue full, dropping oldest packet");
        // Avanzar read_index para hacer espacio
        g_sharing_queue.read_index = (g_sharing_queue.read_index + 1) % 32;
        g_sharing_queue.packet_count--;
    }
    
    // Crear paquete de datos
    shared_data_packet_t *packet = &g_sharing_queue.packets[g_sharing_queue.write_index];
    packet->timestamp = esp_timer_get_time() / 1000;
    packet->data_type = data_type;
    packet->priority = priority;
    packet->data_size = data_size;
    memcpy(packet->data_payload, data, data_size);
    packet->checksum = calculate_checksum(packet);
    packet->requires_acknowledgment = (priority > 200); // Alta prioridad requiere ACK
    
    // Avanzar índices
    g_sharing_queue.write_index = (g_sharing_queue.write_index + 1) % 32;
    g_sharing_queue.packet_count++;
    
    xSemaphoreGive(g_sharing_queue.queue_mutex);
    
    ESP_LOGD("DATA_SHARE", "Queued data type %d for sharing (priority %d)", data_type, priority);
    
    return ESP_OK;
}

// Verificar si un tipo de dato es compartible según configuración
static bool is_data_type_shareable(shared_data_type_t data_type) {
    data_sharing_config_t *config = &g_current_config->sharing_settings;
    
    switch (data_type) {
        case SHARED_DATA_HEART_RATE:
            return config->share_heart_rate;
        case SHARED_DATA_STRESS_LEVEL:
            return config->share_stress_level;
        case SHARED_DATA_ENERGY_LEVEL:
            return config->share_energy_level;
        case SHARED_DATA_MOOD_STATE:
            return config->share_mood;
        case SHARED_DATA_ACTIVITY_STATUS:
            return config->share_activity_status;
        case SHARED_DATA_SLEEP_STATUS:
            return config->share_sleep_status;
        case SHARED_DATA_LOCATION_ZONE:
            return config->share_location_zone;
        case SHARED_DATA_EMERGENCY_ALERT:
            return config->share_emergency_events;
        case SHARED_DATA_STRESS_ALERT:
            return config->share_stress_alerts;
        case SHARED_DATA_WELLNESS_INSIGHT:
            return config->share_wellness_insights;
        case SHARED_DATA_ACHIEVEMENT:
            return config->share_achievement_milestones;
        case SHARED_DATA_FLOW_STATE:
            return config->share_flow_state_events;
        default:
            return false;
    }
}

// Procesar cola de compartición
static void process_data_sharing_queue(void) {
    if (g_sharing_queue.packet_count == 0 || !g_partner_state.is_paired) {
        return;
    }
    
    if (xSemaphoreTake(g_sharing_queue.queue_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    
    // Procesar hasta 5 paquetes por iteración para no bloquear
    int packets_processed = 0;
    while (g_sharing_queue.packet_count > 0 && packets_processed < 5) {
        shared_data_packet_t *packet = &g_sharing_queue.packets[g_sharing_queue.read_index];
        
        // Enviar paquete vía BLE
        esp_err_t result = send_data_packet_to_partner(packet);
        
        if (result == ESP_OK) {
            // Paquete enviado exitosamente
            g_sharing_queue.read_index = (g_sharing_queue.read_index + 1) % 32;
            g_sharing_queue.packet_count--;
            packets_processed++;
            
            ESP_LOGD("DATA_SHARE", "Successfully sent data type %d to partner", packet->data_type);
        } else {
            ESP_LOGW("DATA_SHARE", "Failed to send data packet: %d", result);
            break; // Salir del bucle si falla el envío
        }
    }
    
    xSemaphoreGive(g_sharing_queue.queue_mutex);
}

// Enviar paquete de datos a la pareja vía BLE
static esp_err_t send_data_packet_to_partner(const shared_data_packet_t *packet) {
    if (!packet || !g_partner_state.real_time_connection_active) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Serializar paquete para transmisión BLE
    uint8_t ble_packet[80]; // Header + datos
    size_t packet_size = serialize_data_packet(packet, ble_packet, sizeof(ble_packet));
    
    // Enviar vía característica BLE de compartición de datos
    esp_err_t result = esp_ble_gatts_send_indicate(
        g_gatts_if,
        g_partner_connection_id,
        g_data_sharing_char_handle,
        packet_size,
        ble_packet,
        packet->requires_acknowledgment
    );
    
    if (result == ESP_OK) {
        g_partner_state.total_shared_data_bytes += packet_size;
    }
    
    return result;
}

// Compartición rutinaria de datos vitales
static void perform_routine_data_sharing(void) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Verificar ventana de tiempo para compartición
    if (!is_sharing_time_window_active(current_time)) {
        return;
    }
    
    // Compartir datos vitales básicos si están habilitados
    if (g_current_config->sharing_settings.share_heart_rate) {
        uint8_t hr = get_current_heart_rate();
        intelligent_share_data(SHARED_DATA_HEART_RATE, &hr, sizeof(hr), 50);
    }
    
    if (g_current_config->sharing_settings.share_stress_level) {
        uint8_t stress = get_current_stress_level();
        // Solo compartir si supera el umbral configurado
        if (stress >= g_current_config->sharing_settings.stress_threshold_to_share) {
            intelligent_share_data(SHARED_DATA_STRESS_LEVEL, &stress, sizeof(stress), 100);
        }
    }
    
    if (g_current_config->sharing_settings.share_energy_level) {
        uint8_t energy = get_current_energy_level();
        if (energy <= g_current_config->sharing_settings.energy_threshold_to_share) {
            intelligent_share_data(SHARED_DATA_ENERGY_LEVEL, &energy, sizeof(energy), 75);
        }
    }
    
    if (g_current_config->sharing_settings.share_activity_status) {
        uint8_t activity = get_current_activity_status();
        intelligent_share_data(SHARED_DATA_ACTIVITY_STATUS, &activity, sizeof(activity), 25);
    }
}

// Verificar si estamos en ventana de tiempo para compartir
static bool is_sharing_time_window_active(uint32_t current_time_seconds) {
    // Convertir a minutos desde medianoche
    uint32_t minutes_since_midnight = (current_time_seconds % 86400) / 60;
    
    uint32_t start_minutes = g_current_config->sharing_settings.sharing_start_time;
    uint32_t end_minutes = g_current_config->sharing_settings.sharing_end_time;
    
    // Si start == end, compartición habilitada todo el día
    if (start_minutes == end_minutes) {
        return true;
    }
    
    // Verificar si estamos en el rango de tiempo
    if (start_minutes < end_minutes) {
        // Rango normal (ej: 8:00 - 22:00)
        return (minutes_since_midnight >= start_minutes && minutes_since_midnight <= end_minutes);
    } else {
        // Rango que cruza medianoche (ej: 22:00 - 8:00)
        return (minutes_since_midnight >= start_minutes || minutes_since_midnight <= end_minutes);
    }
}

// Funciones de API pública para compartición específica
esp_err_t share_emergency_alert(const char* emergency_type, const char* location) {
    char emergency_data[64];
    snprintf(emergency_data, sizeof(emergency_data), "%s|%s", emergency_type, location);
    
    return intelligent_share_data(SHARED_DATA_EMERGENCY_ALERT, emergency_data, 
                                strlen(emergency_data), 255); // Máxima prioridad
}

esp_err_t share_stress_alert_to_partner(uint8_t stress_level, const char* context) {
    if (!g_current_config->sharing_settings.share_stress_alerts) {
        return ESP_OK;
    }
    
    struct {
        uint8_t stress_level;
        char context[32];
    } stress_data = {
        .stress_level = stress_level
    };
    strncpy(stress_data.context, context ? context : "", sizeof(stress_data.context) - 1);
    
    return intelligent_share_data(SHARED_DATA_STRESS_ALERT, &stress_data, 
                                sizeof(stress_data), 200);
}

esp_err_t share_wellness_insight(const char* insight_type, float value, const char* recommendation) {
    if (!g_current_config->sharing_settings.share_wellness_insights) {
        return ESP_OK;
    }
    
    struct {
        char type[16];
        float value;
        char recommendation[32];
    } insight_data;
    
    strncpy(insight_data.type, insight_type, sizeof(insight_data.type) - 1);
    insight_data.value = value;
    strncpy(insight_data.recommendation, recommendation ? recommendation : "", 
            sizeof(insight_data.recommendation) - 1);
    
    return intelligent_share_data(SHARED_DATA_WELLNESS_INSIGHT, &insight_data, 
                                sizeof(insight_data), 150);
}

esp_err_t share_custom_care_message(const char* message) {
    if (!message || strlen(message) == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return intelligent_share_data(SHARED_DATA_CUSTOM_MESSAGE, message, 
                                strlen(message), 180);
}

// Emparejamiento de collares
esp_err_t pair_with_partner_collar(const char* partner_device_id, const char* pairing_code) {
    if (!partner_device_id || !pairing_code) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Verificar código de emparejamiento (implementación segura)
    if (!verify_pairing_code(partner_device_id, pairing_code)) {
        ESP_LOGW("DATA_SHARE", "Invalid pairing code for device %s", partner_device_id);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Establecer emparejamiento
    strncpy(g_partner_state.partner_device_id, partner_device_id, 
            sizeof(g_partner_state.partner_device_id) - 1);
    g_partner_state.is_paired = true;
    g_partner_state.last_seen_timestamp = esp_timer_get_time() / 1000;
    
    // Guardar estado de emparejamiento en NVS
    save_partner_pairing_state();
    
    ESP_LOGI("DATA_SHARE", "Successfully paired with partner: %s", partner_device_id);
    
    return ESP_OK;
}

esp_err_t unpair_from_partner(void) {
    memset(&g_partner_state, 0, sizeof(partner_pairing_state_t));
    save_partner_pairing_state();
    
    ESP_LOGI("DATA_SHARE", "Unpaired from partner collar");
    
    return ESP_OK;
}
```

**Sistema de Alertas de Cuidado Proactivo:**

```c
// proactive_care_system.c

#include "granular_config_system.h"
#include "intelligent_data_sharing.h"
#include "advanced_haptic_system.h"
#include "esp_log.h"
#include <math.h>

// Estado de bienestar de la pareja
typedef struct {
    // Métricas de salud actuales
    uint8_t stress_level;               // Nivel de estrés 0-100
    uint8_t energy_level;               // Nivel de energía 0-100
    uint8_t mood_score;                 // Puntuación de ánimo 0-100
    uint32_t last_activity_time;        // Última actividad detectada
    uint32_t sleep_quality_score;       // Calidad de sueño 0-100
    bool is_in_distress;                // Estado de angustia detectado
    
    // Tendencias y patrones
    float stress_trend;                 // Tendencia de estrés (+/- por hora)
    float energy_trend;                 // Tendencia de energía (+/- por hora)
    uint32_t consecutive_high_stress_time; // Tiempo consecutivo con estrés alto
    uint32_t inactivity_duration;       // Duración de inactividad actual
    
    // Historial para análisis predictivo
    uint8_t stress_history[24];         // Historial de estrés (24 horas)
    uint8_t energy_history[24];         // Historial de energía (24 horas)
    uint8_t current_hour_index;         // Índice hora actual en historial
    
    // Estado de alertas enviadas
    bool stress_alert_sent;             // Alerta de estrés ya enviada
    bool energy_alert_sent;             // Alerta de energía ya enviada
    bool inactivity_alert_sent;         // Alerta de inactividad ya enviada
    uint32_t last_care_message_time;    // Última vez que se envió mensaje de cuidado
} partner_wellness_state_t;

// Tipos de alertas de cuidado
typedef enum {
    CARE_ALERT_STRESS_SUPPORT = 0x01,      // Apoyo por estrés
    CARE_ALERT_ENERGY_BOOST = 0x02,        // Impulso de energía
    CARE_ALERT_ACTIVITY_ENCOURAGE = 0x03,  // Estímulo de actividad
    CARE_ALERT_MOOD_LIFT = 0x04,           // Mejorar ánimo
    CARE_ALERT_SLEEP_CONCERN = 0x05,       // Preocupación por sueño
    CARE_ALERT_HEALTH_CHECK = 0x06,        // Verificación de salud
    CARE_ALERT_GENTLE_REMINDER = 0x07,     // Recordatorio suave
    CARE_ALERT_EMERGENCY_CHECK = 0x08      // Verificación de emergencia
} care_alert_type_t;

// Acciones de cuidado sugeridas
typedef struct {
    care_alert_type_t alert_type;
    char suggested_action[64];
    char care_message[128];
    uint8_t urgency_level;              // 0-255
    uint32_t estimated_help_duration;   // Tiempo estimado de ayuda (minutos)
} care_action_suggestion_t;

// Estado global del sistema de cuidado proactivo
static partner_wellness_state_t g_partner_wellness = {0};
static master_config_t *g_care_config = NULL;
static TaskHandle_t g_proactive_care_task = NULL;

// Inicialización del sistema de cuidado proactivo
esp_err_t proactive_care_system_init(master_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    g_care_config = config;
    
    // Inicializar historial de wellness
    memset(&g_partner_wellness, 0, sizeof(partner_wellness_state_t));
    
    // Crear tarea de monitoreo proactivo
    BaseType_t task_created = xTaskCreatePinnedToCore(
        proactive_care_monitoring_task,
        "proactive_care",
        4096,
        NULL,
        configMAX_PRIORITIES - 4,
        &g_proactive_care_task,
        1  // Core 1
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE("PROACTIVE_CARE", "Failed to create proactive care task");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI("PROACTIVE_CARE", "Proactive care system initialized");
    return ESP_OK;
}

// Tarea principal de monitoreo proactivo
static void proactive_care_monitoring_task(void *pvParameters) {
    TickType_t last_wellness_analysis = 0;
    TickType_t last_trend_calculation = 0;
    TickType_t last_partner_check = 0;
    
    ESP_LOGI("PROACTIVE_CARE", "Proactive care monitoring task started");
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // Verificar estado de la pareja cada 30 segundos
        if (current_time - last_partner_check > pdMS_TO_TICKS(30000)) {
            proactive_care_check_partner_status();
            last_partner_check = current_time;
        }
        
        // Análisis de bienestar cada 2 minutos
        if (current_time - last_wellness_analysis > pdMS_TO_TICKS(120000)) {
            analyze_partner_wellness_state();
            last_wellness_analysis = current_time;
        }
        
        // Cálculo de tendencias cada 10 minutos
        if (current_time - last_trend_calculation > pdMS_TO_TICKS(600000)) {
            calculate_wellness_trends();
            last_trend_calculation = current_time;
        }
        
        // Verificar si necesita enviar alertas de cuidado
        check_and_send_care_alerts();
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // Verificar cada 10 segundos
    }
}

// Verificar estado actual del partner
esp_err_t proactive_care_check_partner_status(void) {
    if (!g_care_config->granular_settings.partner_presence_detection ||
        !g_partner_state.is_paired) {
        return ESP_OK; // Sistema deshabilitado o no hay pareja
    }
    
    // Obtener datos más recientes del partner
    partner_wellness_state_t *wellness = &g_partner_wellness;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Actualizar métricas desde datos compartidos recibidos
    wellness->stress_level = get_partner_stress_level();
    wellness->energy_level = get_partner_energy_level();
    wellness->mood_score = get_partner_mood_score();
    
    // Detectar inactividad prolongada
    uint32_t last_activity = get_partner_last_activity_time();
    if (last_activity > 0) {
        wellness->inactivity_duration = current_time - last_activity;
    }
    
    // Actualizar historial cada hora
    struct tm timeinfo;
    localtime_r((time_t*)&current_time, &timeinfo);
    uint8_t current_hour = timeinfo.tm_hour;
    
    if (current_hour != wellness->current_hour_index) {
        wellness->stress_history[current_hour] = wellness->stress_level;
        wellness->energy_history[current_hour] = wellness->energy_level;
        wellness->current_hour_index = current_hour;
    }
    
    ESP_LOGD("PROACTIVE_CARE", "Partner status: stress=%d, energy=%d, mood=%d, inactive=%lus", 
            wellness->stress_level, wellness->energy_level, wellness->mood_score, 
            wellness->inactivity_duration);
    
    return ESP_OK;
}

// Analizar estado de bienestar de la pareja
static void analyze_partner_wellness_state(void) {
    partner_wellness_state_t *wellness = &g_partner_wellness;
    proactive_care_config_t *care_config = &g_care_config->proactive_care_settings;
    
    // Detectar estrés sostenido alto
    if (wellness->stress_level >= care_config->stress_alert_threshold) {
        if (wellness->consecutive_high_stress_time == 0) {
            wellness->consecutive_high_stress_time = esp_timer_get_time() / 1000;
        }
    } else {
        wellness->consecutive_high_stress_time = 0;
        wellness->stress_alert_sent = false; // Reset alerta
    }
    
    // Detectar problemas de energía persistentes
    if (wellness->energy_level <= care_config->energy_low_threshold) {
        if (!wellness->energy_alert_sent) {
            // Verificar si es un patrón o algo temporal
            float avg_energy_last_6h = calculate_average_energy_last_hours(6);
            if (avg_energy_last_6h <= care_config->energy_low_threshold) {
                wellness->energy_alert_sent = true;
            }
        }
    } else if (wellness->energy_level > care_config->energy_low_threshold + 20) {
        wellness->energy_alert_sent = false; // Reset si mejora significativamente
    }
    
    // Detectar inactividad prolongada
    if (wellness->inactivity_duration >= care_config->inactivity_alert_time_ms / 1000) {
        if (!wellness->inactivity_alert_sent) {
            wellness->inactivity_alert_sent = true;
        }
    } else {
        wellness->inactivity_alert_sent = false;
    }
    
    // Detectar estado de angustia general
    wellness->is_in_distress = (wellness->stress_level > 80 && wellness->energy_level < 30) ||
                              (wellness->mood_score < 20) ||
                              (wellness->consecutive_high_stress_time > 3600); // 1 hora estrés alto
}

// Calcular tendencias de bienestar
static void calculate_wellness_trends(void) {
    partner_wellness_state_t *wellness = &g_partner_wellness;
    
    // Calcular tendencia de estrés (últimas 6 horas)
    float stress_sum_recent = 0, stress_sum_previous = 0;
    int recent_count = 0, previous_count = 0;
    
    for (int i = 0; i < 6; i++) {
        int hour_index = (wellness->current_hour_index - i + 24) % 24;
        if (wellness->stress_history[hour_index] > 0) {
            stress_sum_recent += wellness->stress_history[hour_index];
            recent_count++;
        }
    }
    
    for (int i = 6; i < 12; i++) {
        int hour_index = (wellness->current_hour_index - i + 24) % 24;
        if (wellness->stress_history[hour_index] > 0) {
            stress_sum_previous += wellness->stress_history[hour_index];
            previous_count++;
        }
    }
    
    if (recent_count > 0 && previous_count > 0) {
        float avg_recent = stress_sum_recent / recent_count;
        float avg_previous = stress_sum_previous / previous_count;
        wellness->stress_trend = avg_recent - avg_previous;
    }
    
    // Calcular tendencia de energía de manera similar
    float energy_sum_recent = 0, energy_sum_previous = 0;
    recent_count = previous_count = 0;
    
    for (int i = 0; i < 6; i++) {
        int hour_index = (wellness->current_hour_index - i + 24) % 24;
        if (wellness->energy_history[hour_index] > 0) {
            energy_sum_recent += wellness->energy_history[hour_index];
            recent_count++;
        }
    }
    
    for (int i = 6; i < 12; i++) {
        int hour_index = (wellness->current_hour_index - i + 24) % 24;
        if (wellness->energy_history[hour_index] > 0) {
            energy_sum_previous += wellness->energy_history[hour_index];
            previous_count++;
        }
    }
    
    if (recent_count > 0 && previous_count > 0) {
        float avg_recent = energy_sum_recent / recent_count;
        float avg_previous = energy_sum_previous / previous_count;
        wellness->energy_trend = avg_recent - avg_previous;
    }
    
    ESP_LOGD("PROACTIVE_CARE", "Trends calculated: stress_trend=%.1f, energy_trend=%.1f", 
            wellness->stress_trend, wellness->energy_trend);
}

// Verificar y enviar alertas de cuidado
static void check_and_send_care_alerts(void) {
    partner_wellness_state_t *wellness = &g_partner_wellness;
    proactive_care_config_t *care_config = &g_care_config->proactive_care_settings;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Cooldown entre mensajes de cuidado (mínimo 15 minutos)
    if (current_time - wellness->last_care_message_time < 900) {
        return;
    }
    
    // Verificar alerta de estrés alto sostenido
    if (care_config->partner_stress_alert && !wellness->stress_alert_sent) {
        uint32_t stress_duration = current_time - wellness->consecutive_high_stress_time;
        if (stress_duration >= 1800) { // 30 minutos de estrés alto
            send_stress_support_alert(wellness->stress_level, stress_duration);
            wellness->stress_alert_sent = true;
            wellness->last_care_message_time = current_time;
            return;
        }
    }
    
    // Verificar alerta de energía baja persistente
    if (care_config->partner_low_energy_alert && wellness->energy_alert_sent && 
        wellness->energy_trend < -5.0f) { // Tendencia de energía decreciente
        send_energy_boost_alert(wellness->energy_level, wellness->energy_trend);
        wellness->last_care_message_time = current_time;
        return;
    }
    
    // Verificar alerta de inactividad prolongada
    if (care_config->partner_inactivity_alert && wellness->inactivity_alert_sent) {
        send_activity_encouragement_alert(wellness->inactivity_duration);
        wellness->last_care_message_time = current_time;
        return;
    }
    
    // Verificar preocupación de salud general
    if (care_config->partner_health_concern_alert && wellness->is_in_distress) {
        send_general_health_concern_alert();
        wellness->last_care_message_time = current_time;
        return;
    }
    
    // Verificar alteración del sueño
    if (care_config->partner_sleep_disturbance_alert) {
        uint32_t sleep_quality = get_partner_sleep_quality();
        if (sleep_quality > 0 && sleep_quality < 40) {
            send_sleep_concern_alert(sleep_quality);
            wellness->last_care_message_time = current_time;
            return;
        }
    }
}

// Enviar alerta de apoyo por estrés
static esp_err_t send_stress_support_alert(uint8_t stress_level, uint32_t duration_seconds) {
    proactive_care_config_t *care_config = &g_care_config->proactive_care_settings;
    
    care_action_suggestion_t suggestion = {
        .alert_type = CARE_ALERT_STRESS_SUPPORT,
        .urgency_level = (stress_level > 90) ? 200 : 150,
        .estimated_help_duration = 10 // 10 minutos
    };
    
    // Personalizar mensaje según configuración
    if (strlen(care_config->custom_stress_message) > 0) {
        strncpy(suggestion.care_message, care_config->custom_stress_message, 
                sizeof(suggestion.care_message) - 1);
    } else {
        snprintf(suggestion.care_message, sizeof(suggestion.care_message),
                "Tu pareja lleva %lu min con estrés alto (%d%%). ¿Podrías ofrecerle apoyo?", 
                duration_seconds / 60, stress_level);
    }
    
    snprintf(suggestion.suggested_action, sizeof(suggestion.suggested_action),
            "Mensaje de apoyo, llamada, o actividad relajante juntos");
    
    return send_care_alert_to_user(&suggestion);
}

// Enviar alerta de impulso de energía
static esp_err_t send_energy_boost_alert(uint8_t energy_level, float energy_trend) {
    proactive_care_config_t *care_config = &g_care_config->proactive_care_settings;
    
    care_action_suggestion_t suggestion = {
        .alert_type = CARE_ALERT_ENERGY_BOOST,
        .urgency_level = (energy_level < 20) ? 180 : 120,
        .estimated_help_duration = 15
    };
    
    if (strlen(care_config->custom_support_message) > 0) {
        strncpy(suggestion.care_message, care_config->custom_support_message, 
                sizeof(suggestion.care_message) - 1);
    } else {
        snprintf(suggestion.care_message, sizeof(suggestion.care_message),
                "Tu pareja tiene energía baja (%d%%) y bajando. ¿Un break juntos?", 
                energy_level);
    }
    
    snprintf(suggestion.suggested_action, sizeof(suggestion.suggested_action),
            "Café/té juntos, caminata corta, o snack energético");
    
    return send_care_alert_to_user(&suggestion);
}

// Enviar alerta de estímulo de actividad
static esp_err_t send_activity_encouragement_alert(uint32_t inactivity_duration) {
    care_action_suggestion_t suggestion = {
        .alert_type = CARE_ALERT_ACTIVITY_ENCOURAGE,
        .urgency_level = (inactivity_duration > 7200) ? 150 : 100, // >2 horas
        .estimated_help_duration = 20
    };
    
    snprintf(suggestion.care_message, sizeof(suggestion.care_message),
            "Tu pareja lleva %lu min sin actividad. ¿Una actividad juntos?", 
            inactivity_duration / 60);
    
    snprintf(suggestion.suggested_action, sizeof(suggestion.suggested_action),
            "Caminata, ejercicio ligero, o simplemente cambiar de ambiente");
    
    return send_care_alert_to_user(&suggestion);
}

// Enviar alerta de preocupación general de salud
static esp_err_t send_general_health_concern_alert(void) {
    proactive_care_config_t *care_config = &g_care_config->proactive_care_settings;
    
    care_action_suggestion_t suggestion = {
        .alert_type = CARE_ALERT_HEALTH_CHECK,
        .urgency_level = 180,
        .estimated_help_duration = 5
    };
    
    if (strlen(care_config->custom_care_message) > 0) {
        strncpy(suggestion.care_message, care_config->custom_care_message, 
                sizeof(suggestion.care_message) - 1);
    } else {
        snprintf(suggestion.care_message, sizeof(suggestion.care_message),
                "Las métricas de tu pareja muestran signos de malestar. ¿Todo bien?");
    }
    
    snprintf(suggestion.suggested_action, sizeof(suggestion.suggested_action),
            "Mensaje de verificación o llamada rápida");
    
    return send_care_alert_to_user(&suggestion);
}

// Enviar alerta de preocupación por sueño
static esp_err_t send_sleep_concern_alert(uint32_t sleep_quality) {
    care_action_suggestion_t suggestion = {
        .alert_type = CARE_ALERT_SLEEP_CONCERN,
        .urgency_level = 100,
        .estimated_help_duration = 5
    };
    
    snprintf(suggestion.care_message, sizeof(suggestion.care_message),
            "Tu pareja tuvo una mala noche (calidad %lu%%). ¿Un poco de cuidado extra?", 
            sleep_quality);
    
    snprintf(suggestion.suggested_action, sizeof(suggestion.suggested_action),
            "Mensaje de apoyo o ayuda extra durante el día");
    
    return send_care_alert_to_user(&suggestion);
}

// Enviar alerta de cuidado al usuario
static esp_err_t send_care_alert_to_user(const care_action_suggestion_t *suggestion) {
    if (!suggestion) return ESP_ERR_INVALID_ARG;
    
    proactive_care_config_t *care_config = &g_care_config->proactive_care_settings;
    
    // Respuesta háptica empática si está habilitada
    if (care_config->haptic_empathy_response) {
        haptic_execute_pattern(HAPTIC_PATTERN_PARTNER_STRESS_EMPATHY);
    }
    
    // Notificación directa si está habilitada
    if (care_config->direct_notification_response) {
        ble_notify_proactive_care_alert(suggestion->alert_type, 
                                       suggestion->care_message,
                                       suggestion->suggested_action,
                                       suggestion->urgency_level);
    }
    
    // Recordatorio suave si está habilitado
    if (care_config->gentle_reminder_response) {
        // Usar patrón háptico suave para no ser intrusivo
        haptic_execute_pattern_with_intensity(HAPTIC_PATTERN_OPTIMAL_ENVIRONMENT_SPARKLE, 
                                            HAPTIC_CONSCIOUS_SOFT);
    }
    
    // Registrar evento
    storage_log_event("PROACTIVE_CARE_ALERT", suggestion->care_message);
    
    ESP_LOGI("PROACTIVE_CARE", "Care alert sent: type=%d, urgency=%d", 
            suggestion->alert_type, suggestion->urgency_level);
    
    return ESP_OK;
}

// Configurar umbral de alerta específica
esp_err_t proactive_care_configure_alert_threshold(uint32_t alert_type, uint8_t threshold) {
    proactive_care_config_t *care_config = &g_care_config->proactive_care_settings;
    
    switch (alert_type) {
        case CARE_ALERT_STRESS_SUPPORT:
            care_config->stress_alert_threshold = threshold;
            break;
        case CARE_ALERT_ENERGY_BOOST:
            care_config->energy_low_threshold = threshold;
            break;
        case CARE_ALERT_ACTIVITY_ENCOURAGE:
            care_config->inactivity_alert_time_ms = threshold * 60000; // minutos a ms
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    // Guardar configuración actualizada
    granular_config_save_to_nvs(g_care_config);
    
    ESP_LOGI("PROACTIVE_CARE", "Alert threshold updated: type=%lu, threshold=%d", 
            alert_type, threshold);
    
    return ESP_OK;
}

// Enviar mensaje de apoyo personalizado
esp_err_t proactive_care_send_support_alert(const char* message) {
    if (!message || strlen(message) == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Crear sugerencia personalizada
    care_action_suggestion_t suggestion = {
        .alert_type = CARE_ALERT_GENTLE_REMINDER,
        .urgency_level = 100,
        .estimated_help_duration = 5
    };
    
    strncpy(suggestion.care_message, message, sizeof(suggestion.care_message) - 1);
    strncpy(suggestion.suggested_action, "Mensaje personalizado de cuidado", 
            sizeof(suggestion.suggested_action) - 1);
    
    return send_care_alert_to_user(&suggestion);
}

// Funciones auxiliares
static float calculate_average_energy_last_hours(int hours) {
    partner_wellness_state_t *wellness = &g_partner_wellness;
    float sum = 0;
    int count = 0;
    
    for (int i = 0; i < hours && i < 24; i++) {
        int hour_index = (wellness->current_hour_index - i + 24) % 24;
        if (wellness->energy_history[hour_index] > 0) {
            sum += wellness->energy_history[hour_index];
            count++;
        }
    }
    
    return (count > 0) ? (sum / count) : 0;
}
```

**Modos Contextuales Automáticos y Sistema de Privacidad:**

```c
// contextual_modes_privacy.c

#include "granular_config_system.h"
#include "advanced_haptic_system.h"
#include "esp_log.h"
#include <time.h>

// Estado del modo contextual actual
typedef struct {
    user_context_t current_mode;
    user_context_t previous_mode;
    uint32_t mode_start_time;
    uint32_t mode_change_count_today;
    bool auto_mode_enabled;
    float mode_confidence_score;        // 0.0-1.0 confianza en detección
    
    // Métricas para detección automática
    float movement_intensity_avg;       // Promedio de intensidad de movimiento
    float heart_rate_avg;              // Promedio de ritmo cardíaco
    float stress_level_avg;            // Promedio de nivel de estrés
    bool partner_proximity;            // Pareja cercana
    uint8_t ambient_light_level;       // Nivel de luz ambiental
    uint32_t last_significant_movement; // Última actividad significativa
} contextual_mode_state_t;

// Estado del modo privado y configuración temporal
typedef struct {
    bool private_mode_active;
    uint32_t private_mode_start;
    uint32_t private_mode_duration;
    uint32_t private_mode_auto_exit_time;
    
    // Funciones temporalmente deshabilitadas
    uint64_t temp_disabled_functions;
    uint32_t temp_disable_start;
    uint32_t temp_disable_duration;
    
    // Programación de cambios
    bool scheduled_changes_active;
    scheduled_profile_change_t scheduled_changes[8];
    uint8_t scheduled_change_count;
    
    // Override de emergencia
    bool emergency_override_active;
    granular_config_t emergency_backup_config;
} privacy_temporal_state_t;

// Cambio de perfil programado
typedef struct {
    uint32_t trigger_time;              // Tiempo en minutos desde medianoche
    user_profile_type_t target_profile;
    bool repeat_daily;
    bool weekends_different;
    char description[32];
} scheduled_profile_change_t;

// Estado global
static contextual_mode_state_t g_contextual_state = {0};
static privacy_temporal_state_t g_privacy_state = {0};
static master_config_t *g_context_config = NULL;
static TaskHandle_t g_contextual_task = NULL;

// Inicialización del sistema contextual y de privacidad
esp_err_t contextual_privacy_system_init(master_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    g_context_config = config;
    
    // Inicializar estado contextual
    g_contextual_state.current_mode = USER_CONTEXT_AUTO_DETECT;
    g_contextual_state.auto_mode_enabled = true;
    g_contextual_state.mode_confidence_score = 0.5f;
    
    // Cargar estado de privacidad desde NVS
    load_privacy_state_from_nvs();
    
    // Crear tarea de manejo contextual
    BaseType_t task_created = xTaskCreatePinnedToCore(
        contextual_mode_management_task,
        "contextual_modes",
        4096,
        NULL,
        configMAX_PRIORITIES - 5,
        &g_contextual_task,
        1  // Core 1
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE("CONTEXTUAL", "Failed to create contextual task");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI("CONTEXTUAL", "Contextual modes and privacy system initialized");
    return ESP_OK;
}

// Tarea de gestión de modos contextuales
static void contextual_mode_management_task(void *pvParameters) {
    TickType_t last_mode_detection = 0;
    TickType_t last_scheduled_check = 0;
    TickType_t last_privacy_check = 0;
    
    ESP_LOGI("CONTEXTUAL", "Contextual mode management task started");
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // Detección automática de modo cada 30 segundos
        if (g_contextual_state.auto_mode_enabled && 
            current_time - last_mode_detection > pdMS_TO_TICKS(30000)) {
            detect_and_apply_contextual_mode();
            last_mode_detection = current_time;
        }
        
        // Verificar cambios programados cada minuto
        if (current_time - last_scheduled_check > pdMS_TO_TICKS(60000)) {
            check_scheduled_profile_changes();
            last_scheduled_check = current_time;
        }
        
        // Verificar configuración temporal cada 10 segundos
        if (current_time - last_privacy_check > pdMS_TO_TICKS(10000)) {
            check_temporal_configuration();
            last_privacy_check = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Verificar cada 5 segundos
    }
}

// Detectar y aplicar modo contextual automáticamente
static void detect_and_apply_contextual_mode(void) {
    // Recopilar métricas actuales
    collect_contextual_metrics();
    
    // Detectar modo basado en métricas
    user_context_t detected_mode = analyze_contextual_mode();
    
    // Aplicar modo si cambió y confianza es alta
    if (detected_mode != g_contextual_state.current_mode && 
        g_contextual_state.mode_confidence_score > 0.7f) {
        
        apply_contextual_mode(detected_mode);
    }
}

// Recopilar métricas para detección contextual
static void collect_contextual_metrics(void) {
    contextual_mode_state_t *state = &g_contextual_state;
    uint32_t current_time = esp_timer_get_time() / 1000000;
    
    // Métricas de movimiento
    imu_data_t imu_data;
    if (bmi270_read_data(&imu_data) == ESP_OK) {
        float movement = sqrt(imu_data.accel_x * imu_data.accel_x + 
                            imu_data.accel_y * imu_data.accel_y + 
                            imu_data.accel_z * imu_data.accel_z);
        
        // Promedio móvil de movimiento
        state->movement_intensity_avg = (state->movement_intensity_avg * 0.9f) + (movement * 0.1f);
        
        // Detectar última actividad significativa
        if (movement > 1.5f) {
            state->last_significant_movement = current_time;
        }
    }
    
    // Métricas de salud
    state->heart_rate_avg = (state->heart_rate_avg * 0.9f) + (get_current_heart_rate() * 0.1f);
    state->stress_level_avg = (state->stress_level_avg * 0.9f) + (get_current_stress_level() * 0.1f);
    
    // Proximidad de pareja
    state->partner_proximity = is_partner_nearby();
    
    // Nivel de luz ambiental (estimado desde sensores ambientales)
    bme688_data_t env_data;
    if (bme688_read_all(&env_data) == ESP_OK) {
        // Estimar luz basándose en temperatura y otros factores
        state->ambient_light_level = estimate_ambient_light_level(&env_data);
    }
}

// Analizar modo contextual basado en métricas
static user_context_t analyze_contextual_mode(void) {
    contextual_mode_state_t *state = &g_contextual_state;
    uint32_t current_time = esp_timer_get_time() / 1000000;
    struct tm timeinfo;
    localtime_r((time_t*)&current_time, &timeinfo);
    
    float confidence = 0.0f;
    user_context_t detected_mode = USER_CONTEXT_RELAXING;
    
    // Detectar sueño (22:00-07:00 + baja actividad + baja luz)
    if (((timeinfo.tm_hour >= 22) || (timeinfo.tm_hour < 7)) && 
        state->movement_intensity_avg < 0.2f && 
        state->ambient_light_level < 30) {
        detected_mode = USER_CONTEXT_SLEEPING;
        confidence = 0.9f;
    }
    // Detectar ejercicio (alta actividad + HR elevado)
    else if (state->movement_intensity_avg > 2.0f && 
             state->heart_rate_avg > get_baseline_heart_rate() + 30) {
        detected_mode = USER_CONTEXT_EXERCISING;
        confidence = 0.8f;
    }
    // Detectar trabajo (horario laboral + baja variabilidad + concentración)
    else if (timeinfo.tm_hour >= 9 && timeinfo.tm_hour < 17 && 
             state->movement_intensity_avg < 0.8f && 
             get_movement_variability() < 0.3f) {
        detected_mode = USER_CONTEXT_WORKING_FOCUSED;
        confidence = 0.7f;
    }
    // Detectar tiempo de pareja (pareja cerca + horario social/personal)
    else if (state->partner_proximity && 
             ((timeinfo.tm_hour >= 18) || (timeinfo.tm_hour < 9) || 
              (timeinfo.tm_wday == 0 || timeinfo.tm_wday == 6))) {
        detected_mode = USER_CONTEXT_COUPLE_TIME;
        confidence = 0.8f;
    }
    // Detectar interacción social (movimiento variable + horario social)
    else if (get_movement_variability() > 0.7f && 
             state->movement_intensity_avg > 0.5f &&
             (timeinfo.tm_hour >= 17 || timeinfo.tm_hour < 2)) {
        detected_mode = USER_CONTEXT_SOCIAL_INTERACTION;
        confidence = 0.6f;
    }
    
    state->mode_confidence_score = confidence;
    
    ESP_LOGD("CONTEXTUAL", "Mode analysis: detected=%d, confidence=%.2f", 
            detected_mode, confidence);
    
    return detected_mode;
}

// Aplicar modo contextual
static esp_err_t apply_contextual_mode(user_context_t new_mode) {
    contextual_mode_state_t *state = &g_contextual_state;
    contextual_modes_config_t *modes_config = &g_context_config->contextual_settings;
    
    // Verificar si el auto-modo está habilitado para este contexto
    bool auto_enabled = false;
    switch (new_mode) {
        case USER_CONTEXT_WORKING_FOCUSED:
            auto_enabled = modes_config->auto_work_mode;
            break;
        case USER_CONTEXT_SLEEPING:
            auto_enabled = modes_config->auto_sleep_mode;
            break;
        case USER_CONTEXT_EXERCISING:
            auto_enabled = modes_config->auto_exercise_mode;
            break;
        case USER_CONTEXT_SOCIAL_INTERACTION:
            auto_enabled = modes_config->auto_social_mode;
            break;
        case USER_CONTEXT_COUPLE_TIME:
            auto_enabled = true; // Siempre habilitado
            break;
        default:
            auto_enabled = true;
            break;
    }
    
    if (!auto_enabled) {
        ESP_LOGD("CONTEXTUAL", "Auto mode disabled for context %d", new_mode);
        return ESP_OK;
    }
    
    // Guardar modo anterior
    state->previous_mode = state->current_mode;
    state->current_mode = new_mode;
    state->mode_start_time = esp_timer_get_time() / 1000000;
    state->mode_change_count_today++;
    
    // Aplicar configuración específica del modo
    apply_mode_specific_configuration(new_mode);
    
    // Notificar cambio de modo vía BLE
    ble_notify_contextual_mode_change(new_mode, state->mode_confidence_score);
    
    // Log del cambio
    storage_log_event("CONTEXTUAL_MODE_CHANGE", get_mode_name(new_mode));
    
    ESP_LOGI("CONTEXTUAL", "Applied contextual mode: %s (confidence: %.2f)", 
            get_mode_name(new_mode), state->mode_confidence_score);
    
    return ESP_OK;
}

// Aplicar configuración específica del modo
static void apply_mode_specific_configuration(user_context_t mode) {
    contextual_modes_config_t *modes_config = &g_context_config->contextual_settings;
    granular_config_t *target_config = NULL;
    
    // Seleccionar configuración según modo
    switch (mode) {
        case USER_CONTEXT_WORKING_FOCUSED:
            target_config = &modes_config->work_mode_config;
            break;
        case USER_CONTEXT_SLEEPING:
            target_config = &modes_config->sleep_mode_config;
            break;
        case USER_CONTEXT_EXERCISING:
            target_config = &modes_config->exercise_mode_config;
            break;
        case USER_CONTEXT_SOCIAL_INTERACTION:
            target_config = &modes_config->social_mode_config;
            break;
        case USER_CONTEXT_COUPLE_TIME:
            // Habilitar funciones de pareja
            enable_couple_specific_features();
            break;
        case USER_CONTEXT_RELAXING:
            target_config = &modes_config->relaxation_mode_config;
            break;
        default:
            return; // No hay configuración específica
    }
    
    if (target_config) {
        // Aplicar configuración temporal (no persistente)
        apply_temporary_configuration(target_config);
    }
}

// Verificar cambios de perfil programados
static void check_scheduled_profile_changes(void) {
    if (!g_privacy_state.scheduled_changes_active) {
        return;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000000;
    struct tm timeinfo;
    localtime_r((time_t*)&current_time, &timeinfo);
    
    uint32_t current_minutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    bool is_weekend = (timeinfo.tm_wday == 0 || timeinfo.tm_wday == 6);
    
    for (int i = 0; i < g_privacy_state.scheduled_change_count; i++) {
        scheduled_profile_change_t *change = &g_privacy_state.scheduled_changes[i];
        
        // Verificar si es tiempo de cambio
        if (current_minutes == change->trigger_time) {
            // Verificar si aplica para fin de semana
            if (change->weekends_different && is_weekend) {
                continue;
            }
            
            // Aplicar cambio de perfil
            granular_config_set_profile(change->target_profile);
            
            ESP_LOGI("CONTEXTUAL", "Scheduled profile change: %s at %02d:%02d", 
                    get_profile_name(change->target_profile), 
                    timeinfo.tm_hour, timeinfo.tm_min);
            
            // Registrar evento
            storage_log_event("SCHEDULED_PROFILE_CHANGE", change->description);
        }
    }
}

// Verificar configuración temporal y modo privado
static void check_temporal_configuration(void) {
    privacy_temporal_state_t *privacy = &g_privacy_state;
    uint32_t current_time = esp_timer_get_time() / 1000000;
    
    // Verificar salida automática del modo privado
    if (privacy->private_mode_active && privacy->private_mode_auto_exit_time > 0) {
        if (current_time >= privacy->private_mode_auto_exit_time) {
            granular_config_disable_private_mode();
            ESP_LOGI("PRIVACY", "Auto-exited private mode");
        }
    }
    
    // Verificar reactivación de funciones temporalmente deshabilitadas
    if (privacy->temp_disabled_functions != 0) {
        uint32_t disable_duration = current_time - privacy->temp_disable_start;
        if (disable_duration >= privacy->temp_disable_duration) {
            // Reactivar funciones
            privacy->temp_disabled_functions = 0;
            ESP_LOGI("PRIVACY", "Re-enabled temporarily disabled functions");
            
            // Guardar estado
            save_privacy_state_to_nvs();
        }
    }
}

// Funciones de modo privado
esp_err_t granular_config_enable_private_mode(uint32_t duration_minutes) {
    privacy_temporal_state_t *privacy = &g_privacy_state;
    uint32_t current_time = esp_timer_get_time() / 1000000;
    
    privacy->private_mode_active = true;
    privacy->private_mode_start = current_time;
    privacy->private_mode_duration = duration_minutes * 60;
    
    if (duration_minutes > 0) {
        privacy->private_mode_auto_exit_time = current_time + privacy->private_mode_duration;
    } else {
        privacy->private_mode_auto_exit_time = 0; // Modo indefinido
    }
    
    // Parar compartición de datos inmediatamente
    pause_data_sharing();
    
    // Notificación háptica de activación
    haptic_execute_pattern(HAPTIC_PATTERN_OPTIMAL_ENVIRONMENT_SPARKLE);
    
    // Guardar estado
    save_privacy_state_to_nvs();
    
    ESP_LOGI("PRIVACY", "Private mode enabled for %lu minutes", duration_minutes);
    
    return ESP_OK;
}

esp_err_t granular_config_disable_private_mode(void) {
    privacy_temporal_state_t *privacy = &g_privacy_state;
    
    privacy->private_mode_active = false;
    privacy->private_mode_auto_exit_time = 0;
    
    // Reanudar compartición de datos
    resume_data_sharing();
    
    // Notificación háptica de desactivación
    haptic_execute_pattern_with_intensity(HAPTIC_PATTERN_OPTIMAL_ENVIRONMENT_SPARKLE, 
                                        HAPTIC_CONSCIOUS_SOFT);
    
    // Guardar estado
    save_privacy_state_to_nvs();
    
    ESP_LOGI("PRIVACY", "Private mode disabled");
    
    return ESP_OK;
}

// Deshabilitar función temporalmente
esp_err_t granular_config_disable_function_temporarily(uint32_t function_id, uint32_t duration_minutes) {
    if (function_id >= 64) return ESP_ERR_INVALID_ARG; // Máximo 64 funciones
    
    privacy_temporal_state_t *privacy = &g_privacy_state;
    uint32_t current_time = esp_timer_get_time() / 1000000;
    
    // Establecer bit de función deshabilitada
    privacy->temp_disabled_functions |= (1ULL << function_id);
    privacy->temp_disable_start = current_time;
    privacy->temp_disable_duration = duration_minutes * 60;
    
    // Guardar estado
    save_privacy_state_to_nvs();
    
    ESP_LOGI("PRIVACY", "Function %lu disabled for %lu minutes", function_id, duration_minutes);
    
    return ESP_OK;
}

// Programar cambio de perfil
esp_err_t granular_config_schedule_profile_change(uint32_t time_hour_minute, 
                                                  user_profile_type_t profile) {
    privacy_temporal_state_t *privacy = &g_privacy_state;
    
    if (privacy->scheduled_change_count >= 8) {
        ESP_LOGW("PRIVACY", "Maximum scheduled changes reached");
        return ESP_ERR_NO_MEM;
    }
    
    // Convertir hora:minuto a minutos totales
    uint32_t hours = time_hour_minute / 100;
    uint32_t minutes = time_hour_minute % 100;
    uint32_t total_minutes = hours * 60 + minutes;
    
    if (hours >= 24 || minutes >= 60) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Agregar cambio programado
    scheduled_profile_change_t *change = &privacy->scheduled_changes[privacy->scheduled_change_count];
    change->trigger_time = total_minutes;
    change->target_profile = profile;
    change->repeat_daily = true;
    change->weekends_different = false;
    snprintf(change->description, sizeof(change->description), 
            "Auto change to %s at %02lu:%02lu", 
            get_profile_name(profile), hours, minutes);
    
    privacy->scheduled_change_count++;
    privacy->scheduled_changes_active = true;
    
    // Guardar estado
    save_privacy_state_to_nvs();
    
    ESP_LOGI("PRIVACY", "Scheduled profile change: %s at %02lu:%02lu", 
            get_profile_name(profile), hours, minutes);
    
    return ESP_OK;
}

// Verificar si función está habilitada
bool granular_config_is_function_enabled(uint32_t function_id) {
    // Verificar si está temporalmente deshabilitada
    if (g_privacy_state.temp_disabled_functions & (1ULL << function_id)) {
        return false;
    }
    
    // Verificar configuración granular
    return is_function_enabled_in_granular_config(function_id);
}
```

**Comandos Bluetooth Completos para Configuración Remota:**

```c
// bluetooth_config_commands.c

// Comandos completos de configuración BLE
typedef enum {
    BLE_CMD_SET_PROFILE = 0x10,
    BLE_CMD_ENABLE_FUNCTION = 0x11,
    BLE_CMD_DISABLE_FUNCTION = 0x12,
    BLE_CMD_SET_SHARING_PREF = 0x13,
    BLE_CMD_ENABLE_PRIVATE_MODE = 0x14,
    BLE_CMD_DISABLE_PRIVATE_MODE = 0x15,
    BLE_CMD_SCHEDULE_PROFILE_CHANGE = 0x16,
    BLE_CMD_SET_CONTEXTUAL_MODE = 0x17,
    BLE_CMD_CONFIGURE_CARE_ALERT = 0x18,
    BLE_CMD_PAIR_WITH_PARTNER = 0x19,
    BLE_CMD_UNPAIR_PARTNER = 0x1A,
    BLE_CMD_FACTORY_RESET = 0x1B,
    BLE_CMD_EXPORT_CONFIG = 0x1C,
    BLE_CMD_IMPORT_CONFIG = 0x1D,
    BLE_CMD_GET_STATUS = 0x1E,
    BLE_CMD_EMERGENCY_OVERRIDE = 0x1F
} ble_config_command_t;

// Estructura de comando BLE extendido
typedef struct {
    uint8_t command;
    uint8_t subcommand;
    uint16_t parameter1;
    uint32_t parameter2;
    char string_param[64];
    uint8_t checksum;
} ble_extended_command_t;

// Handler principal de comandos BLE
esp_err_t ble_config_command_handler(const uint8_t *data, uint16_t len) {
    if (len < sizeof(ble_extended_command_t)) {
        ESP_LOGW("BLE_CONFIG", "Invalid command length: %d", len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    ble_extended_command_t *cmd = (ble_extended_command_t *)data;
    
    // Verificar checksum
    if (!verify_command_checksum(cmd)) {
        ESP_LOGW("BLE_CONFIG", "Invalid command checksum");
        return ESP_ERR_INVALID_CRC;
    }
    
    esp_err_t result = ESP_OK;
    
    switch (cmd->command) {
        case BLE_CMD_SET_PROFILE:
            result = ble_handle_set_profile(cmd);
            break;
        case BLE_CMD_ENABLE_FUNCTION:
            result = ble_handle_enable_function(cmd);
            break;
        case BLE_CMD_DISABLE_FUNCTION:
            result = ble_handle_disable_function(cmd);
            break;
        case BLE_CMD_SET_SHARING_PREF:
            result = ble_handle_set_sharing_preference(cmd);
            break;
        case BLE_CMD_ENABLE_PRIVATE_MODE:
            result = granular_config_enable_private_mode(cmd->parameter1);
            break;
        case BLE_CMD_DISABLE_PRIVATE_MODE:
            result = granular_config_disable_private_mode();
            break;
        case BLE_CMD_SCHEDULE_PROFILE_CHANGE:
            result = granular_config_schedule_profile_change(cmd->parameter1, cmd->parameter2);
            break;
        case BLE_CMD_PAIR_WITH_PARTNER:
            result = ble_handle_pair_with_partner(cmd);
            break;
        case BLE_CMD_FACTORY_RESET:
            result = granular_config_factory_reset();
            break;
        case BLE_CMD_GET_STATUS:
            result = ble_send_complete_status();
            break;
        default:
            ESP_LOGW("BLE_CONFIG", "Unknown command: 0x%02X", cmd->command);
            result = ESP_ERR_NOT_SUPPORTED;
            break;
    }
    
    // Enviar respuesta
    ble_send_command_response(cmd->command, result);
    
    return result;
}

// Enviar estado completo del dispositivo
static esp_err_t ble_send_complete_status(void) {
    char status_json[1024];
    size_t json_size = sizeof(status_json);
    
    // Generar JSON con estado completo
    snprintf(status_json, json_size,
        "{"
        "\"profile\":\"%s\","
        "\"private_mode\":%s,"
        "\"current_context\":\"%s\","
        "\"partner_paired\":%s,"
        "\"battery\":%d,"
        "\"functions_enabled\":%d,"
        "\"sharing_active\":%s,"
        "\"learning_mode\":%s"
        "}",
        get_profile_name(g_context_config->current_profile),
        g_privacy_state.private_mode_active ? "true" : "false",
        get_mode_name(g_contextual_state.current_mode),
        g_partner_state.is_paired ? "true" : "false",
        power_get_battery_percentage(),
        count_enabled_functions(),
        is_data_sharing_active() ? "true" : "false",
        g_context_config->granular_settings.learning_mode ? "true" : "false"
    );
    
    return ble_send_long_response(status_json, strlen(status_json));
}