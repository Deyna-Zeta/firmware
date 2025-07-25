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

esp_err_t ble_service_init(void);
esp_err_t ble_start_advertising(void);
esp_err_t ble_notify_heart_rate(uint8_t bpm);
esp_err_t ble_notify_stress_level(uint8_t level);
esp_err_t ble_set_partner_vibration(uint8_t pattern);
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

### Control de Vibración

Manejo del motor vibrador para retroalimentación háptica.

```c
// vibration.h
typedef enum {
    VIB_PATTERN_SHORT,
    VIB_PATTERN_LONG,
    VIB_PATTERN_DOUBLE,
    VIB_PATTERN_BREATHING,
    VIB_PATTERN_PARTNER_SYNC
} vibration_pattern_t;

esp_err_t vibration_init(void);
esp_err_t vibration_set_pattern(vibration_pattern_t pattern);
esp_err_t vibration_set_intensity(uint8_t intensity);
esp_err_t vibration_stop(void);
```

**Patrones implementados:**
- **Alerta de postura**: Triple pulso corto
- **Estrés alto**: Vibración rítmica para respiración guiada
- **Hidratación baja**: Doble pulso largo
- **Calidad del aire**: Serie de pulsos cortos
- **Sincronización de pareja**: Patrón especial reconocible

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
    
    while (1) {
        // Leer datos del IMU
        if (bmi270_read_data(&imu_data) == ESP_OK) {
            current_angle = bmi270_calculate_tilt_angle(&imu_data);
            
            // Evaluar postura
            if (current_angle > POSTURE_THRESHOLD_DEGREES) {
                if (bad_posture_start == 0) {
                    bad_posture_start = esp_timer_get_time() / 1000;
                } else {
                    uint32_t duration = (esp_timer_get_time() / 1000) - bad_posture_start;
                    if (duration > POSTURE_ALERT_TIME_MS) {
                        // Activar alerta
                        vibration_set_pattern(VIB_PATTERN_POSTURE_ALERT);
                        storage_log_event("POSTURE_ALERT", NULL);
                        ble_notify_posture_alert(current_angle);
                        
                        bad_posture_start = 0;  // Reset para evitar spam
                        vTaskDelay(pdMS_TO_TICKS(POSTURE_COOLDOWN_MS));
                    }
                }
            } else {
                bad_posture_start = 0;  // Reset si mejora postura
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

### Ventajas Técnicas

- **Multitarea real** con núcleos dedicados para estabilidad
- **Conectividad dual** BLE para tiempo real, Wi-Fi para sincronización
- **Algoritmos optimizados** para procesamiento local sin depender de la nube
- **Actualizaciones OTA** para evolución continua del firmware
- **Escalabilidad** para agregar nuevos sensores o funcionalidades

El firmware resultante proporciona una base sólida para un dispositivo wearable de próxima generación, enfocado en el bienestar integral y la conexión emocional entre parejas, manteniendo los más altos estándares de calidad técnica y eficiencia energética.