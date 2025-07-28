#ifndef SENSOR_MAX30102_H
#define SENSOR_MAX30102_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "common_types.h"

// Estructura para muestras PPG
typedef struct {
    uint32_t ir_value;
    uint32_t red_value;
    uint32_t timestamp;
} max30102_sample_t;

/**
 * Inicializar el sensor MAX30102
 */
esp_err_t max30102_init(i2c_port_t port);

/**
 * Leer FIFO de muestras PPG
 */
esp_err_t max30102_read_fifo(max30102_sample_t *samples, size_t *count);

/**
 * Calcular frecuencia cardíaca
 */
uint16_t max30102_calculate_heart_rate(const max30102_sample_t *samples, size_t count);

/**
 * Calcular saturación de oxígeno SpO2
 */
uint8_t max30102_calculate_spo2(const max30102_sample_t *samples, size_t count);

#endif // SENSOR_MAX30102_H