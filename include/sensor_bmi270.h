#ifndef SENSOR_BMI270_H
#define SENSOR_BMI270_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "common_types.h"

/**
 * Inicializar el sensor BMI270
 */
esp_err_t bmi270_init(i2c_port_t port);

/**
 * Leer datos del acelerómetro y giroscopio
 */
esp_err_t bmi270_read_data(imu_data_t *data);

/**
 * Calcular ángulo de inclinación
 */
float bmi270_calculate_tilt_angle(const imu_data_t *data);

#endif // SENSOR_BMI270_H