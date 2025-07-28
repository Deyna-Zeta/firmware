#ifndef SENSOR_HEADERS_H
#define SENSOR_HEADERS_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "common_types.h"

// ============ SENSOR BMM150 (Magnetómetro) ============
esp_err_t bmm150_init(i2c_port_t port);
esp_err_t bmm150_read_data(imu_data_t *data);

// ============ SENSOR MAX30205 (Temperatura) ============
esp_err_t max30205_init(i2c_port_t port);
float max30205_read_temperature(void);

// ============ SENSOR MAX86176 (Hidratación) ============
esp_err_t max86176_init(i2c_port_t port);
esp_err_t max86176_read_hydration(hydration_data_t *data);
esp_err_t max86176_calibrate_baseline(void);

// ============ SENSOR BME688 (Ambiental) ============
esp_err_t bme688_init(i2c_port_t port);
esp_err_t bme688_read_all(environment_data_t *data);
esp_err_t bme688_force_measurement(void);

#endif // SENSOR_HEADERS_H