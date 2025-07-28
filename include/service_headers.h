#ifndef SERVICE_HEADERS_H
#define SERVICE_HEADERS_H

#include "esp_err.h"
#include "common_types.h"

// ============ SERVICIO DE VIBRACIÓN ============
esp_err_t vibration_init(void);
esp_err_t vibration_set_pattern(vibration_pattern_t pattern);

// ============ SERVICIO BLE ============
esp_err_t ble_service_init(void);
bool ble_is_connected(void);
esp_err_t ble_notify_posture_alert(float angle, const char *message);
esp_err_t ble_notify_stress_alert(float stress_index, const char *message);
esp_err_t ble_notify_sleep_report(uint32_t total_time, uint8_t efficiency, const char *message);
esp_err_t ble_notify_hydration_alert(uint8_t level, float temp, const char *message);
esp_err_t ble_notify_hr_elevated_alert(uint16_t hr, uint32_t duration, const char *message);
esp_err_t ble_notify_air_quality_alert(const environment_data_t *env, const char *message);
esp_err_t ble_notify_step_data(const step_data_t *steps);
esp_err_t ble_notify_sedentary_alert(uint32_t duration, uint32_t count, const char *message);
esp_err_t ble_notify_posture_correction_alert(float dev_x, float dev_y, float dev_z, const char *message);

// ============ SERVICIO DE ALMACENAMIENTO ============
esp_err_t storage_init(void);
esp_err_t storage_log_event(const char *event_type, const char *details);
esp_err_t storage_save_health_metrics(const health_metrics_t *metrics);
esp_err_t storage_save_environment_data(const environment_data_t *data);
esp_err_t storage_save_sleep_data(const sleep_data_t *data);
esp_err_t storage_save_daily_steps(const step_data_t *steps);
esp_err_t storage_log_sleep_phase_change(int old_phase, int new_phase);

// ============ SERVICIO DE GESTIÓN DE ENERGÍA ============
esp_err_t power_manager_init(void);
esp_err_t power_manager_check_battery(void);

#endif // SERVICE_HEADERS_H