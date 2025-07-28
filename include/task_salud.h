#ifndef TASK_SALUD_H
#define TASK_SALUD_H

#include "common_types.h"

// Función principal de detección de estrés
void task_deteccion_estres(void *pvParameters);

// Función principal de alerta de hidratación
void task_alerta_hidratacion(void *pvParameters);

// Función principal de ritmo cardíaco elevado
void task_ritmo_cardiaco_elevado(void *pvParameters);

#endif // TASK_SALUD_H