#ifndef TASK_POSTURA_H
#define TASK_POSTURA_H

#include "common_types.h"

// Función principal de detección de postura encorvada
void task_postura_encorvada(void *pvParameters);

// Función principal de postura incorrecta general
void task_postura_incorrecta_general(void *pvParameters);

#endif // TASK_POSTURA_H