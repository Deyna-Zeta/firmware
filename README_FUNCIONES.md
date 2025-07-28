# Firmware Vitamina P - 9 Funciones Implementadas

Este firmware implementa las **9 funciones específicas** requeridas para el collar inteligente "Vitamina P" usando ESP32-S3 y múltiples sensores.

## ✅ FUNCIONES IMPLEMENTADAS

### 1. DETECCIÓN DE POSTURA ENCORVADA
- **Archivo**: `src/tasks/task_postura_encorvada.c`
- **Sensores**: BMI270 + BMM150
- **Función**: Detectar cuando los hombros se desplazan hacia adelante (encorvamiento)
- **Umbral**: Detectar inclinación >25° por más de 5 minutos
- **Alerta**: "Postura encorvada detectada: ¿Necesitas un descanso?"
- **Acción**: Vibración + notificación por Bluetooth

### 2. DETECCIÓN DE ESTRÉS
- **Archivo**: `src/tasks/task_deteccion_estres.c`
- **Sensores**: MAX30102 + BME688 + BMM150
- **Función**: Analizar variabilidad cardíaca (HRV) + sudoración + tensión muscular
- **Criterios**: Frecuencia cardíaca irregular + cambios en humedad ambiente + movimientos tensos
- **Alerta**: "¿Estás estresado?" con sugerencia de respiración profunda
- **Acción**: Vibración suave + notificación

### 3. CALIDAD DE SUEÑO
- **Archivo**: `src/tasks/task_calidad_sueño.c`
- **Sensores**: BMI270 + MAX30102 + MAX30205
- **Función**: Monitorear movimientos nocturnos + SpO2 + temperatura corporal
- **Análisis**: Detectar fases de sueño profundo vs. ligero
- **Reporte**: "Dormiste X horas (Y% eficiencia)" al despertar

### 4. ALERTA DE HIDRATACIÓN
- **Archivo**: `src/tasks/task_alertas_salud.c` (función `check_hydration_levels`)
- **Sensores**: MAX86176 + MAX30205
- **Función**: Correlacionar hidratación cutánea óptica con temperatura corporal
- **Criterios**: Hidratación baja detectada por sensor óptico + temperatura elevada
- **Alerta**: "Hidratación baja: Bebe agua"
- **Acción**: Notificación simple

### 5. RITMO CARDÍACO ELEVADO SIN ACTIVIDAD
- **Archivo**: `src/tasks/task_alertas_salud.c` (función `check_elevated_hr_without_activity`)
- **Sensores**: MAX30102 + BMI270
- **Función**: Detectar frecuencia cardíaca >100 BPM sin movimiento físico reciente
- **Tiempo**: Sin actividad por >5 minutos pero HR elevada
- **Alerta**: "Ritmo cardíaco elevado sin actividad física. ¿Estás estresado?"
- **Acción**: Vibración + notificación

### 6. AMBIENTE MAL VENTILADO
- **Archivo**: `src/tasks/task_alertas_salud.c` (función `check_air_quality`)
- **Sensor**: BME688
- **Función**: Detectar CO2 alto y VOCs (Compuestos Orgánicos Volátiles)
- **Umbral**: Resistencia de gas <50 kΩ (indicador de mala calidad del aire)
- **Alerta**: "Ambiente mal ventilado"
- **Sugerencia**: "Abre una ventana"

### 7. CONTADOR DE PASOS Y MOVIMIENTO
- **Archivo**: `src/tasks/task_actividad_fisica.c` (función `task_contador_pasos`)
- **Sensores**: BMI270 + BMM150
- **Función**: Algoritmo de conteo de pasos usando acelerómetro + magnetómetro para orientación
- **Registro**: Pasos diarios, distancia estimada, calorías
- **Mostrar**: Estadísticas en la app

### 8. DETECCIÓN DE SEDENTARISMO MEJORADA
- **Archivo**: `src/tasks/task_actividad_fisica.c` (función `task_sedentarismo_mejorado`)
- **Sensores**: BMI270 + BMM150
- **Función**: Detectar ausencia de movimiento significativo
- **Umbral**: >1 hora sin movimiento (aceleración <0.1g)
- **Alerta**: Vibración progresiva (suave → fuerte)
- **Mensaje**: "Llevas más de 1 hora sin moverte. ¡Es hora de activarse!"

### 9. ALERTA DE POSTURA INCORRECTA GENERAL
- **Archivo**: `src/tasks/task_actividad_fisica.c` (función `task_postura_incorrecta_general`)
- **Sensores**: BMI270 + BMM150
- **Función**: Detectar cualquier desviación de la postura calibrada
- **Incluye**: Cabeza hacia adelante, hombros caídos, espalda encorvada
- **Acción**: Vibración inmediata + notificación
- **Mensaje**: "Corrige tu postura"

## 📁 ESTRUCTURA DEL PROYECTO

```
firmware_vitamina_p/
├── main.c                              # Punto de entrada principal
├── CMakeLists.txt                      # Configuración de compilación
├── include/                            # Headers
│   ├── common_types.h                  # Tipos y constantes comunes
│   ├── sensor_headers.h                # Headers de sensores
│   ├── service_headers.h               # Headers de servicios
│   ├── task_postura.h                  # Header tareas postura
│   ├── task_salud.h                    # Header tareas salud
│   ├── task_ambiente.h                 # Header tareas ambiente
│   ├── task_sedentarismo.h             # Header sedentarismo
│   ├── task_pasos.h                    # Header contador pasos
│   └── task_sueño.h                    # Header calidad sueño
└── src/
    └── tasks/                          # Implementación de funciones
        ├── task_postura_encorvada.c    # FUNCIÓN 1: Postura encorvada
        ├── task_deteccion_estres.c     # FUNCIÓN 2: Detección estrés
        ├── task_calidad_sueño.c        # FUNCIÓN 3: Calidad sueño
        ├── task_alertas_salud.c        # FUNCIONES 4,5,6: Alertas salud
        └── task_actividad_fisica.c     # FUNCIONES 7,8,9: Actividad física
```

## 🔧 CARACTERÍSTICAS TÉCNICAS

### Multitarea FreeRTOS
- **9 tareas independientes** ejecutándose en paralelo
- **Prioridades diferenciadas** según criticidad
- **Comunicación entre tareas** via colas y event groups
- **Gestión eficiente de memoria** con buffers circulares

### Algoritmos Avanzados
- **Calibración automática** de posturas de referencia
- **Filtrado y suavizado** de señales de sensores
- **Detección de patrones** en tiempo real
- **Machine learning básico** para clasificación de estados

### Gestión de Energía
- **Intervalos de verificación optimizados** por función
- **Sensores de baja frecuencia** para funciones menos críticas
- **Sleep modes** cuando es apropiado
- **Alertas inteligentes** con cooldown para evitar spam

### Conectividad
- **Bluetooth LE** para notificaciones en tiempo real
- **Almacenamiento local** con respaldo automático
- **Sincronización de datos** cuando hay conectividad

## 📊 UMBRALES Y CONFIGURACIÓN

| Función | Umbral Principal | Tiempo Evaluación | Cooldown Alertas |
|---------|------------------|-------------------|------------------|
| Postura encorvada | >25° inclinación | 5 minutos | 15 minutos |
| Estrés | HRV <30ms | 2 segundos | 20 minutos |
| Hidratación | <40% nivel | 10 minutos | 2 horas |
| HR elevada | >100 BPM | 5 minutos sin actividad | 30 minutos |
| Aire mal ventilado | <50 kΩ resistencia gas | 2 minutos | 30 minutos |
| Sedentarismo | <0.1g movimiento | 1 hora | 15 minutos |
| Postura general | ±15° desviación XY | 1 segundo | 5 minutos |

## 🚀 COMPILACIÓN Y USO

### Prerrequisitos
- ESP-IDF v5.0 o superior
- Toolchain ESP32-S3
- Drivers de sensores (BMI270, MAX30102, etc.)

### Compilar
```bash
idf.py build
```

### Flashear
```bash
idf.py flash monitor
```

### Logs del Sistema
El firmware genera logs detallados para cada función:
- **ESP_LOGI**: Eventos importantes y estado del sistema
- **ESP_LOGW**: Alertas activadas
- **ESP_LOGD**: Debug detallado de sensores
- **ESP_LOGE**: Errores de sensores o sistema

## 🎯 VERIFICACIÓN DE FUNCIONES

Para verificar que todas las 9 funciones están activas, busca en los logs:

```
=== INICIANDO TAREA DETECCIÓN POSTURA ENCORVADA ===
=== INICIANDO TAREA DETECCIÓN DE ESTRÉS ===
=== INICIANDO TAREA CALIDAD DE SUEÑO ===
=== INICIANDO TAREAS DE ALERTAS DE SALUD ===
=== INICIANDO TAREA RITMO CARDÍACO ELEVADO ===
=== INICIANDO TAREA AMBIENTE MAL VENTILADO ===
=== INICIANDO TAREA CONTADOR DE PASOS ===
=== INICIANDO TAREA SEDENTARISMO MEJORADO ===
=== INICIANDO TAREA POSTURA INCORRECTA GENERAL ===
```

## 📈 EXPANSIÓN FUTURA

El código está diseñado para fácil expansión:
- **Nuevos sensores**: Agregar en `sensor_headers.h`
- **Nuevas funciones**: Crear nuevas tareas siguiendo el patrón existente
- **Algoritmos mejorados**: Reemplazar funciones de procesamiento
- **Conectividad adicional**: WiFi, LoRa, etc.

---

✅ **TODAS LAS 9 FUNCIONES REQUERIDAS ESTÁN IMPLEMENTADAS Y FUNCIONALES**