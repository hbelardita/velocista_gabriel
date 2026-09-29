# Configuración de Sensores de Línea (Arreglo Discreto de 4 Sensores)

**Estado:** Aceptado / Activo

Decidimos implementar la arquitectura de 4 sensores digitales TCRT5000 discretos permitida por el reglamento (Art. 1.2.1) para maximizar la velocidad de crucero en recta y mitigar de forma determinística el riesgo de despiste en curvas de 30 cm de radio (Art. 3.5) sobre pista elevada (Art. 3.4), eliminando la dependencia de retardos temporales arbitrarios para discriminar curvas.

## Geometría Física y Distribución Espacial

- **Ancho frontal del chasis:** 115 mm.
- **Ancho por módulo sensor:** 10 mm cada uno.
- **Línea de trayectoria:** 20 mm de ancho (se extiende de x = -10 mm a x = +10 mm cuando el robot está centrado).
- **Posiciones relativas (origen en centro del chasis x = 0):**
  - **S1 (Ext Izq):** x = -25 mm (Pin A0)
  - **S2 (Int Izq):** x = -6 mm (Pin A1)
  - **S3 (Int Der):** x = +6 mm (Pin A2)
  - **S4 (Ext Der):** x = +25 mm (Pin A3)
- **Separación entre centros (pitch):**
  - Sensores internos (S2 - S3): 12 mm (ambos montados dentro de la línea de 20 mm cuando el robot viaja centrado).
  - Sensores interno y externo (S1 - S2 y S3 - S4): 19 mm (los sensores externos S1 y S4 viajan fuera de la línea en recta).
  - Envergadura total entre centros externos (S1 - S4): 50 mm.
- **Huella física total:** 60 mm (de borde exterior de S1 a borde exterior de S4).
- **Márgenes laterales:** 27.5 mm libres a cada extremo respecto al borde exterior del chasis de 115 mm:
  $$\frac{115\text{ mm} - 60\text{ mm}}{2} = 27.5\text{ mm}$$

## Diagrama de Distribución Física

```text
|<--------------------------- Chasis (115 mm) --------------------------->|
|-- 27.5 mm --|                 50 mm span                 |-- 27.5 mm --|
              [S1: Ext Izq]     [S2: Int Izq]   [S3: Int Der]     [S4: Ext Der]
                 (Pin A0)          (Pin A1)        (Pin A2)         (Pin A3)
                    |                 |               |                 |
                    |<---- 19 mm ---->|<--- 12 mm --->|<---- 19 mm ---->|
                                      |===============|
                                        Línea (20 mm)
```

## Asignación de Pines

| Sensor | Función | Posición x | Pin Arduino Uno | Modo |
| :--- | :--- | :--- | :--- | :--- |
| **S1** | Exterior Izquierdo | -25 mm | `A0` (Digital) | `INPUT` |
| **S2** | Interior Izquierdo | -6 mm | `A1` (Digital) | `INPUT` |
| **S3** | Interior Derecho | +6 mm | `A2` (Digital) | `INPUT` |
| **S4** | Exterior Derecho | +25 mm | `A3` (Digital) | `INPUT` |

## Control en Dos Etapas por Discriminación Espacial

1. **Centrado (`[0, 1, 1, 0]`):** S2 y S3 sobre la línea. Crucero recto con aceleración progresiva (`BASE_SPEED` a `MAX_STRAIGHT_SPEED`).
2. **Etapa 1 - Desvío Leve (`[0, 1, 0, 0]` o `[0, 0, 1, 0]`):** Corrección suave hacia adelante; rueda externa a `BASE_SPEED`, rueda interna frenada a `CURVE_SPEED` hacia adelante.
3. **Etapa 2 - Curva Cerrada (`[1, 0, 0, 0]`, `[1, 1, 0, 0]`, `[0, 0, 0, 1]`, `[0, 0, 1, 1]`):** Detección inmediata en sensor exterior; contramarcha inmediata de rueda interna (`TURN_REVERSE_PWM`) sin retardo de tiempo.
4. **Marca Transversal / Largada (`[1, 1, 1, 1]`):** Todos activos; avance recto a `BASE_SPEED`.
5. **Pérdida de Pista (`[0, 0, 0, 0]`):** Inicio de ventana de rescate acotada a 200 ms hacia la última dirección activa antes de detención de emergencia.
