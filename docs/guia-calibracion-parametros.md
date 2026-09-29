# Guía de Calibración de Parámetros y Diagnóstico de Comportamiento

Este documento detalla qué controla cada constante de navegación en [`LineFollowerController.h`](../include/LineFollowerController.h) y cómo diagnosticar y ajustar el robot según su comportamiento dinámico en pista.

---

## 1. Significado de cada Parámetro

### 1. `BASE_SPEED` (PWM 0–255)
* **Qué controla:** Velocidad nominal de crucero en línea recta (cuando ambos sensores detectan pista limpia a los lados de la línea).
* **Efecto físico:** Determina la inercia del vehículo antes de encontrar una perturbación o entrar en una curva.
* **Impacto del ajuste:**
  * **Subir:** Mayor velocidad y mejor tiempo de vuelta, pero la inercia puede superar la capacidad de frenado/reacción y desatar oscilaciones (zigzag).
  * **Bajar:** Mayor adherencia y estabilidad, pero rendimiento más lento.

### 2. `TRIM_RIGHT` (PWM 0–50 aprox.)
* **Qué controla:** Compensación de desviación en recta (`rightPwm = currentSpeed - TRIM_RIGHT`).
* **Efecto físico:** Nivela disparidades de fabricación entre ambos motores de corriente continua o desequilibrios de rozamiento mecánico.
* **Impacto del ajuste:**
  * **Subir:** El motor derecho gira más lento; el robot corrige inclinándose más hacia la **derecha**.
  * **Bajar (o llevar a 0):** El motor derecho recibe igual o mayor potencia; el robot corrige inclinándose más hacia la **izquierda**.

### 3. `CURVE_SPEED` (PWM 0–255, típicamente 0 a 60)
* **Qué controla:** Velocidad de avance de la **rueda interna** durante la **Etapa 1** de giro (amortiguación inicial).
* **Efecto físico:** Actúa como freno dinámico suave. Mientras la rueda externa empuja a velocidad de crucero, la rueda interna reduce su marcha a este valor sin invertir el sentido de giro.
* **Impacto del ajuste:**
  * **Subir (ej. 50–60):** Giro muy suave y progresivo; ideal para absorber pequeñas imperfecciones sin cabecear, pero gira poco en curvas cerradas.
  * **Bajar (ej. 0–20):** Freno casi total en la rueda interna. Permite pivotar con más rapidez, pero correcciones leves pueden sacudir el chasis.

### 4. `REVERSE_ENGAGEMENT_DELAY_MS` (Milisegundos)
* **Qué controla:** Tiempo de permanencia en **Etapa 1** antes de autorizar la contramarcha (reversa) en la rueda interna.
* **Efecto físico:** Filtro temporal anti-zigzag. Distingue entre un desvío o ruido pasajero (vibración) y una curva real y persistente de 90° o 180°.
* **Impacto del ajuste:**
  * **Subir (ej. 120–160 ms):** Mayor zona de amortiguación suave. Elimina el zigzag en recta, pero en curvas cerradas puede demorar la reversa y salirse de pista.
  * **Bajar (ej. 60–80 ms):** Reacción instantánea ante curvas; gran agilidad en chicanas, pero puede volverse inestable y oscilar en rectas.

### 5. `TURN_REVERSE_PWM` (PWM 0–255, típicamente 70 a 100)
* **Qué controla:** Potencia de contramarcha de la rueda interna durante la **Etapa 2** (giro cerrado sostenido).
* **Efecto físico:** Provoca un par de giro sobre el centro del chasis (rueda externa hacia adelante, rueda interna en reversa).
* **Impacto del ajuste:**
  * **Subir (ej. 90–110):** Giro muy cerrado y agresivo sobre el propio eje, pero disipa velocidad longitudinal rápidamente.
  * **Bajar (ej. 70–80):** Giro más fluido con menos pérdida de velocidad hacia adelante, pero con mayor radio de curvatura.

---

## 2. Aceleración Progresiva en Rectas

Parámetros introducidos en la evolución del controlador para aprovechar rectas largas sin comprometer la entrada a curva:

* **`MAX_STRAIGHT_SPEED` (ej. 180):** Velocidad máxima a la que puede llegar el robot si permanece centrado en una recta prolongada.
* **`STRAIGHT_ACCEL_DELAY_MS` (ej. 150 ms):** Tiempo que debe transcurrir rodando perfectamente centrado antes de comenzar la rampa de aceleración.
* **`ACCEL_STEP_PWM` (+5 PWM) / `ACCEL_STEP_INTERVAL_MS` (50 ms):** Pendiente de la rampa de aceleración (aumenta 5 PWM cada 50 ms transcurridos después del delay inicial).
* **Reseteo instantáneo:** En el momento exacto en que cualquiera de los sensores detecta desviación (inicio de curva), la velocidad vuelve de inmediato a `BASE_SPEED`, garantizando una frenada previa al giro.

---

## 3. Matriz de Diagnóstico y Ajuste en Pista

| Comportamiento observado | Diagnóstico primario | Acción correctiva recomendada |
| :--- | :--- | :--- |
| **Bamboleo / Zigzag en recta** | La contramarcha se activa ante perturbaciones leves o el frenado en Etapa 1 es muy brusco. | 1. **Subir** `REVERSE_ENGAGEMENT_DELAY_MS` (+20 a +30 ms).<br>2. **Subir** `CURVE_SPEED` (ej. subir a 40–50) para suavizar la Etapa 1. |
| **Se sale de pista en curvas de 90° o 180°** | Tarda demasiado en entrar en reversa o la fuerza de giro es insuficiente. | 1. **Bajar** `REVERSE_ENGAGEMENT_DELAY_MS` (-20 ms).<br>2. **Subir** `TURN_REVERSE_PWM` (+10 PWM).<br>3. Si persiste, **bajar** `BASE_SPEED`. |
| **En recta se desvía progresivamente hacia la izquierda** | El motor derecho empuja más que el motor izquierdo. | **Subir** `TRIM_RIGHT` (+2 a +5 PWM). |
| **En recta se desvía progresivamente hacia la derecha** | El motor izquierdo empuja más que el motor derecho. | **Bajar** `TRIM_RIGHT` (acercar hacia 0). |
| **Gira de golpe y pierde la línea hacia el lado contrario** | Sobreviraje por exceso de contramarcha en Etapa 2. | 1. **Bajar** `TURN_REVERSE_PWM` (-10 PWM).<br>2. **Subir** `CURVE_SPEED` para dar una transición más progresiva. |
| **Frenazos violentos al rozar la línea en recta** | La Etapa 1 bloquea totalmente la rueda interna. | Asegurar `CURVE_SPEED` >= 30 y verificar que `REVERSE_ENGAGEMENT_DELAY_MS` sea al menos 100 ms. |
| **Pierde el control al final de rectas largas** | La aceleración progresiva acumula demasiada inercia antes de la curva. | **Bajar** `MAX_STRAIGHT_SPEED` (ej. de 180 a 150) o **aumentar** `STRAIGHT_ACCEL_DELAY_MS`. |

---

## 4. Metodología de Calibración Recomendada

1. **Aislar variables:** Modificar siempre **un solo parámetro por iteración**, realizar varias pasadas en pista y anotar el resultado.
2. **Orden prioritario de ajuste:**
   ```
   1. TRIM_RIGHT                 (Garantizar trayectoria recta limpia)
          │
          ▼
   2. REVERSE_ENGAGEMENT_DELAY   (Eliminar oscilaciones y zigzag)
          │
          ▼
   3. TURN_REVERSE_PWM / CURVE   (Asegurar trazada en curvas cerradas)
          │
          ▼
   4. BASE_SPEED & ACCEL         (Maximizar velocidad competitiva)
   ```
