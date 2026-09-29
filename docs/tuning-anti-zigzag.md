# Guía de Tuning Anti-Zigzag — Velocista Gabriel

> Documento vivo: probá combinaciones, anotá resultados, iterá.

---

## Configuración actual (baseline v2)

```cpp
static const uint8_t BASE_SPEED = 115;
static const uint8_t TRIM_RIGHT = 20;
static const uint8_t TURN_REVERSE_PWM = 80;
static const uint8_t CURVE_SPEED = 40;
static const uint32_t REVERSE_ENGAGEMENT_DELAY_MS = 120;
```

**Comportamiento esperado**: corrección suave 120 ms antes de meter reversa, rueda interna a PWM 40 en Etapa 1, reversa moderada (80) en Etapa 2, velocidad base conservadora.

---

## Parámetros y sus efectos

| Parámetro | Qué controla | Subir = | Bajar = |
|-----------|--------------|---------|---------|
| `BASE_SPEED` | Velocidad en recta | Más rápido, menos estable | Más lento, más estable |
| `TRIM_RIGHT` | Compensación deriva derecha | Gira más a la izquierda | Gira más a la derecha |
| `TURN_REVERSE_PWM` | Fuerza reversa Etapa 2 | Giro más cerrado/agresivo | Giro más abierto/suave |
| `CURVE_SPEED` | Velocidad rueda interna Etapa 1 (freno suave) | Menos freno, corrección más suave | Más freno, corrección más brusca |
| `REVERSE_ENGAGEMENT_DELAY_MS` | Tiempo en Etapa 1 antes de pasar a reversa | Más tiempo suave, menos zigzag | Reacción más rápida, más zigzag |

---

## Matriz de pruebas sugeridas

### Serie A: Delay de engagement (efecto principal anti-zigzag)

| Test | `REVERSE_ENGAGEMENT_DELAY_MS` | `CURVE_SPEED` | `TURN_REVERSE_PWM` | `BASE_SPEED` | Notas |
|------|------------------------------|---------------|-------------------|--------------|-------|
| A1 (actual) | 120 | 40 | 80 | 115 | Baseline |
| A2 | **150** | 40 | 80 | 115 | Más zona suave |
| A3 | **100** | 40 | 80 | 115 | Reacción más rápida |
| A4 | **180** | 40 | 80 | 115 | Muy conservador |

### Serie B: Velocidad residual en Etapa 1 (suavizado)

| Test | `REVERSE_ENGAGEMENT_DELAY_MS` | `CURVE_SPEED` | `TURN_REVERSE_PWM` | `BASE_SPEED` | Notas |
|------|------------------------------|---------------|-------------------|--------------|-------|
| B1 | 120 | **20** | 80 | 115 | Casi frenado |
| B2 | 120 | **60** | 80 | 115 | Muy suave |
| B3 | 120 | **80** | 80 | 115 | Casi sin freno |

### Serie C: Agresividad reversa Etapa 2

| Test | `REVERSE_ENGAGEMENT_DELAY_MS` | `CURVE_SPEED` | `TURN_REVERSE_PWM` | `BASE_SPEED` | Notas |
|------|------------------------------|---------------|-------------------|--------------|-------|
| C1 | 120 | 40 | **70** | 115 | Reversa muy suave |
| C2 | 120 | 40 | **90** | 115 | Reversa más mordiente |
| C3 | 120 | 40 | **100** | 115 | Reversa fuerte |

### Serie D: Velocidad base (tradeoff velocidad/estabilidad)

| Test | `REVERSE_ENGAGEMENT_DELAY_MS` | `CURVE_SPEED` | `TURN_REVERSE_PWM` | `BASE_SPEED` | Notas |
|------|------------------------------|---------------|-------------------|--------------|-------|
| D1 | 120 | 40 | 80 | **100** | Muy estable, lento |
| D2 | 120 | 40 | 80 | **130** | Más rápido, arriesgado |
| D3 | 120 | 40 | 80 | **140** | Límite superior razonable |

### Serie E: Combinadas "prometedoras"

| Test | `REVERSE_ENGAGEMENT_DELAY_MS` | `CURVE_SPEED` | `TURN_REVERSE_PWM` | `BASE_SPEED` | Perfil |
|------|------------------------------|---------------|-------------------|--------------|--------|
| E1 | 150 | 60 | 70 | 110 | **Máxima estabilidad** |
| E2 | 120 | 50 | 85 | 120 | **Equilibrado** |
| E3 | 100 | 40 | 90 | 125 | **Deportivo** |
| E4 | 180 | 30 | 75 | 105 | **Curvas muy cerradas** |

---

## Cómo probar

1. Cambiá **un solo parámetro** a la vez (salvo en Serie E que son combinaciones validadas).
2. Compilá y subí al robot.
3. Probá en: recta larga, curva 90°, curva 180°, chicana, cruce transversal.
4. Anotá en la tabla de resultados abajo.

---

## Registro de resultados

| Test | Zigzag recta (1-5) | Curva 90° (1-5) | Curva 180° (1-5) | Cruce transversal | Comentarios |
|------|-------------------|-----------------|------------------|-------------------|-------------|
| A1 | | | | | Baseline actual |
| A2 | | | | | |
| A3 | | | | | |
| A4 | | | | | |
| B1 | | | | | |
| B2 | | | | | |
| B3 | | | | | |
| C1 | | | | | |
| C2 | | | | | |
| C3 | | | | | |
| D1 | | | | | |
| D2 | | | | | |
| D3 | | | | | |
| E1 | | | | | |
| E2 | | | | | |
| E3 | | | | | |
| E4 | | | | | |

**Escala zigzag**: 1 = perfecto (ni un bamboleo) → 5 = incontrolable
**Escala curvas**: 1 = pasa limpio, rápido → 5 = se sale / no gira lo suficiente

---

## Próximos pasos si nada alcanza

1. **Separación física sensores**: mover a 30-35 mm (requiere hardware).
2. **Agregar 3er sensor central**: lógica de 3 sensores (izq/centro/der) da zona muerta real.
3. **Filtro digital**: media móvil de 3-5 lecturas en `normalizeSensors()`.
4. **PID suave**: reemplazar lógica de 2 etapas por PID con derivada limitada.

---

## Límites máximos de velocidad con 2 sensores (straddle)

> **Con 2 sensores straddle no vas a pasar de ~130-135 PWM base sin zigzag.**  
> La geometría te obliga a corregir tarde y brusco. El PID no existe, tenés bang-bang con delay.

### Límites prácticos estimados

| Parámetro | Límite "seguro" | Límite "al filo" | Por qué |
|-----------|-----------------|------------------|---------|
| `BASE_SPEED` | **125-130** | **140-145** | A >130 la inercia gana: el robot detecta la desviación *después* de haberla recorrido |
| `REVERSE_ENGAGEMENT_DELAY_MS` | **100 ms** (mín) | **70 ms** (original) | Menos delay = corrección más temprana PERO más zigzag |
| `CURVE_SPEED` | **20-30** (mín para estabilidad) | **0** (freno total) | A alta velocidad necesitás *algo* de rueda interna girando |
| `TURN_REVERSE_PWM` | **85-90** | **100-110** | Reversa fuerte ayuda a cerrar curvas, pero sacude en recta |

### Config "máxima velocidad razonable" (2 sensores)

```cpp
static const uint8_t BASE_SPEED = 130;        // Límite superior realista
static const uint8_t TRIM_RIGHT = 20;
static const uint8_t TURN_REVERSE_PWM = 90;   // Reversa decidida para curvas
static const uint8_t CURVE_SPEED = 30;        // Ruido residual mínimo
static const uint32_t REVERSE_ENGAGEMENT_DELAY_MS = 90;  // Compromiso
```

**Esperable**: recta ~15-20% más rápida que baseline (115), **zigzag visible pero controlable** en recta perfecta. En pista sucia / vibraciones / batería baja → inestable.

### Si necesitás más velocidad: la única vía real

| Opción | Sensores | PWM base estable | Complejidad |
|--------|----------|------------------|-------------|
| Actual | 2 (straddle) | 125-130 | Baja |
| **+1 central** | 3 (centro + laterales) | **160-180** | Media (PID simple) |
| **+2 laterales** | 4 (máx reglamento) | **200+** | Media-alta (estado completo) |

Regla del campeonato: **máx 4 sensores** (Art 1.2.1).

---

## Notas de sesión

- **Fecha**: 2026-09-28
- **Hardware**: 2x TCRT5000 digitales, separación ~18 mm, línea 20 mm
- **Reglamento**: Curvas radio 30 cm, pista elevada, 4 sensores máx (Art 1.2.1)