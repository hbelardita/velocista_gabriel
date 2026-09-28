# Configuración Inicial de Sensores de Línea y Camino de Mejora

Decidimos iniciar las pruebas con 2 sensores digitales TCRT5000 para validar tracción y control base, manteniendo documentada la arquitectura objetivo de 4 sensores individuales permitida por el reglamento (Art. 1.2.1) para mitigar el riesgo de despiste en curvas de 30 cm de radio (Art. 3.5) sobre pista elevada (Art. 3.4).

## Diagrama de Distribución Objetivo (4 Sensores)

```text
       [S1: Ext Izq]     [S2: Int Izq]   [S3: Int Der]     [S4: Ext Der]
             |                 |               |                 |
             |<---- 18 mm ---->|<--- 12 mm --->|<---- 18 mm ---->|
                               |===============|
                                 Línea (20 mm)
```

## Diagrama de Distribución Inicial (2 Sensores)

```text
               [S_IZQ]                   [S_DER]
                  |                         |
                  |<------- 16-20 mm ------>|
                  |=========================|
                         Línea (20 mm)
```
