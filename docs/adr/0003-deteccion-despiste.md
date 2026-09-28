# Detección de Despiste y Ventana de Rescate

Decidimos que ante la pérdida simultánea de línea en ambos sensores (despiste), el robot ejecuta una ventana de rescate acotada (150-200 ms) girando hacia el último lado conocido antes de aplicar el frenado de emergencia (`detenerMotores()`), protegiendo al robot de invadir el carril del oponente (Art. 6.3.4) sin penalizar micro-desvíos corregibles (Art. 6.2.9).

## Comportamiento

1. **Estado Centrado**: Ambos sensores sobre la línea (avance recto).
2. **Corrección Progresiva**: Un sensor fuera de línea (giro diferencial hacia el sensor activo).
3. **Despiste**: Ambos sensores en fondo.
   - Si el tiempo desde la pérdida es menor a 200 ms: mantiene giro hacia el último sensor que vio la línea.
   - Si expira el tiempo sin recuperar la línea: detención total inmediata de ambos motores.
