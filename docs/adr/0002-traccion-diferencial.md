# Tracción y Giro Diferencial Progresivo

Decidimos implementar giro diferencial hacia adelante (reduciendo el ciclo de trabajo PWM de la rueda interior sin invertir el sentido de rotación del motor) para preservar la inercia del robot en pista, erradicar oscilaciones bruscas y prevenir la rotura de los engranajes plásticos de los motores amarillos exigidos por el reglamento (Art. 1.2.1).

## Consideraciones

- **Velocidad base**: Aplicada a la rueda exterior durante curvas o a ambas ruedas en tramo recto.
- **Velocidad interna en curva**: Reducción controlada (30-40% del ciclo PWM) sin freno motor agresivo ni contramarcha.
- **Contramarcha / Reversa**: Prohibida en la rutina normal de navegación; solo admisible como maniobra extrema de detención ante despiste total.
