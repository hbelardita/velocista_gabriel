#ifndef LINE_FOLLOWER_CONTROLLER_H
#define LINE_FOLLOWER_CONTROLLER_H

#include <stdint.h>

struct SensorInputs {
    bool leftDetected;
    bool rightDetected;
};

struct MotorOutputs {
    uint8_t leftPwm;
    uint8_t rightPwm;
    bool leftForward;
    bool rightForward;
};

struct ControllerOutputs {
    MotorOutputs motors;
    bool indicatorActive;
    bool isStopped;
};

enum class RobotState {
    STANDBY,
    RACING,
    RESCUING,
    EMERGENCY_STOP
};

enum class TrackPolarity {
    WHITE_LINE, // Línea blanca sobre fondo oscuro (Sensor activo = LOW / 0)
    BLACK_LINE  // Línea negra sobre fondo blanco (Sensor activo = HIGH / 1)
};

class LineFollowerController {
public:
    enum class ActiveTurn {
        NONE,
        LEFT,
        RIGHT
    };

    static const uint8_t BASE_SPEED = 115;
    static const uint8_t TRIM_RIGHT = 20;
    static const uint8_t TURN_REVERSE_PWM = 80; // Contramarcha en rueda interna para giro cerrado sobre su eje
    static const uint8_t CURVE_SPEED = 40;
    static const uint32_t REVERSE_ENGAGEMENT_DELAY_MS = 120;

    explicit LineFollowerController(TrackPolarity polarity = TrackPolarity::WHITE_LINE)
        : state_(RobotState::STANDBY),
          wasButtonPressed_(false),
          polarity_(polarity),
          activeTurn_(ActiveTurn::NONE),
          turnStartTimeMs_(0) {
        outputs_.motors.leftPwm = 0;
        outputs_.motors.rightPwm = 0;
        outputs_.motors.leftForward = true;
        outputs_.motors.rightForward = true;
        outputs_.indicatorActive = false;
        outputs_.isStopped = true;
    }

    void setTrackPolarity(TrackPolarity polarity) {
        polarity_ = polarity;
    }

    TrackPolarity getTrackPolarity() const {
        return polarity_;
    }

    ActiveTurn getActiveTurn() const {
        return activeTurn_;
    }

    // Normalización de sensores según la polaridad de la pista sin duplicar lógica
    static SensorInputs normalizeSensors(int rawLeft, int rawRight, TrackPolarity polarity) {
        int activeLevel = (polarity == TrackPolarity::WHITE_LINE) ? 0 : 1;
        return SensorInputs{
            rawLeft == activeLevel,
            rawRight == activeLevel
        };
    }

    // Actualización del ciclo de control
    // Cumple con el Reglamento de Carreras:
    // - Art 2.2.3 & 4.1.1: El robot permanece inmóvil en STANDBY mientras se mantiene presionado el pulsador de largada.
    // - Art 2.2.7 & 4.1.2: Al soltarse el pulsador, pasa inmediatamente a RACING y enciende el indicador luminoso.
    // - Topología a horcajadas (línea en el medio):
    //   * !left && !right: centrado en recta -> avance recto a BASE_SPEED con trim.
    //   * left && right: cruce transversal -> avance recto continuo.
    //   * Control de giro en 2 etapas (anti-zigzag y curvas cerradas):
    //     - Etapa 1 (< 70 ms): frenado suave forward (rueda interna PWM CURVE_SPEED = 0).
    //     - Etapa 2 (>= 70 ms persistente): contramarcha en reversa (rueda interna PWM TURN_REVERSE_PWM = 90).
    void update(bool buttonPressed, const SensorInputs& sensors, uint32_t currentTimeMs) {
        if (state_ == RobotState::STANDBY) {
            if (buttonPressed) {
                wasButtonPressed_ = true;
            } else if (wasButtonPressed_) {
                // El pulsador fue presionado y ahora es liberado -> inicio inmediato de Rutina de Carrera
                state_ = RobotState::RACING;
                activeTurn_ = ActiveTurn::NONE;
                turnStartTimeMs_ = 0;
                outputs_.indicatorActive = true;
                outputs_.isStopped = false;
            }
        }

        if (state_ == RobotState::RACING) {
            outputs_.indicatorActive = true;
            outputs_.isStopped = false;

            if (!sensors.leftDetected && !sensors.rightDetected) {
                // Centrado: ambos sensores en fondo (línea en el medio) -> avance recto
                activeTurn_ = ActiveTurn::NONE;
                outputs_.motors.leftForward = true;
                outputs_.motors.rightForward = true;
                outputs_.motors.leftPwm = BASE_SPEED;
                outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
            } else if (!sensors.leftDetected && sensors.rightDetected) {
                // Curva a la derecha / desvío a la izquierda
                if (activeTurn_ != ActiveTurn::RIGHT) {
                    activeTurn_ = ActiveTurn::RIGHT;
                    turnStartTimeMs_ = currentTimeMs;
                }
                uint32_t elapsed = currentTimeMs - turnStartTimeMs_;
                if (elapsed < REVERSE_ENGAGEMENT_DELAY_MS) {
                    // Etapa 1 (corrección suave / anti-zigzag): rueda interna frenada hacia adelante
                    outputs_.motors.leftForward = true;
                    outputs_.motors.rightForward = true;
                    outputs_.motors.leftPwm = BASE_SPEED;
                    outputs_.motors.rightPwm = CURVE_SPEED;
                } else {
                    // Etapa 2 (curva cerrada prolongada): rueda interna en reversa para giro sobre su eje
                    outputs_.motors.leftForward = true;
                    outputs_.motors.rightForward = false;
                    outputs_.motors.leftPwm = BASE_SPEED;
                    outputs_.motors.rightPwm = TURN_REVERSE_PWM;
                }
            } else if (sensors.leftDetected && !sensors.rightDetected) {
                // Curva a la izquierda / desvío a la derecha
                if (activeTurn_ != ActiveTurn::LEFT) {
                    activeTurn_ = ActiveTurn::LEFT;
                    turnStartTimeMs_ = currentTimeMs;
                }
                uint32_t elapsed = currentTimeMs - turnStartTimeMs_;
                if (elapsed < REVERSE_ENGAGEMENT_DELAY_MS) {
                    // Etapa 1 (corrección suave / anti-zigzag): rueda interna frenada hacia adelante
                    outputs_.motors.leftForward = true;
                    outputs_.motors.rightForward = true;
                    outputs_.motors.leftPwm = CURVE_SPEED;
                    outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
                } else {
                    // Etapa 2 (curva cerrada prolongada): rueda interna en reversa para giro sobre su eje
                    outputs_.motors.leftForward = false;
                    outputs_.motors.rightForward = true;
                    outputs_.motors.leftPwm = TURN_REVERSE_PWM;
                    outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
                }
            } else {
                // Cruce transversal / marca de largada (ambos sensores detectan línea) -> avance recto continuo
                activeTurn_ = ActiveTurn::NONE;
                outputs_.motors.leftForward = true;
                outputs_.motors.rightForward = true;
                outputs_.motors.leftPwm = BASE_SPEED;
                outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
            }
        }
    }

    void updateRaw(bool buttonPressed, int rawLeft, int rawRight, uint32_t currentTimeMs) {
        update(buttonPressed, normalizeSensors(rawLeft, rawRight, polarity_), currentTimeMs);
    }

    ControllerOutputs getOutputs() const {
        return outputs_;
    }

    RobotState getState() const {
        return state_;
    }

private:
    RobotState state_;
    bool wasButtonPressed_;
    TrackPolarity polarity_;
    ControllerOutputs outputs_;
    ActiveTurn activeTurn_;
    uint32_t turnStartTimeMs_;
};

#endif // LINE_FOLLOWER_CONTROLLER_H
